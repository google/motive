// Copyright 2014 Google Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mathfu/constants.h"
#include "motive/anim.h"
#include "motive/engine.h"
#include "motive/init.h"
#include "motive/math/angle.h"
#include "motive/math/bulk_spline_evaluator.h"

using mathfu::vec4;
using mathfu::mat4;
using fpl::Angle;

namespace motive {

// static
mathfu::mat4 MatrixInit::kIdentityTransform(mathfu::mat4::Identity());
static const MatrixOpArray kEmptyOps(0);

RigInit::RigInit(const RigAnim& anim, const mat4* bone_transforms,
                 const BoneIndex* bone_parents, BoneIndex num_bones)
    : MotivatorInit(kType),
      anim_(&anim),
      bone_parents_(bone_parents),
      bone_transforms_(bone_transforms),
      num_bones_(num_bones) {
  // The `bone` parameters come from the mesh. If the mesh has no skeleton,
  // we apply just the root transform of `anim`.
  const bool mesh_has_no_skeleton = num_bones == 0;
  if (mesh_has_no_skeleton) {
    bone_parents_ = &kInvalidBoneIdx;
    bone_transforms_ = &MatrixInit::kIdentityTransform;
    num_bones_ = 1;
  } else {
    // Ensure the animation and the mesh have the same hierarchy.
    assert(anim.MatchesHierarchy(bone_parents, num_bones));
  }
}

class RigData {
 public:
  explicit RigData(const RigInit& init, MotiveTime start_time,
                   MotiveEngine* engine)
      : motivators_(init.num_bones()),
        global_transforms_(init.num_bones()),
        parents_(init.bone_parents()),
        end_time_(start_time + init.anim().end_time()) {
    const RigAnim& rig_anim = init.anim();
    const mat4* bone_transforms = init.bone_transforms();
    const int num_anim_bones = rig_anim.NumBones();
    const int num_bones = init.num_bones();
    assert(num_bones > 0 && num_anim_bones > 0);

    // Initialize global transforms to default pose.
    // These will get overridden the first time AdvanceFrame() is called, but
    // we initialize them nicely anyway.
    memcpy(&global_transforms_[0], bone_transforms,
           sizeof(global_transforms_[0]) * num_bones);

    // Initialize the motivators that drive the local transforms.
    for (int i = 0; i < num_bones; ++i) {
      const MatrixOpArray& ops = i >= num_anim_bones ?
                                 kEmptyOps : rig_anim.Anim(i).ops();
      const MatrixInit matrix_init(ops, bone_transforms[i]);
      motivators_[i].Initialize(matrix_init, engine);
    }
  }

  void UpdateGlobalTransforms() {
    CalculateGlobalTransforms(&global_transforms_[0]);
  }

  const mat4* GlobalTransforms() const { return &global_transforms_[0]; }

  BoneIndex NumBones() const {
    return static_cast<BoneIndex>(global_transforms_.size());
  }

  MotiveTime end_time() const { return end_time_; }

 private:
  /// Traverse hierarchy, converting local transforms from `motivators_` into
  /// global transforms. The `parents_` are layed out such that the parent
  /// always come before the child.
  // TODO OPT: optimize `parents_` layout so that we can parallelize this call.
  void CalculateGlobalTransforms(mat4* out) const {
    const int num_bones = NumBones();
    for (int i = 0; i < num_bones; ++i) {
      const mat4& local_transform = motivators_[i].Value();
      const int parent_idx = parents_[i];
      if (parent_idx == kInvalidBoneIdx) {
        out[i] = local_transform;
      } else {
        assert(i > parent_idx);
        out[i] = out[parent_idx] * local_transform;
      }
    }
  }

  std::vector<MotivatorMatrix4f> motivators_;
  std::vector<mat4> global_transforms_;
  const BoneIndex* parents_;

  /// Time that the animation is expected to complete.
  MotiveTime end_time_;
};

// See comments on RigInit for details on this class.
class RigMotiveProcessor : public MotiveProcessorRig {
 public:
  RigMotiveProcessor() : time_(0) {}

  virtual ~RigMotiveProcessor() {
    const MotiveIndex num_indices = NumIndices();
    for (MotiveIndex index = 0; index < num_indices; ++index) {
      RemoveIndex(index);
    }
  }

  virtual void AdvanceFrame(MotiveTime delta_time) {
    Defragment();

    // Process the series of matrix operations for each index.
    const MotiveIndex num_indices = NumIndices();
    for (MotiveIndex index = 0; index < num_indices; ++index) {
      RigData& d = Data(index);
      d.UpdateGlobalTransforms();
    }

    // Update our global time. It shouldn't matter if this wraps
    // around, since we only calculate times relative to it.
    time_ += delta_time;
  }

  virtual MotivatorType Type() const { return RigInit::kType; }
  virtual int Priority() const { return 3; }

  virtual const mat4* GlobalTransforms(MotiveIndex index) const {
    return Data(index).GlobalTransforms();
  }

  virtual MotiveTime TimeRemaining(MotiveIndex index) const {
    const MotiveTime end_time = Data(index).end_time();
    return end_time == kMotiveTimeEndless ? kMotiveTimeEndless
                                          : end_time - time_;
  }

 protected:
  MotiveIndex NumIndices() const {
    return static_cast<MotiveIndex>(data_.size());
  }

  virtual void InitializeIndex(const MotivatorInit& init, MotiveIndex index,
                               MotiveEngine* engine) {
    RemoveIndex(index);
    auto rig_init = static_cast<const RigInit&>(init);
    data_[index] = new RigData(rig_init, time_, engine);
  }

  virtual void RemoveIndex(MotiveIndex index) {
    if (data_[index] != nullptr) {
      delete data_[index];
      data_[index] = nullptr;
    }
  }

  virtual void MoveIndex(MotiveIndex old_index, MotiveIndex new_index) {
    data_[new_index] = data_[old_index];
    data_[old_index] = nullptr;
  }

  virtual void SetNumIndices(MotiveIndex num_indices) {
    const MotiveIndex old_num_indices = NumIndices();

    // Ensure old items are deleted.
    for (MotiveIndex i = num_indices; i < old_num_indices; ++i) {
      RemoveIndex(i);
    }

    // Initialize new items to nullptr.
    data_.resize(num_indices, nullptr);
  }

  const RigData& Data(MotiveIndex index) const {
    assert(ValidIndex(index));
    return *data_[index];
  }

  RigData& Data(MotiveIndex index) {
    assert(ValidIndex(index));
    return *data_[index];
  }

  std::vector<RigData*> data_;
  MotiveTime time_;
};

MOTIVE_INSTANCE(RigInit, RigMotiveProcessor);

}  // namespace motive
