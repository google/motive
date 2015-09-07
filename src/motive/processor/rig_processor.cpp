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

RigInit::RigInit(const RigAnim& defining_anim,
                 const mathfu::mat4* bone_transforms,
                 const BoneIndex* bone_parents, BoneIndex num_bones)
    : MotivatorInit(kType),
      defining_anim_(&defining_anim),
      bone_transforms_(
          num_bones == 0 ? &MatrixInit::kIdentityTransform : bone_transforms) {
  // Ensure the animation and the mesh have the same hierarchy.
  // We allow the one exception where there are no bones and only popsicle
  // stick animations.
  assert((num_bones == 0 && defining_anim.NumBones() == 1) ||
         MatchesHierarchy(defining_anim, bone_parents, num_bones));
}

// static
bool RigInit::MatchesHierarchy(const BoneIndex* parents_a, BoneIndex len_a,
                               const BoneIndex* parents_b, BoneIndex len_b) {
  // TODO: Implement runtime retargetting by allowing the hiearchy to be
  //       slightly different on bones that aren't animated.
  return len_a == len_b &&
         memcmp(parents_a, parents_b, len_a * sizeof(parents_a[0])) == 0;
}

// static
bool RigInit::MatchesHierarchy(const RigAnim& anim, const BoneIndex* parents_b,
                               BoneIndex len_b) {
  return MatchesHierarchy(anim.bone_parents(), anim.NumBones(), parents_b,
                          len_b);
}

// static
bool RigInit::MatchesHierarchy(const RigAnim& anim_a, const RigAnim& anim_b) {
  return MatchesHierarchy(anim_a.bone_parents(), anim_a.NumBones(),
                          anim_b.bone_parents(), anim_b.NumBones());
}

class RigData {
 public:
  explicit RigData(const RigInit& init, MotiveTime start_time,
                   MotiveEngine* engine)
      : motivators_(init.defining_anim().NumBones()),
        global_transforms_(init.defining_anim().NumBones()),
        defining_anim_(&init.defining_anim()),
        end_time_(start_time) {
    const BoneIndex num_bones = defining_anim_->NumBones();

    // Initialize global transforms to default pose.
    // These will get overridden the first time AdvanceFrame() is called, but
    // we initialize them nicely anyway.
    memcpy(&global_transforms_[0], init.bone_transforms(),
           sizeof(global_transforms_[0]) * num_bones);

    // Initialize the motivators that drive the local transforms.
    for (BoneIndex i = 0; i < num_bones; ++i) {
      const MatrixOpArray& ops = defining_anim_->Anim(i).ops();
      const MatrixInit matrix_init(ops, global_transforms_[i]);
      motivators_[i].Initialize(matrix_init, engine);
    }
  }

  void BlendToAnim(const RigAnim& anim, MotiveTime start_time) {
    end_time_ = start_time + anim.end_time();

    // When animation has only one bone, or mesh has only one bone,
    // we simply animate the root node only.
    const int anim_num_bones = anim.NumBones();
    const int defining_num_bones = defining_anim_->NumBones();
    assert(defining_num_bones == 1 || anim_num_bones == 1 ||
           RigInit::MatchesHierarchy(anim, *defining_anim_));

    // Update the motivators to blend to our new values.
    for (BoneIndex i = 0; i < defining_num_bones; ++i) {
      const MatrixOpArray& ops =
          i >= anim_num_bones ? kEmptyOps : anim.Anim(i).ops();
      motivators_[i].BlendToOps(ops);
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
  /// global transforms. The `parents` are layed out such that the parent
  /// always come before the child.
  // TODO OPT: optimize `parents` layout so that we can parallelize this call.
  void CalculateGlobalTransforms(mat4* out) const {
    const BoneIndex* parents = defining_anim_->bone_parents();
    const int num_bones = NumBones();
    for (int i = 0; i < num_bones; ++i) {
      const mat4& local_transform = motivators_[i].Value();
      const int parent_idx = parents[i];
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
  const RigAnim* defining_anim_;

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

  virtual void BlendToAnim(MotiveIndex index, const RigAnim& anim) {
    Data(index).BlendToAnim(anim, time_);
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
