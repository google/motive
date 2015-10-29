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

#include <sstream>

#include "mathfu/constants.h"
#include "motive/anim.h"
#include "motive/engine.h"
#include "motive/init.h"
#include "motive/math/angle.h"
#include "motive/math/bulk_spline_evaluator.h"

using fpl::Angle;
using fpl::kPi;
using mathfu::AffineTransform;
using mathfu::mat4;
using mathfu::vec4;

namespace motive {

// static
mathfu::mat4 MatrixInit::kIdentityTransform(mathfu::mat4::Identity());
static const MatrixOpArray kEmptyOps(0);

class RigData {
 public:
  explicit RigData(const RigInit& init, MotiveTime start_time,
                   MotiveEngine* engine)
      : motivators_(nullptr),
        global_transforms_(nullptr),
        defining_anim_(&init.defining_anim()),
        current_anim_(nullptr),
        end_time_(start_time) {
    const BoneIndex num_bones = defining_anim_->NumBones();

    // Visual Studio 2010 does not like std::vectors of mat4, since they are
    // a 16-byte aligned type. Use plain old arrays instead.
    motivators_ = new MatrixMotivator4f[num_bones];
    global_transforms_ = new AffineTransform[num_bones];

    // Initialize the motivators that drive the local transforms.
    for (BoneIndex i = 0; i < num_bones; ++i) {
      const MatrixOpArray& ops = defining_anim_->Anim(i).ops();
      const MatrixInit matrix_init(
          ops, mat4::FromAffineTransform(init.bone_transforms()[i]));
      motivators_[i].Initialize(matrix_init, engine);
    }

    // Initialize global transforms to default pose.
    // These will get overridden the first time AdvanceFrame() is called, but
    // we initialize them nicely anyway.
    UpdateGlobalTransforms();
  }

  ~RigData() {
    delete[] motivators_;
    motivators_ = nullptr;

    delete[] global_transforms_;
    global_transforms_ = nullptr;
  }

  void BlendToAnim(const RigAnim& anim, const fpl::SplinePlayback& playback,
                   MotiveTime start_time) {
    end_time_ = start_time + anim.end_time();

    // When animation has only one bone, or mesh has only one bone,
    // we simply animate the root node only.
    const int anim_num_bones = anim.NumBones();
    const int defining_num_bones = NumBones();
    assert(defining_num_bones == 1 || anim_num_bones == 1 ||
           RigInit::MatchesHierarchy(anim, *defining_anim_));

    // Update the motivators to blend to our new values.
    for (BoneIndex i = 0; i < defining_num_bones; ++i) {
      const MatrixOpArray& ops =
          i >= anim_num_bones ? kEmptyOps : anim.Anim(i).ops();
      motivators_[i].BlendToOps(ops, playback);
    }

    // Remember the currently playing animation, for debugging purposes.
    current_anim_ = &anim;
  }

  void UpdateGlobalTransforms() {
    CalculateGlobalTransforms(global_transforms_);
  }

  const AffineTransform* GlobalTransforms() const { return global_transforms_; }

  BoneIndex NumBones() const {
    return defining_anim_->NumBones();
  }

  MotiveTime end_time() const { return end_time_; }

  const RigAnim* defining_anim() const { return defining_anim_; }

  void ChildValuesForDebugging(std::vector<float>* values) const {
    values->resize(defining_anim_->NumOps());

    int k = 0;
    const int defining_num_bones = NumBones();
    for (BoneIndex i = 0; i < defining_num_bones; ++i) {
      const MotiveChildIndex num_children = motivators_[i].NumChildren();
      for (MotiveChildIndex j = 0; j < num_children; ++j) {
        (*values)[k++] = motivators_[i].ChildValue1f(j);
      }
    }
  }

  std::string CsvHeaderForDebugging() const {
    std::ostringstream oss;
    oss << "animation name,time," << defining_anim_->CsvHeaderForDebugging(0)
        << std::endl;
    oss << ",," << defining_anim_->CsvHeaderForDebugging(1) << std::endl;
    return oss.str();
  }

  std::string CsvValuesForDebugging(MotiveTime current_time) const {
    std::vector<float> values;
    ChildValuesForDebugging(&values);

    std::ostringstream oss;
    const MotiveTime anim_time =
        current_time - end_time_ + current_anim_->end_time();
    oss << current_anim_->anim_name() << ',' << anim_time << ',';

    int k = 0;
    const int defining_num_bones = NumBones();
    for (BoneIndex i = 0; i < defining_num_bones; ++i) {
      const MatrixOpArray::OpVector& ops = defining_anim_->Anim(i).ops().ops();
      for (size_t j = 0; j < ops.size(); ++j) {
        const float multiplier = RotateOp(ops[j].type) ? 180.0f / kPi : 1.0f;
        oss << multiplier * values[k] << ',';
        k++;
      }
    }

    return oss.str();
  }

 private:
  /// Traverse hierarchy, converting local transforms from `motivators_` into
  /// global transforms. The `parents` are layed out such that the parent
  /// always come before the child.
  // TODO OPT: optimize `parents` layout so that we can parallelize this call.
  void CalculateGlobalTransforms(AffineTransform* out) const {
    const BoneIndex* parents = defining_anim_->bone_parents();
    const int num_bones = NumBones();
    for (int i = 0; i < num_bones; ++i) {
      const mat4& local_transform = motivators_[i].Value();
      const int parent_idx = parents[i];
      if (parent_idx == kInvalidBoneIdx) {
        out[i] = mat4::ToAffineTransform(local_transform);
      } else {
        assert(i > parent_idx);
        out[i] = mat4::ToAffineTransform(
            mat4::FromAffineTransform(out[parent_idx]) * local_transform);
      }
    }
  }

  MatrixMotivator4f* motivators_;
  AffineTransform* global_transforms_;
  const RigAnim* defining_anim_;
  const RigAnim* current_anim_;

  /// Time that the animation is expected to complete.
  MotiveTime end_time_;
};

// See comments on RigInit for details on this class.
class MotiveRigProcessor : public RigProcessor {
 public:
  MotiveRigProcessor() : time_(0) {}

  virtual ~MotiveRigProcessor() {
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

  virtual void BlendToAnim(MotiveIndex index, const RigAnim& anim,
                           const fpl::SplinePlayback& playback) {
    Data(index).BlendToAnim(anim, playback, time_);
  }

  virtual MotivatorType Type() const { return RigInit::kType; }
  virtual int Priority() const { return 3; }

  virtual const AffineTransform* GlobalTransforms(MotiveIndex index) const {
    return Data(index).GlobalTransforms();
  }

  virtual MotiveTime TimeRemaining(MotiveIndex index) const {
    const MotiveTime end_time = Data(index).end_time();
    return end_time == kMotiveTimeEndless ? kMotiveTimeEndless
                                          : end_time - time_;
  }

  virtual const RigAnim* DefiningAnim(MotiveIndex index) const {
    return Data(index).defining_anim();
  }

  virtual std::string CsvHeaderForDebugging(MotiveIndex index) const {
    return Data(index).CsvHeaderForDebugging();
  }

  virtual std::string CsvValuesForDebugging(MotiveIndex index) const {
    return Data(index).CsvValuesForDebugging(time_);
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

MOTIVE_INSTANCE(RigInit, MotiveRigProcessor);

}  // namespace motive
