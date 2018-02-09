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

#ifndef MOTIVE_RIG_DATA_H_
#define MOTIVE_RIG_DATA_H_

#include <iomanip>
#include <sstream>

#include "mathfu/constants.h"
#include "motive/engine.h"
#include "motive/math/angle.h"
#include "motive/math/bulk_spline_evaluator.h"
#include "motive/matrix_init.h"
#include "motive/matrix_motivator.h"
#include "motive/rig_anim.h"
#include "motive/rig_init.h"

namespace motive {

// static
static const std::vector<MatrixOperationInit> kEmptyOps;

class RigData {
 public:
  explicit RigData(const RigInit& init, MotiveTime start_time,
                   MotiveEngine* engine)
      : defining_anim_(&init.defining_anim()),
        current_anim_(nullptr),
        end_time_(start_time) {
    const BoneIndex num_bones = defining_anim_->NumBones();

    motivators_.resize(num_bones);
    global_transforms_.resize(num_bones);

    // Initialize the motivators that drive the local transforms.
    for (BoneIndex i = 0; i < num_bones; ++i) {
      const std::vector<MatrixOperationInit>& ops =
          defining_anim_->Anim(i).ops();
      motivators_[i].Initialize(MatrixInit(ops), engine);
    }

    // Initialize global transforms to default pose.
    // These will get overridden the first time AdvanceFrame() is called, but
    // we initialize them nicely anyway.
    UpdateGlobalTransforms();
  }

  ~RigData() {
  }

  void BlendToAnim(const RigAnim& anim, const motive::SplinePlayback& playback,
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
      const std::vector<MatrixOperationInit>& ops =
          i >= anim_num_bones ? kEmptyOps : anim.Anim(i).ops();
      motivators_[i].BlendToOps(ops, playback);
    }

    // Remember the currently playing animation, for debugging purposes.
    current_anim_ = &anim;
  }

  const RigAnim* current_anim() const { return current_anim_; }

  void SetPlaybackRate(float playback_rate) {
    // Update the motivators to have the new playback rate.
    // TODO: Do this in bulk.
    const int defining_num_bones = NumBones();
    for (BoneIndex i = 0; i < defining_num_bones; ++i) {
      motivators_[i].SetPlaybackRate(playback_rate);
    }
  }

  MotiveTime TimeRemaining() const {
    if (end_time_ == kMotiveTimeEndless) {
      return kMotiveTimeEndless;
    }
    MotiveTime time = 0;
    const int defining_num_bones = NumBones();
    for (BoneIndex i = 0; i < defining_num_bones; ++i) {
      time = std::max(time, motivators_[i].TimeRemaining());
    }
    return time;
  }

  void UpdateGlobalTransforms() {
    CalculateGlobalTransforms(global_transforms_.data());
  }

  const mathfu::AffineTransform* GlobalTransforms() const {
    return global_transforms_.data();
  }

  BoneIndex NumBones() const { return defining_anim_->NumBones(); }

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
      const std::vector<MatrixOperationInit>& ops =
          defining_anim_->Anim(i).ops();
      for (size_t j = 0; j < ops.size(); ++j) {
        const float multiplier = RotateOp(ops[j].type) ? 180.0f / kPi : 1.0f;
        oss << multiplier * values[k] << ',';
        k++;
      }
    }

    return oss.str();
  }

  std::string LocalTransformsForDebugging(BoneIndex bone,
                                          MotiveTime time) const {
    const BoneIndex* bone_parents = defining_anim_->bone_parents();

    // Output four lines: one per row of matrix.
    std::ostringstream oss;
    oss.precision(2);
    oss << std::fixed << std::right;

    // Output header
    const MotiveTime time_until_end = end_time_ - time;
    const MotiveTime time_since_start =
        current_anim_->end_time() - time_until_end;
    oss << current_anim_->anim_name() << " at time " << time_since_start << " ("
        << (time_since_start * 24.0f / 1000.0f) << " @24fps)" << std::endl;
    for (BoneIndex idx = bone; idx != kInvalidBoneIdx;
         idx = bone_parents[idx]) {
      // Output the bone's name.
      const char* bone_name = defining_anim_->BoneName(idx);
      oss << bone_name << std::endl;

      // Output the bone's matrix.
      const mathfu::mat4& m = motivators_[idx].Value();
      for (int row = 0; row < 3; ++row) {
        oss << "  (" << std::setw(7) << m(row, 0) << std::setw(7) << m(row, 1)
            << std::setw(7) << m(row, 2) << std::setw(7) << m(row, 3) << ')'
            << std::endl;
      }

      // Output the operations on this bone.
      const std::vector<MatrixOperationInit>& ops =
          current_anim_->Anim(idx).ops();
      oss << "  ";
      for (size_t i = 0; i < ops.size(); ++i) {
        const float multiplier = RotateOp(ops[i].type) ? 180.0f / kPi : 1.0f;
        const float value =
            multiplier *
            motivators_[idx].ChildValue1f(static_cast<MotiveChildIndex>(i));
        oss << MatrixOpName(ops[i].type) << "=" << value;
        if (i < ops.size() - 1) {
          oss << ", ";
        } else {
          oss << std::endl;
        }
      }
      oss << std::endl;
    }
    return oss.str();
  }

 private:
  /// Traverse hierarchy, converting local transforms from `motivators_` into
  /// global transforms. The `parents` are layed out such that the parent
  /// always come before the child.
  // TODO OPT: optimize `parents` layout so that we can parallelize this call.
  void CalculateGlobalTransforms(mathfu::AffineTransform* out) const {
    const BoneIndex* parents = defining_anim_->bone_parents();
    const int num_bones = NumBones();
    for (int i = 0; i < num_bones; ++i) {
      // TODO: Return an AffineTransform from the MatrixMotivator.
      const mathfu::mat4& local_transform = motivators_[i].Value();
      const int parent_idx = parents[i];
      if (parent_idx == kInvalidBoneIdx) {
        out[i] = mathfu::mat4::ToAffineTransform(local_transform);
      } else {
        assert(i > parent_idx);
        out[i] = mathfu::mat4::ToAffineTransform(
            mathfu::mat4::FromAffineTransform(out[parent_idx]) *
            local_transform);
      }
    }
  }

  std::vector<MatrixMotivator4f, mathfu::simd_allocator<MatrixMotivator4f>>
      motivators_;
  std::vector<mathfu::AffineTransform,
              mathfu::simd_allocator<mathfu::AffineTransform>>
      global_transforms_;
  const RigAnim* defining_anim_;
  const RigAnim* current_anim_;

  /// Time that the animation is expected to complete.
  MotiveTime end_time_;
};

}  // namespace motive

#endif  // MOTIVE_RIG_DATA_H_
