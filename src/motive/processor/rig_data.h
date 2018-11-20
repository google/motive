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
        root_motion_bone_(init.root_motion_bone()),
        root_motion_transform_(mathfu::AffineTransform::Identity()),
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

  ~RigData() {}

  void BlendToAnim(const RigAnim& anim, const motive::SplinePlayback& playback,
                   MotiveTime start_time) {
    end_time_ = start_time + anim.end_time();

    // When animation has only one bone, or mesh has only one bone,
    // we simply animate the root node only.
    const int anim_num_bones = anim.NumBones();
    const int defining_num_bones = NumBones();
    assert(defining_num_bones == 1 || anim_num_bones == 1 ||
           RigInit::MatchesHierarchy(anim, *defining_anim_));

    // TODO(b/111071408): instead of resizing back to a single animation, blend
    // each of the current animations to the new animation and then try to
    // collapse them when the transition is complete. For example, if A and B
    // are running with weights of .3 and .7, blend both to C, then when the
    // transition is complete, collapse .3C + .7C into just C.
    weights_.resize(1, 1.f);
    motivators_.resize(defining_num_bones);

    // Update the motivators to blend to our new values.
    for (BoneIndex i = 0; i < defining_num_bones; ++i) {
      const std::vector<MatrixOperationInit>& ops =
          i >= anim_num_bones ? kEmptyOps : anim.Anim(i).ops();
      motivators_[i].BlendToOps(ops, playback);
    }

    // Remember the currently playing animation, for debugging purposes.
    current_anim_ = &anim;
  }

  void BlendToAnims(const RigAnim** anims, const SplinePlayback* playbacks,
                    const float* weights, int count, MotiveEngine* engine,
                    MotiveTime start_time) {
    const int old_count = weights_.size();
    weights_.resize(count);

    MotiveTime max_time = 0;
    float total_weight = 0.f;
    for (int i = 0; i < count; ++i) {
      // Weights cannot be negative, but can be zero if the animation is
      // temporarily inactive but may be active in the near future.
      assert(weights[i] >= 0.f);
      total_weight += weights[i];
      max_time = std::max(max_time, anims[i]->end_time());
    }
    assert(total_weight > 0.f);
    end_time_ = start_time + max_time;

    // Allocate a MatrixMotivator per bone per animation.
    // TODO(b/111071408): We may need to be more careful about which motivators
    // might be removed when this array is resized in order to correctly handle
    // complex blending.
    const int defining_num_bones = NumBones();
    motivators_.resize(defining_num_bones * count);

    // Go through each of the new animations.
    for (int i = 0; i < count; ++i) {
      const RigAnim& anim = *anims[i];
      const SplinePlayback& playback = playbacks[i];
      const int base_index = BaseBoneIndex(i);

      // When the animation has only one bone or the mesh has only one bone, we
      // simply animate the root node only. Otherwise, the rig hierarchies must
      // match.
      const int anim_num_bones = anim.NumBones();
      assert(defining_num_bones == 1 || anim_num_bones == 1 ||
             RigInit::MatchesHierarchy(anim, *defining_anim_));

      // Set the weight.
      weights_[i] = weights[i] / total_weight;

      // Update all Motivators.
      for (BoneIndex j = 0; j < defining_num_bones; ++j) {
        const int index = base_index + j;

        // TODO(b/111071408): If there's more than 1 animation running, collapse
        // them into a single animation, then initialize the new ones to that.

        // If the Motivator was just created, it must be initialized. If there
        // was previously a single animation, duplicate it. Otherwise, use the
        // defining animation.
        if (i >= old_count) {
          if (old_count == 1) {
            motivators_[index].CloneFrom(&motivators_[j]);
          } else {
            const std::vector<MatrixOperationInit>& ops =
                defining_anim_->Anim(j).ops();
            motivators_[index].Initialize(MatrixInit(ops), engine);
          }
        }

        // Blend the Motivator to its new animation.
        const std::vector<MatrixOperationInit>& ops =
            j >= anim_num_bones ? kEmptyOps : anim.Anim(j).ops();
        motivators_[index].BlendToOps(ops, playback);
      }
    }

    // TODO(b/111080871): decide if it's worth storing all of the animations.
    // Arbitrarily remember the currently playing animation, for debugging
    // purposes.
    if (count > 0) {
      current_anim_ = anims[0];
    }
  }

  const RigAnim* current_anim() const { return current_anim_; }

  void SetPlaybackRate(float playback_rate) {
    // Update the motivators to have the new playback rate.
    // TODO: Do this in bulk.
    for (size_t i = 0; i < motivators_.size(); ++i) {
      motivators_[i].SetPlaybackRate(playback_rate);
    }
  }

  void SetPlaybackRates(const float* playback_rates, int count) {
    // Update the motivators to have the new playback rate.
    const int defining_num_bones = NumBones();
    for (int i = 0; i < count; ++i) {
      const int base_index = BaseBoneIndex(i);
      const float playback_rate = playback_rates[i];
      for (BoneIndex j = 0; j < defining_num_bones; ++j) {
        motivators_[base_index + j].SetPlaybackRate(playback_rate);
      }
    }
  }

  void SetWeights(const float* weights, int count) {
    float total_weight = 0.f;
    for (int i = 0; i < count; ++i) {
      assert(weights[i] >= 0.f);
      total_weight += weights[i];
    }
    assert(total_weight > 0.f);
    for (int i = 0; i < weights_.size(); ++i) {
      if (i < count) {
        weights_[i] = weights[i] / total_weight;
      } else {
        weights_[i] = 0.f;
      }
    }
  }

  void SetRepeating(bool repeat) {
    for (size_t i = 0; i < motivators_.size(); ++i) {
      motivators_[i].SetRepeating(repeat);
    }
  }

  MotiveTime TimeRemaining() const {
    if (end_time_ == kMotiveTimeEndless) {
      return kMotiveTimeEndless;
    }
    MotiveTime time = 0;
    for (size_t i = 0; i < motivators_.size(); ++i) {
      time = std::max(time, motivators_[i].TimeRemaining());
    }
    return time;
  }

  MotiveTime ChildTimeRemaining(MotiveIndex index) const {
    if (index >= weights_.size()) {
      return 0;
    }
    MotiveTime time = 0;
    const int base_index = BaseBoneIndex(index);
    const int defining_num_bones = NumBones();
    for (BoneIndex i = 0; i < defining_num_bones; ++i) {
      time = std::max(time, motivators_[base_index + i].TimeRemaining());
    }
    return time;
  }

  void UpdateGlobalTransforms() {
    // Only do a weighted average if there's more than one animation.
    if (weights_.size() <= 1) {
      CalculateGlobalTransforms(global_transforms_.data(),
                                &root_motion_transform_);
    } else {
      CalculateBlendedGlobalTransforms(global_transforms_.data(),
                                       &root_motion_transform_);
    }
  }

  const mathfu::AffineTransform* GlobalTransforms() const {
    return global_transforms_.data();
  }

  const mathfu::AffineTransform& RootMotionTransform() const {
    return root_motion_transform_;
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
        const float value = multiplier * motivators_[idx].ChildValue1f(
                                             static_cast<MotiveChildIndex>(i));
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
  void CalculateGlobalTransforms(
      mathfu::AffineTransform* out,
      mathfu::AffineTransform* root_motion_transform) const {
    const BoneIndex* parents = defining_anim_->bone_parents();
    const int num_bones = NumBones();
    for (int i = 0; i < num_bones; ++i) {
      // TODO: Return an AffineTransform from the MatrixMotivator.
      const mathfu::mat4& local_transform = motivators_[i].Value();
      const int parent_idx = parents[i];

      // Root motion bone transforms are stored separately and treated as the
      // identity transform when computing child bone transforms.
      if (i == root_motion_bone_) {
        *root_motion_transform =
            mathfu::mat4::ToAffineTransform(local_transform);
        // The root motion bone shouldn't have a parent index, but if for some
        // reason it does, respect the transform of that bone.
        if (parent_idx == kInvalidBoneIdx) {
          out[i] = mathfu::AffineTransform::Identity();
        } else {
          assert(i > parent_idx);
          out[i] = out[parent_idx];
        }
      } else if (parent_idx == kInvalidBoneIdx) {
        out[i] = mathfu::mat4::ToAffineTransform(local_transform);
      } else {
        assert(i > parent_idx);
        out[i] = mathfu::mat4::ToAffineTransform(
            mathfu::mat4::FromAffineTransform(out[parent_idx]) *
            local_transform);
      }
    }
  }

  void CalculateBlendedGlobalTransforms(
      mathfu::AffineTransform* out,
      mathfu::AffineTransform* root_motion_transform) const {
    const BoneIndex* parents = defining_anim_->bone_parents();
    const int num_bones = NumBones();
    const int num_anims = weights_.size();

    // For each bone...
    for (int i = 0; i < num_bones; ++i) {
      // TODO(b/111070174) use a scratchpad instead of a local gather SQT to
      // make iteration less "jumpy", and just go through all motivators
      // linearly.

      // Gather the position, rotation, and scale.
      mathfu::vec3 bone_position(0, 0, 0);
      mathfu::quat bone_rotation(0, 0, 0, 0);
      mathfu::vec3 bone_scale(0, 0, 0);

      // The quaternions q and -q represent the same orientation (but not the
      // same rotation). Since this matrix is simply an orientation, ensure that
      // all quaternions are in the same 4-dimensional hemisphere, else their
      // weighted average is incorrect. For example, .5q + .5(-q) should be
      // either q or -q, not 0.
      mathfu::quat first_quat;

      // For each animation...
      for (int j = 0; j < num_anims; ++j) {
        const MatrixMotivator4f& motivator = motivators_.at(i + j * num_bones);
        const float weight = weights_[j];
        float rotation_weight = weight;

        // Get the SQT for the bone.
        mathfu::vec3 position;
        mathfu::vec4 rotation_vec;
        mathfu::vec3 scale;
        motivator.Value(&position, &rotation_vec, &scale);
        const mathfu::quat rotation(rotation_vec.w, rotation_vec.xyz());

        // Check if the quaternion needs to be flipped.
        if (j == 0) {
          first_quat = rotation;
        } else if (mathfu::quat::DotProduct(first_quat, rotation) < 0.f) {
          rotation_weight *= -1.f;
        }

        // Gather the components.
        bone_position += position * weight;
        bone_rotation[0] += rotation[0] * rotation_weight;
        bone_rotation[1] += rotation[1] * rotation_weight;
        bone_rotation[2] += rotation[2] * rotation_weight;
        bone_rotation[3] += rotation[3] * rotation_weight;
        bone_scale += scale * weight;
      }

      // Since the weights are normalized to sum to 1 at all times, only the
      // rotation needs to be normalized.
      bone_rotation.Normalize();

      // Multiply this bone's transform by the parent transform (if it exists).
      const mathfu::mat4 local_transform = mathfu::mat4::Transform(
          bone_position, bone_rotation.ToMatrix(), bone_scale);
      const int parent_idx = parents[i];

      // Root motion bone transforms are stored separately and treated as the
      // identity transform when computing child bone transforms.
      if (i == root_motion_bone_) {
        *root_motion_transform =
            mathfu::mat4::ToAffineTransform(local_transform);
        if (parent_idx == kInvalidBoneIdx) {
          out[i] = mathfu::AffineTransform::Identity();
        } else {
          assert(i > parent_idx);
          out[i] = out[parent_idx];
        }
      } else {
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
  }

  int BaseBoneIndex(int anim_index) const { return anim_index * NumBones(); }

  // TODO(b/111070174): decide if array-of-structs is better. It's faster for
  // calculating transforms but significantly uglier for adding/removing
  // animations and makes updating playback rates require a stride-based
  // approach. Alternatively, add a scratchpad of SQTs with the struct-of-arrays
  // approach to optimize calculating global transforms.

  // Motivators for all the current animations stored struct-of-arrays style.
  // For a defining animation with N bones an animations A and B, the
  // motivators are stored in the following order (Bone_X^Y means "bone X's
  // motivator in animation Y"):
  //   Bone_1^A, Bone_2^A, ..., Bone_N^A, Bone_1^B, Bone_2^B, ..., Bone_N^B.
  std::vector<MatrixMotivator4f, mathfu::simd_allocator<MatrixMotivator4f>>
      motivators_;
  std::vector<mathfu::AffineTransform,
              mathfu::simd_allocator<mathfu::AffineTransform>>
      global_transforms_;

  // The list of weights per running animation, normalized to sum to 1.
  std::vector<float> weights_;

  const RigAnim* defining_anim_;
  const RigAnim* current_anim_;

  // The root motion bone and it's most recent transform.
  BoneIndex root_motion_bone_;
  mathfu::AffineTransform root_motion_transform_;

  /// Time that the animation is expected to complete.
  MotiveTime end_time_;
};

}  // namespace motive

#endif  // MOTIVE_RIG_DATA_H_
