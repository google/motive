// Copyright 2015 Google Inc. All rights reserved.
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

#ifndef MOTIVE_ANIM_H
#define MOTIVE_ANIM_H

#include <vector>
#include "motive/init.h"
#include "motive/math/compact_spline.h"

namespace motive {

/// @class MatrixAnim
/// @brief Animation for a MatrixMotivator.
class MatrixAnim {
 public:
  struct Spline {
    fpl::CompactSpline spline;
    SmoothInit init;
    fpl::SplinePlayback playback;
  };

  explicit MatrixAnim(int expected_num_ops = 0) : ops_(expected_num_ops) {}

  // For construction. Allocate storage for spline data.
  Spline* Construct(int num_splines) {
    splines_.resize(num_splines);
    return &splines_[0];
  }

  // Non-const version is for construction.
  MatrixOpArray& ops() { return ops_; }

  // Const version is to initialize a MatrixMotivator.
  const MatrixOpArray& ops() const { return ops_; }

 private:
  /// Initialization structure for a MatrixMotivator.
  /// When initialized with this struct, the MatrixMotivator will play back
  /// the animation described in this class.
  MatrixOpArray ops_;

  /// Hold spline animation data that is referenced by `init_`.
  std::vector<Spline> splines_;
};

/// @class RigAnim
/// @brief Animation for a rigged model
class RigAnim {
 public:
  void Init(BoneIndex num_bones, bool record_names) {
    assert(num_bones <= kMaxNumBones);
    anims_.resize(num_bones);
    bone_parents_.resize(num_bones);
    if (record_names) {
      bone_names_.resize(num_bones);
    }
  }

  // @param parent If no parent exists, pass in kInvalidBoneIdx.
  MatrixAnim& InitMatrixAnim(BoneIndex idx, BoneIndex parent,
                             const char* bone_name) {
    assert(idx < static_cast<int>(anims_.size()));
    assert(parent < idx || parent == kInvalidBoneIdx);
    bone_parents_[idx] = static_cast<uint8_t>(parent);
    if (bone_names_.size() > 0) {
      bone_names_[idx] = bone_name;
    }
    return anims_[idx];
  }

  MotiveTime end_time() const { return end_time_; }
  void set_end_time(MotiveTime t) { end_time_ = t; }

  const BoneIndex* bone_parents() const { return &bone_parents_[0]; }

  BoneIndex NumBones() const { return static_cast<BoneIndex>(anims_.size()); }
  const char* BoneName(BoneIndex idx) const {
    return idx < bone_names_.size() ? bone_names_[idx].c_str() : "unknown";
  }

  const MatrixAnim& Anim(BoneIndex idx) const {
    assert(idx < anims_.size());
    return anims_[idx];
  }

 private:
  std::vector<MatrixAnim> anims_;
  std::vector<BoneIndex> bone_parents_;
  std::vector<std::string> bone_names_;
  MotiveTime end_time_;
};

}  // namespace motive

#endif  // MOTIVE_ANIM_H
