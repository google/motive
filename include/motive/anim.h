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
/// @brief Animation for a MatrixMotivator. Drives a single bone's transform.
class MatrixAnim {
 public:
  struct Spline {
    fpl::CompactSpline spline;
    SmoothInit init;
    fpl::SplinePlayback playback;
  };

  explicit MatrixAnim(int expected_num_ops = 0) : ops_(expected_num_ops) {}

  /// For construction. Allocates storage for spline data, and returns it.
  /// @param num_splines Total number of splines in the animation. Not all ops
  ///                    use a spline (some are const ops).
  Spline* Construct(int num_splines) {
    if (num_splines == 0) return nullptr;

    // Note: It's important that the `splines_` array is not moved, since
    //       `ops_` points into it.
    // TODO: Revisit this layout to eliminate the internal pointers, making it
    //       more robust.
    splines_.resize(num_splines);
    return &splines_[0];
  }

  /// Return the op array. Non-const version is for construction.
  MatrixOpArray& ops() { return ops_; }

  /// Return the op array. Const version is to initialize a MatrixMotivator.
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
/// @brief Animation for a RigMotivator. Drives a fully rigged model.
class RigAnim {
 public:
  /// Initialize the basic data. After calling this function, `InitMatrixAnim()`
  /// should be called once for every bone in the animation.
  void Init(const char* anim_name, BoneIndex num_bones, bool record_names);

  /// For construction. Return the `idx`th bone's animation for initialization.
  /// @param idx The bone whose animation you want to initialize.
  /// @param parent If no parent exists, pass in kInvalidBoneIdx.
  /// @param bone_name For debugging. Recorded if `record_names` was true in
  ///                  `Init()`.
  MatrixAnim& InitMatrixAnim(BoneIndex idx, BoneIndex parent,
                             const char* bone_name);

  /// Return the animation of the `idx`th bone. Each bone animates a matrix.
  const MatrixAnim& Anim(BoneIndex idx) const {
    assert(idx < anims_.size());
    return anims_[idx];
  }

  /// Number of bones. Bones are arranged in an hierarchy. Each bone animates
  /// a matrix. The matrix describes the transform of the bone from its parent.
  BoneIndex NumBones() const { return static_cast<BoneIndex>(anims_.size()); }

  /// For debugging. If `record_names` was specified in `Init()`, the names of
  /// the bones are stored. Very useful when an animation is applied to a mesh
  /// that doesn't match: with the bone names you can determine whether the
  /// mesh or the animation is out of date.
  const char* BoneName(BoneIndex idx) const {
    return idx < bone_names_.size() ? bone_names_[idx].c_str() : "unknown";
  }

  /// Total number of matrix operations across all MatrixAnims in this RigAnim.
  int NumOps() const;

  /// For debugging. The number of lines in the header. You call them separately
  /// in case you want to prefix or append extra columns.
  int NumCsvHeaderLines() const { return 2; }

  /// Output a line of comma-separated-values that has header information for
  /// the CSV data output by RigMotivator::CsvValues().
  std::string CsvHeaderForDebugging(int line) const;

  /// Amount of time required by this animation. Time units are set by the
  /// caller.
  MotiveTime end_time() const { return end_time_; }

  /// For construction. The end time should be set to the maximal end time of
  /// all the `anims_`.
  void set_end_time(MotiveTime t) { end_time_ = t; }

  /// Returns an array of length NumBones() representing the bone heirarchy.
  /// `bone_parents()[i]` is the bone index of the ith bone's parent.
  /// `bone_parents()[i]` < `bone_parents()[j]` for all i < j.
  /// For bones at the root (i.e. no parent) value is kInvalidBoneIdx.
  const BoneIndex* bone_parents() const { return &bone_parents_[0]; }

  /// For debugging. The name of the animation currently being played.
  /// Only valid if `record_names` is true in `Init()`.
  const std::string& anim_name() const { return anim_name_; }

 private:
  std::vector<MatrixAnim> anims_;
  std::vector<BoneIndex> bone_parents_;
  std::vector<std::string> bone_names_;
  MotiveTime end_time_;
  std::string anim_name_;
};

}  // namespace motive

#endif  // MOTIVE_ANIM_H
