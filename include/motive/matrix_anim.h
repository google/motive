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

#ifndef MOTIVE_MATRIX_ANIM_H_
#define MOTIVE_MATRIX_ANIM_H_

#include "motive/matrix_op.h"
#include "motive/spline_init.h"

namespace motive {

/// @class MatrixAnim
/// @brief Animation for a MatrixMotivator. Drives a single bone's transform.
class MatrixAnim {
 public:
  struct Spline {
    Spline() : spline(nullptr) {}
    ~Spline() {
      CompactSpline::Destroy(spline);
      spline = nullptr;
    }
    CompactSpline* spline;
    SplineInit init;
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
    return splines_.data();
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

}  // namespace motive

#endif  // MOTIVE_MATRIX_ANIM_H_
