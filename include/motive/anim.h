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

  // For construction. Allocate storage for spline data.
  Spline* Construct(int num_splines) {
    splines_.resize(num_splines);
    return &splines_[0];
  }

  // Non-const version is for construction.
  MatrixInit& init() { return init_; }

  // Const version is to initialize a MatrixMotivator.
  const MatrixInit& init() const { return init_; }

 private:
  /// Initialization structure for a MatrixMotivator.
  /// When initialized with this struct, the MatrixMotivator will play back
  /// the animation described in this class.
  MatrixInit init_;

  /// Hold spline animation data that is referenced by `init_`.
  std::vector<Spline> splines_;
};

}  // namespace motive

#endif  // MOTIVE_ANIM_H
