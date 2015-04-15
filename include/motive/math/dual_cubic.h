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

#ifndef MOTIVE_MATH_DUAL_CUBIC_H_
#define MOTIVE_MATH_DUAL_CUBIC_H_

#include "motive/math/curve.h"

namespace fpl {

/// Find a point in between init.start_x and init.end_x such that the two
/// cubic functions look smoother than the one cubic function created by init.
///
/// Please see docs/dual_cubic.pdf for the math.
///
/// @param init Start and end values for the cubic that we're splitting into
///             two cubics.
/// @param x Output mid point's x value.
/// @param y Output mid point's y value.
/// @param derivative Output mid point's derivative value.
void CalculateDualCubicMidNode(const CubicInit &init, float *x, float *y,
                               float *derivative);

}  // namespace fpl

#endif  // MOTIVE_MATH_DUAL_CUBIC_H_
