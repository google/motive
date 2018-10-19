// Copyright 2018 Google Inc. All rights reserved.
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

#include "motive/math/euler_filter.h"

#include <math.h>
#include "mathfu/constants.h"

namespace {

static constexpr float kTwoPi = 2.f * mathfu::kPi;

// Returns the sum of the absolute differences between |a| and |b|.
float EulerDistance(const mathfu::vec3& a, const mathfu::vec3& b) {
  return fabs(a.x - b.x) + fabs(a.y - b.y) + fabs(a.z - b.z);
}

// Returns an equivalent angle to |value| normalized to be within +/- pi of
// |target|.
float EulerNormalize(float target, float value) {
  while (fabs(target - value) > mathfu::kPi) {
    if (target < value) {
      value -= kTwoPi;
    } else {
      value += kTwoPi;
    }
  }
  return value;
}

}  // namespace

namespace motive {

mathfu::vec3 EulerFilter(const mathfu::vec3& value, const mathfu::vec3& prev) {
  // Filter the original |value| to be within pi of |prev|.
  const mathfu::vec3 filtered_value(
      EulerNormalize(prev.x, value.x),
      EulerNormalize(prev.y, value.y),
      EulerNormalize(prev.z, value.z));

  // Compute the "Euler flipped" equivalent of |filtered_values|.
  const mathfu::vec3 euler_flipped(
      EulerNormalize(prev.x, filtered_value.x + mathfu::kPi),
      EulerNormalize(prev.y, filtered_value.y * -1.f + mathfu::kPi),
      EulerNormalize(prev.z, filtered_value.z + mathfu::kPi));

  // Return whichever is "closer" to |prev|.
  if (EulerDistance(filtered_value, prev) >
      EulerDistance(euler_flipped, prev)) {
    return euler_flipped;
  } else {
    return filtered_value;
  }
}

}  // namespace motive
