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

#ifndef MOTIVE_MATH_SPLINE_UTIL_H_
#define MOTIVE_MATH_SPLINE_UTIL_H_

#include "mathfu/glsl_mappings.h"

namespace fpl {

inline int NormalizeIdx(const int idx, const int max, const bool wraps) {
  return wraps ? (idx < 0 ? idx + max : idx >= max ? idx - max : idx)
               : (idx < 0 ? 0 : idx >= max ? max - 1 : idx);
}

template <int kDimensions>
int FindFartherIdx(
    const mathfu::VectorPacked<float, kDimensions>* const positions,
    const int num_positions, const bool wraps, const int start_idx,
    const int delta_idx, const float min_dist) {
  typedef typename mathfu::Vector<float, kDimensions> Vec;

  const Vec start_position(positions[start_idx]);
  const int end_idx = NormalizeIdx(
      start_idx + delta_idx * (num_positions / 2 - 1), num_positions, wraps);
  for (int i = NormalizeIdx(start_idx + delta_idx, num_positions, wraps);
       i != end_idx; i = NormalizeIdx(i + delta_idx, num_positions, wraps)) {
    const Vec delta = Vec(positions[i]) - start_position;
    const float dist = delta.Length();
    if (dist >= min_dist) return i;
  }
  return end_idx;
}

/// @brief Calculate times and derivatives for a series of n-dimensional
///        `positions` such that the speed is approximately constant
///        and the entire traversal requires `total_time`.
/// @param positions Array of n-dimensional positions, length `num_positions`.
///                  Note that `VectorPacked` is simply n-floats packed
///                  together, so if necessary you can probably cast from your
///                  own data type.
/// @param num_positions Length of the input `positions`, and the output
///                      `times`, and `derivatives`.
/// @param total_time Total time to traverse all the `positions`.
///                   Except in degenerate cases, when this function returns,
///                   `times[num_positions - 1] = total_time`.
/// @param min_reliable_dist When calculating the tangents, we search adjacent
///                          positions until we find one that is farther than
///                          this distance. That is, we ignore `positions`
///                          that are very close together in the tangent
///                          calculation, and use positions that are farther
///                          away. `min_reliable_dist` defines "very close
///                          together".
/// @param times Output array that recieves the time we achieve each position.
///              Length `num_positions`.
/// @param derivatives Output array that recieves the derivative at each
///                    position. Length `num_positions`.
template <int kDimensions>
void CalculateConstSpeedCurveFromPositions(
    const mathfu::VectorPacked<float, kDimensions>* const positions,
    const int num_positions, const float total_time,
    const float min_reliable_dist, float* const times,
    mathfu::VectorPacked<float, kDimensions>* const derivatives) {
  typedef typename mathfu::Vector<float, kDimensions> Vec;

  // Handle degenerate cases where there aren't enough points to do calculate
  // tangents.
  if (num_positions <= 0) return;
  if (num_positions == 1) {
    derivatives[0] = Vec(0.0f);
    times[0] = 0.0f;
    return;
  }

  // Approximate the curve distance with straight lines.
  // This is a poor approximation, but it's good enough for our first pass.
  std::vector<float> dist(num_positions);
  dist[0] = 0.0f;
  for (int i = 1; i < num_positions; ++i) {
    const Vec delta = Vec(positions[i]) - Vec(positions[i - 1]);
    dist[i] = dist[i - 1] + delta.Length();
  }

  // Estimate the times based on the distance.
  const float total_dist = dist[num_positions - 1];
  const float const_speed_inv = total_time / total_dist;
  const float const_speed = total_dist / total_time;
  for (int i = 0; i < num_positions; ++i) {
    times[i] = const_speed_inv * dist[i];
  }

  // If the positions start and end at the same location, then assume the
  // course wraps around. This affects the tangent calculation.
  const Vec start_to_end =
      Vec(positions[0]) - Vec(positions[num_positions - 1]);
  const bool wraps = start_to_end.Length() < min_reliable_dist;

  // Calculate the tangents by searching backwards and forwards for points that
  // are sufficiently far away to make a good calculation.
  for (int i = 0; i < num_positions; ++i) {
    const int prev_idx = FindFartherIdx(positions, num_positions, wraps, i, -1,
                                        min_reliable_dist);
    const int next_idx = FindFartherIdx(positions, num_positions, wraps, i, 1,
                                        min_reliable_dist);
    const Vec delta = Vec(positions[next_idx]) - Vec(positions[prev_idx]);
    derivatives[i] = const_speed * delta.Normalized();
  }

  // TODO: Create curves to get more accurate calculation of distance.
  //       Then adjust the times and scale the derivatives to match.
}

}  // namespace fpl

#endif  // MOTIVE_MATH_SPLINE_UTIL_H_
