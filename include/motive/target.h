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

#ifndef MOTIVE_TARGET_H
#define MOTIVE_TARGET_H

#include "motive/math/range.h"

namespace motive {

/// @class MotiveNode1f
/// @brief A waypoint in MotiveTarget1f.
/// Describes one key point through which a value is animated.
struct MotiveNode1f {
  float value;                      /// Desired value to animate to at `time`.
  float velocity;                   /// Speed when at `time`.
  MotiveTime time;                  /// Time to achieve this key point.
  fpl::ModularDirection direction;  /// When using modular arithmetic, which of
                                    /// two directions to go.
  MotiveNode1f()
      : value(0.0f),
        velocity(0.0f),
        time(0),
        direction(fpl::kDirectionClosest) {}
  MotiveNode1f(float value, float velocity, MotiveTime time,
               fpl::ModularDirection direction = fpl::kDirectionClosest)
      : value(value), velocity(velocity), time(time), direction(direction) {}
};

/// @class MotiveTarget1f
/// @brief Set the current and/or target state for a one-dimensional Motivator.
///
/// A series of waypoints through which we animate. If the first waypoint has
/// time = 0, the current value and velocity jumps to that waypoint's value and
/// velocity.
///
/// MotiveTarget1fs are most easily created with the utility functions below,
/// for example Current1f, Target1f, CurrentToTarget1f.
///
/// If the current value and velocity are not specified (i.e. if the first
/// waypoint has time > 0), then the current value and velocity in
/// the Motivator are maintained.
///
/// If the target is not specified (i.e. only one waypoint which has time = 0),
/// then the current value is set as specified, and the velocity is set to 0.
///
class MotiveTarget1f {
 public:
  static const int kMaxNodes = 3;

  MotiveTarget1f() : num_nodes_(0) {}

  /// Create with only one waypoint.
  /// If n0.time = 0, set the current value and velocity.
  /// If n0.time > 0, maintain the current value and velocity and animate to
  /// n0's value and velocity in n0.time.
  explicit MotiveTarget1f(const MotiveNode1f& n0) : num_nodes_(1) {
    nodes_[0] = n0;
  }

  /// Create with two waypoints.
  /// Can be current to target, if n0.time = 0.
  /// Or can maintain the current and to through two targets: first n0, then n1.
  /// Precondition: n0.time < n1.time.
  MotiveTarget1f(const MotiveNode1f& n0, const MotiveNode1f& n1)
      : num_nodes_(2) {
    assert(0 <= n0.time && n0.time < n1.time);
    nodes_[0] = n0;
    nodes_[1] = n1;
  }

  /// Create with three waypoints.
  /// 0 <= n0.time < n1.time < n2.time
  MotiveTarget1f(const MotiveNode1f& n0, const MotiveNode1f& n1,
                 const MotiveNode1f& n2)
      : num_nodes_(3) {
    assert(0 <= n0.time && n0.time < n1.time && n1.time < n2.time);
    nodes_[0] = n0;
    nodes_[1] = n1;
    nodes_[2] = n2;
  }

  /// Empty the target of all waypoints.
  void Reset() { num_nodes_ = 0; }

  /// Return nth waypoint.
  /// @param node_index nth waypoint. 0 <= node_index < num_nodes()
  const MotiveNode1f& Node(int node_index) const {
    assert(0 <= node_index && node_index < num_nodes_);
    return nodes_[node_index];
  }

  /// Return smallest range that covers the values of all waypoints.
  /// @param start_value An extra value to include in the min/max calculation.
  ///                    Most often is the current value of the Motivator1f.
  fpl::Range ValueRange(float start_value) const {
    assert(num_nodes_ > 0);
    float min = start_value;
    float max = start_value;
    for (int i = 0; i < num_nodes_; ++i) {
      min = std::min(nodes_[i].value, min);
      max = std::max(nodes_[i].value, max);
    }
    return fpl::Range(min, max);
  }

  /// Return time of the last waypoint.
  MotiveTime EndTime() const {
    assert(num_nodes_ > 0);
    return nodes_[num_nodes_ - 1].time;
  }

  int num_nodes() const { return num_nodes_; }

 private:
  /// Length of nodes_.
  int num_nodes_;

  /// Constant-size array, to avoid dynamic memory allocation.
  /// This class is often used as a parameter and allocated on the stack.
  MotiveNode1f nodes_[kMaxNodes];
};

/// Set the Motivator's current values. Target values are reset to be the same
/// as the new current values.
inline MotiveTarget1f Current1f(float current_value,
                                float current_velocity = 0.0f) {
  return MotiveTarget1f(MotiveNode1f(current_value, current_velocity, 0));
}

/// Keep the Motivator's current values, but set the Motivator's target values.
/// If Motivator uses modular arithmetic, traverse from the current to the
/// target according to 'direction'.
inline MotiveTarget1f Target1f(
    float target_value, float target_velocity, MotiveTime target_time,
    fpl::ModularDirection direction = fpl::kDirectionClosest) {
  assert(target_time > 0);
  return MotiveTarget1f(
      MotiveNode1f(target_value, target_velocity, target_time, direction));
}

/// Set both the current and target values for an Motivator.
inline MotiveTarget1f CurrentToTarget1f(
    float current_value, float current_velocity, float target_value,
    float target_velocity, MotiveTime target_time,
    fpl::ModularDirection direction = fpl::kDirectionClosest) {
  return MotiveTarget1f(
      MotiveNode1f(current_value, current_velocity, 0),
      MotiveNode1f(target_value, target_velocity, target_time, direction));
}

/// Move from the current value to the target value at a constant speed.
inline MotiveTarget1f CurrentToTargetConstVelocity1f(float current_value,
                                                     float target_value,
                                                     MotiveTime target_time) {
  assert(target_time > 0);
  const float velocity = (target_value - current_value) / target_time;
  return MotiveTarget1f(
      MotiveNode1f(current_value, velocity, 0),
      MotiveNode1f(target_value, velocity, target_time, fpl::kDirectionDirect));
}

/// Keep the Motivator's current values, but set two targets for the Motivator.
/// After the first target, go on to the next.
inline MotiveTarget1f TargetToTarget1f(float target_value,
                                       float target_velocity,
                                       MotiveTime target_time,
                                       float third_value, float third_velocity,
                                       MotiveTime third_time) {
  return MotiveTarget1f(
      MotiveNode1f(target_value, target_velocity, target_time),
      MotiveNode1f(third_value, third_velocity, third_time));
}

/// Set the Motivator's current values, and two targets afterwards.
inline MotiveTarget1f CurrentToTargetToTarget1f(
    float current_value, float current_velocity, float target_value,
    float target_velocity, MotiveTime target_time, float third_value,
    float third_velocity, MotiveTime third_time) {
  return MotiveTarget1f(
      MotiveNode1f(current_value, current_velocity, 0),
      MotiveNode1f(target_value, target_velocity, target_time),
      MotiveNode1f(third_value, third_velocity, third_time));
}

}  // namespace motive

#endif  // MOTIVE_TARGET_H
