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

struct MotiveNode1f {
  float value;
  float velocity;
  MotiveTime time;
  fpl::ModularDirection direction;

  MotiveNode1f()
      : value(0.0f),
        velocity(0.0f),
        time(0),
        direction(fpl::kDirectionClosest) {}
  MotiveNode1f(float value, float velocity, MotiveTime time,
               fpl::ModularDirection direction = fpl::kDirectionClosest)
      : value(value), velocity(velocity), time(time), direction(direction) {}
};

// Override the current and/or target state for a one-dimensional Motivator.
// It is valid to set a subset of the parameters here. For example, if you
// want to continually adjust the target value of an Motivator every frame,
// you can call Motivator1f::SetTarget() with an MotiveTarget1f that has only
// the target value set.
//
// If the current value and current velocity are not specified, their current
// values in the motivator are used.
//
// An Motivator's target is set in bulk via the SetTarget call. All the state
// is set in one call because SetTarget will generally involve a lot of
// initialization work. We don't want that initialization to happen twice on
// one frame if we set both 'value' and 'velocity', for instance, so we
// aggregate all the target values into this class, and have only one
// SetTarget() function.
//
class MotiveTarget1f {
 public:
  static const int kMaxNodes = 3;

  MotiveTarget1f() : num_nodes_(0) {}

  explicit MotiveTarget1f(const MotiveNode1f& n0) : num_nodes_(1) {
    nodes_[0] = n0;
  }

  MotiveTarget1f(const MotiveNode1f& n0, const MotiveNode1f& n1)
      : num_nodes_(2) {
    assert(0 <= n0.time && n0.time < n1.time);
    nodes_[0] = n0;
    nodes_[1] = n1;
  }

  MotiveTarget1f(const MotiveNode1f& n0, const MotiveNode1f& n1,
                 const MotiveNode1f& n2)
      : num_nodes_(3) {
    assert(0 <= n0.time && n0.time < n1.time && n1.time < n2.time);
    nodes_[0] = n0;
    nodes_[1] = n1;
    nodes_[2] = n2;
  }

  // Empty the target of all nodes.
  void Reset() { num_nodes_ = 0; }

  const MotiveNode1f& Node(int node_index) const {
    assert(0 <= node_index && node_index < num_nodes_);
    return nodes_[node_index];
  }

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

  MotiveTime EndTime() const {
    assert(num_nodes_ > 0);
    return nodes_[num_nodes_ - 1].time;
  }

  int num_nodes() const { return num_nodes_; }

 private:
  int num_nodes_;
  MotiveNode1f nodes_[kMaxNodes];
};

// Set the Motivator's current values. Target values are reset to be the same
// as the new current values.
inline MotiveTarget1f Current1f(float current_value,
                                float current_velocity = 0.0f) {
  return MotiveTarget1f(MotiveNode1f(current_value, current_velocity, 0));
}

// Keep the Motivator's current values, but set the Motivator's target values.
// If Motivator uses modular arithmetic, traverse from the current to the target
// according to 'direction'.
inline MotiveTarget1f Target1f(
    float target_value, float target_velocity, MotiveTime target_time,
    fpl::ModularDirection direction = fpl::kDirectionClosest) {
  assert(target_time > 0);
  return MotiveTarget1f(
      MotiveNode1f(target_value, target_velocity, target_time, direction));
}

// Set both the current and target values for an Motivator.
inline MotiveTarget1f CurrentToTarget1f(
    float current_value, float current_velocity, float target_value,
    float target_velocity, MotiveTime target_time,
    fpl::ModularDirection direction = fpl::kDirectionClosest) {
  return MotiveTarget1f(
      MotiveNode1f(current_value, current_velocity, 0),
      MotiveNode1f(target_value, target_velocity, target_time, direction));
}

// Move from the current value to the target value at a constant speed.
inline MotiveTarget1f CurrentToTargetConstVelocity1f(float current_value,
                                                     float target_value,
                                                     MotiveTime target_time) {
  assert(target_time > 0);
  const float velocity = (target_value - current_value) / target_time;
  return MotiveTarget1f(
      MotiveNode1f(current_value, velocity, 0),
      MotiveNode1f(target_value, velocity, target_time, fpl::kDirectionDirect));
}

// Keep the Motivator's current values, but set two targets for the Motivator.
// After the first target, go on to the next.
inline MotiveTarget1f TargetToTarget1f(float target_value,
                                       float target_velocity,
                                       MotiveTime target_time,
                                       float third_value, float third_velocity,
                                       MotiveTime third_time) {
  return MotiveTarget1f(
      MotiveNode1f(target_value, target_velocity, target_time),
      MotiveNode1f(third_value, third_velocity, third_time));
}

// Set the Motivator's current values, and two targets afterwards.
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
