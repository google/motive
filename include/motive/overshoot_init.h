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

#ifndef MOTIVE_OVERSHOOT_INIT_H_
#define MOTIVE_OVERSHOOT_INIT_H_

#include "motive/simple_init_template.h"

namespace motive {

/// @class OvershootInit
/// @brief Initialize a MotivatorNf move towards a target using spring physics.
///
/// Call MotivatorNf::SetTargets() to set the target that we swing towards.
/// The name comes from the movement overshooting the target then coming
/// back, the way a dampened oscillator overshoots its resting point.
class OvershootInit : public MotivatorInit {
 public:
  MOTIVE_INTERFACE();

  OvershootInit()
      : MotivatorInit(kType),
        range_(Range::Full()),
        modular_(false),
        max_velocity_(0.0f),
        accel_per_difference_(0.0f),
        wrong_direction_multiplier_(0.0f),
        max_delta_time_(0) {}

  /// Ensure velocity is within the reasonable limits.
  float ClampVelocity(float velocity) const {
    return mathfu::Clamp(velocity, -max_velocity_, max_velocity_);
  }

  /// Ensure the Motivator's 'value' doesn't increment by more than 'max_delta'.
  /// This is different from ClampVelocity because it is independent of time.
  /// No matter how big the timestep, the delta will not be too great.
  float ClampDelta(float delta) const {
    return mathfu::Clamp(delta, -max_delta_, max_delta_);
  }

  /// Return true if we're close to the target and almost stopped.
  /// The definition of "close to" and "almost stopped" are given by the
  /// "at_target" member.
  bool AtTarget(float dist, float velocity) const {
    return at_target_.Settled(dist, velocity);
  }

  const Range& range() const { return range_; }
  void set_range(const Range& r) { range_ = r; }
  bool modular() const { return modular_; }
  void set_modular(bool modular) { modular_ = modular; }
  float max_velocity() const { return max_velocity_; }
  float max_delta() const { return max_delta_; }
  const Settled1f& at_target() const { return at_target_; }
  Settled1f& at_target() { return at_target_; }
  float accel_per_difference() const { return accel_per_difference_; }
  float wrong_direction_multiplier() const {
    return wrong_direction_multiplier_;
  }
  MotiveTime max_delta_time() const { return max_delta_time_; }

  void set_max_velocity(float max_velocity) { max_velocity_ = max_velocity; }
  void set_max_delta(float max_delta) { max_delta_ = max_delta; }
  void set_at_target(const Settled1f& at_target) { at_target_ = at_target; }
  void set_accel_per_difference(float accel_per_difference) {
    accel_per_difference_ = accel_per_difference;
  }
  void set_wrong_direction_multiplier(float wrong_direction_multiplier) {
    wrong_direction_multiplier_ = wrong_direction_multiplier;
  }
  void set_max_delta_time(MotiveTime max_delta_time) {
    max_delta_time_ = max_delta_time;
  }

 private:
  /// Minimum and maximum values for Motivator::Value().
  /// Clamp (if modular_ is false) or wrap-around (if modular_ is true) when
  /// we reach these boundaries.
  Range range_;

  /// A modular value wraps around from min to max. For example, an angle
  /// is modular, where -pi is equivalent to +pi. Setting this to true ensures
  /// that arithmetic wraps around instead of clamping to min/max.
  bool modular_;

  /// Maximum speed at which the value can change. That is, maximum value for
  /// the Motivator::Velocity(). In units/tick.
  /// For example, if the value is an angle, then this is the max angular
  /// velocity, and the units are radians/tick.
  float max_velocity_;

  /// Maximum that Motivator::Value() can be altered on a single call to
  /// MotiveEngine::AdvanceFrame(), regardless of velocity or delta_time.
  float max_delta_;

  /// Cutoff to determine if the Motivator's current state has settled on the
  /// target. Once it has settled, Value() is set to TargetValue() and
  /// Velocity() is set to zero.
  Settled1f at_target_;

  /// Acceleration is a multiple of abs('state_.position' - 'target_.position').
  /// Bigger differences cause faster acceleration.
  float accel_per_difference_;

  /// When accelerating away from the target, we multiply our acceleration by
  /// this amount. We need counter-acceleration to be stronger so that the
  /// amplitude eventually dies down; otherwise, we'd just have a pendulum.
  float wrong_direction_multiplier_;

  /// The algorithm is iterative. When the iteration step gets too big, the
  /// behavior becomes erratic. This value clamps the iteration step.
  MotiveTime max_delta_time_;
};

}  // namespace motive

#endif  // MOTIVE_OVERSHOOT_INIT_H_
