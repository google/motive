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

//! [Motivator Example]

#include "motive/engine.h"
#include "motive/init.h"
#include "motive/math/angle.h"
#include "motive/math/curve.h"

using mathfu::vec2;
using motive::Angle;
using motive::Graph2DPoints;
using motive::kAngleRange;
using motive::kPi;
using motive::Range;

int main() {

  // Since we use the ‘spline’ animation algorithm, we must register it.
  motive::SplineInit::Register();

  // The engine is the central place for animation data.
  motive::MotiveEngine engine;

  // In this example, we animate a one-dimensional floating point value.
  motive::Motivator1f facing_angle;

  // Initialize facing_angle Motivator to animate as a 'Spline' Motivator.
  // Alternatively, we could initialize as an 'Overshoot' Motivator. All
  // Motivator types have the same interface. Internally, they are animated
  // with different algorithms, and they will move differently towards their
  // targets. However, to switch between Motivator types it is a simple matter
  // of initializing with a different kind of MotiveInit struct.
  //
  // Angles wrap around with modular arithmetic. That is, -pi is equivalent to
  // pi. Valid range for angles is -pi..pi, inclusive of +pi and exclusive of
  // -pi.
  const motive::SplineInit init(kAngleRange);
  facing_angle.Initialize(init, &engine);

  // Set initial state of the Motivator, and the target parameters.
  // 'Spline' Motivators animate to a target-value in a target-time. Not all
  // types of Motivators use all target data.
  const Angle start = Angle::FromDegrees(120.0f);
  const float start_angular_velocity = 0.0f;
  const Angle target = Angle::FromDegrees(-120.0f);
  const float target_angular_velocity = 0.0f;
  const motive::MotiveTime target_time = 100;
  const motive::MotiveTime delta_time = 1;
  facing_angle.SetTarget(
      motive::CurrentToTarget1f(start.ToRadians(), start_angular_velocity,
                                target.ToRadians(), target_angular_velocity,
                                target_time));

  std::vector<vec2> points(target_time / delta_time + 1);
  for (motive::MotiveTime t = 0; t <= target_time; t += delta_time) {
    // All Motivators created with 'engine' are animated here.
    engine.AdvanceFrame(delta_time);

    // The current value of the variable being animated is always available.
    const Angle angle_at_t = Angle::FromWithinThreePi(facing_angle.Value());
    points.push_back(
        vec2(static_cast<float>(t), angle_at_t.ToDegrees()));
  }

  printf("\n%s", Graph2DPoints(&points[0], points.size()).c_str());
  return 0;
}

//! [Motivator Example]
