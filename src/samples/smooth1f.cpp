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

using fpl::Angle;
using fpl::Range;
using fpl::kPi;
using fpl::Graph2DPoints;
using mathfu::vec2;

int main() {

  // Since we will be using the ‘smooth’ animation algorithm, we must register it
  // with the engine.
  motive::SmoothInit::Register();

  // The engine is the central place where animation data is stored and processed.
  motive::MotiveEngine engine;

  // In this example, we animate a one-dimensional floating point value.
  // It's also possible to animate a mathfu::vec2 with motive::Motivator2f, and
  // similarly for higher dimensional vectors. We can even animate a mathfu::mat4
  // (a 4x4 matrix) with motive::MotivatorMatrix4f.
  //
  // If you have your own math library, you can also animate those instead of
  // mathfu types. See [Using Your Own Math Types][].
  motive::Motivator1f facing_angle;

  // Initialize facing_angle Motivator to animate as a 'Smooth' Motivator.
  // Alternatively, we could initialize as an 'Overshoot' Motivator. All
  // Motivator types have the same interface. Internally, they are animated
  // with different algorithms, and they will move differently towards their
  // targets. However, to switch between Motivator types it is a simple matter
  // of initializing with a different kind of MotiveInit struct.
  //
  // Angles wrap around with modular arithmetic. That is, -pi is equivalent to
  // pi. Valid range for angles is -pi..pi, inclusive of +pi and exclusive of
  // -pi.
  const motive::SmoothInit init(Range(-kPi, kPi), true);
  facing_angle.Initialize(init, &engine);

  // Set initial state of the Motivator, and the target parameters.
  // 'Smooth' Motivators animate to a target-value in a target-time. Not all
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
    // That is, all Motivators created with 'engine' are animated here.
    engine.AdvanceFrame(delta_time);

    // The current value of the variable being animated is always available.
    // Additionally, we can also access facing_angle.Velocity() for the angular
    // velocity.
    const Angle facing_angle_at_time_t(facing_angle.Value());
    points.push_back(
        vec2(static_cast<float>(t), facing_angle_at_time_t.ToDegrees()));
  }

  printf("\n%s", Graph2DPoints(&points[0], points.size()).c_str());
  return 0;
}

//! [Motivator Example]
