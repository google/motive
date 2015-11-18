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


#include "motive/engine.h"
#include "motive/init.h"
#include "motive/math/angle.h"
#include "motive/math/curve.h"

using mathfu::kZeros2f;
using mathfu::vec2;
using motive::Angle;
using motive::Graph2DPoints;
using motive::kPi;
using motive::Motivator2f;
using motive::MotiveEngine;
using motive::MotiveTime;
using motive::Range;
using motive::SmoothInit;
using motive::Tar2f;


static const vec2 kDuckStartPosition(1.0f, 1.0f);
static const MotiveTime kTimeToTouchPosition = 1000;
static const MotiveTime kDeltaTime = 32;


// Placeholder implementation. This code is not included in the sample code.
static bool ScreenTouch(vec2* touch_position) {
  *touch_position = kZeros2f;
  return true;
}

// Placeholder implementation. This code is not included in the sample code.
static void DrawDuck(const vec2& /*position*/, const vec2& /*velocity*/) {}

int main() {
  std::vector<vec2> points;
  points.reserve(100);

  // Since we will be using the ‘smooth’ animation algorithm, we must register
  // it with the engine.
  SmoothInit::Register();

  // The engine is the central place where animation data is stored and
  // processed.
  MotiveEngine motive_engine;

  //! [Duck Example]
  // Initialize the Motivator to use the "smooth" animation algorithm.
  Motivator2f duck_position;
  duck_position.InitializeWithTarget(SmoothInit(), &motive_engine,
                                     Tar2f::Current(kDuckStartPosition));

  // Smoothly transition the duck to wherever a touch occurs.
  while (true) {
    vec2 touch_position;
    if (ScreenTouch(&touch_position)) {
      // Set the duck's target position, velocity, and time.
      duck_position.SetTarget(Tar2f::Target(touch_position, kZeros2f,
                                            kTimeToTouchPosition));
    }

    // Once per frame, motive_engine.AdvanceFrame() is called to animate *all*
    // Motivators at once. In this case, we only have one Motivator,
    // but in many applications there will be thousands.
    motive_engine.AdvanceFrame(kDeltaTime);

    // Draw the duck at its current position.
    // Use the velocity to draw motion lines behind the duck, too.
    DrawDuck(duck_position.Value(), duck_position.Velocity());
  }
  //! [Duck Example]
  return 0;
}

