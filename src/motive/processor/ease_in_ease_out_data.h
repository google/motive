// Copyright 2016 Google Inc. All rights reserved.
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

#ifndef MOTIVE_EASE_IN_EASE_OUT_DATA_H_
#define MOTIVE_EASE_IN_EASE_OUT_DATA_H_

#include "motive/ease_in_ease_out_init.h"
#include "motive/math/curve_util.h"
#include "motive/simple_processor_template.h"

namespace motive {

struct EaseInEaseOutData {
  EaseInEaseOutData()
      : q_start_time(0.0f), target_time(0.0f), elapsed_time(0.0f) {}

  // Create a straight line with the start value and derivative for q.
  EaseInEaseOutData(const SimpleInit& init, MotiveIndex current_dimension)
      : q(QuadraticEaseInEaseOut(
            QuadraticCurve(QuadraticInitWithOrigin(
                init.start_values[current_dimension],
                init.start_derivatives[current_dimension], 0.0f)),
            0.0f)),
        q_start_time(0.0f),
        target_time(0.0f),
        elapsed_time(0.0f) {}

  // Currently active curve.
  QuadraticEaseInEaseOut q;

  // Shape that holds the bias, typical y-distance that should be traveled, and
  // typical time it takes to travel typical delta value.
  MotiveCurveShape shape;

  // Time at which we started on current curve.
  float q_start_time;

  // Target time of the first curve generated.
  float target_time;

  // Time since the last call to SetTarget().
  float elapsed_time;
};

}  // namespace motive

#endif  // MOTIVE_EASE_IN_EASE_OUT_DATA_H_
