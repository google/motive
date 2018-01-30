// Copyright 2017 Google Inc. All rights reserved.
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

#ifndef MOTIVE_SPRING_DATA_H_
#define MOTIVE_SPRING_DATA_H_

#include "motive/spring_init.h"
#include "motive/math/curve_util.h"
#include "motive/simple_processor_template.h"

namespace motive {

// TODO(jsanmiya): We probably want to make the number of iterations
//                 configurable in MotiveCurveShape.
static const float kNumSpringIterations = 4.0f;

struct SpringData {
  SpringData() : elapsed_time(0.0f), target_time(0.0f) {}

  SpringData(const SimpleInit& init, MotiveIndex current_dimension)
      : q(init.start_values[current_dimension]),
        elapsed_time(0.0f),
        target_time(q.IterationX(kNumSpringIterations)) {}

  // Currently active curve.
  QuadraticSpring q;

  // Evaluation helper that holds portion of QuadraticSpring around
  // `elapsed_time`.
  QuadraticSpring::Context c;

  // Time since the start of q.
  float elapsed_time;

  // Time after kNumSpringIterations.
  float target_time;
};

}  // namespace motive

#endif  // MOTIVE_SPRING_DATA_H_
