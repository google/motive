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

#ifndef MOTIVE_SPRING_INIT_H_
#define MOTIVE_SPRING_INIT_H_

#include "motive/simple_init_template.h"

namespace motive {

/// @class SpringInit
/// @brief Initialize a MotivatorNf move oscillate over a target.
///
/// Call @ref MotivatorNf::SetTargetWithShape to set the target the
/// curve moves towards.
struct SpringInit : public SimpleInit {
  MOTIVE_INTERFACE();
  SpringInit() : SimpleInit(kType) {}
  explicit SpringInit(const float* start_values,
                      const float* start_derivatives = nullptr)
      : SimpleInit(kType, start_values, start_derivatives) {}
};

/// Use these types to initialize their corresponding MotivatorXfs using vector
/// types instead of float arrays.
typedef SimpleInitTemplate<SpringInit, MathFuVectorConverter, 1>
    SpringInit1f;
typedef SimpleInitTemplate<SpringInit, MathFuVectorConverter, 2>
    SpringInit2f;
typedef SimpleInitTemplate<SpringInit, MathFuVectorConverter, 3>
    SpringInit3f;
typedef SimpleInitTemplate<SpringInit, MathFuVectorConverter, 4>
    SpringInit4f;

}  // namespace motive

#endif  // MOTIVE_SPRING_INIT_H_
