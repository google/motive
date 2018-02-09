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

#ifndef MOTIVE_EASE_IN_EASE_OUT_INIT_H_
#define MOTIVE_EASE_IN_EASE_OUT_INIT_H_

#include "motive/simple_init_template.h"

namespace motive {

/// @class EaseInEaseOutInit
/// @brief Initialize a MotivatorNf move towards target using ease-in
///        ease-out math.
///
/// Call @ref MotivatorNf::SetTargetWithShape to set the target the
/// curve moves towards.
struct EaseInEaseOutInit : public SimpleInit {
  MOTIVE_INTERFACE();
  EaseInEaseOutInit() : SimpleInit(kType) {}
  explicit EaseInEaseOutInit(const float* start_values,
                             const float* start_derivatives = nullptr)
      : SimpleInit(kType, start_values, start_derivatives) {}
};

/// Use these types to initialize their corresponding MotivatorXfs using vector
/// types instead of float arrays.
typedef SimpleInitTemplate<EaseInEaseOutInit, MathFuVectorConverter, 1>
    EaseInEaseOutInit1f;
typedef SimpleInitTemplate<EaseInEaseOutInit, MathFuVectorConverter, 2>
    EaseInEaseOutInit2f;
typedef SimpleInitTemplate<EaseInEaseOutInit, MathFuVectorConverter, 3>
    EaseInEaseOutInit3f;
typedef SimpleInitTemplate<EaseInEaseOutInit, MathFuVectorConverter, 4>
    EaseInEaseOutInit4f;

}  // namespace motive

#endif  // MOTIVE_EASE_IN_EASE_OUT_INIT_H_
