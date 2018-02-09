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

#ifndef MOTIVE_CONST_INIT_H_
#define MOTIVE_CONST_INIT_H_

#include "motive/simple_init_template.h"

namespace motive {

/// @class ConstInit
/// @brief Initialize a MotivatorNf that holds values and velocities that
///        never change.
///
/// All calls to SetTarget functions are ignored.
struct ConstInit : public SimpleInit {
  MOTIVE_INTERFACE();
  ConstInit() : SimpleInit(kType) {}
  explicit ConstInit(const float* start_values,
                     const float* start_derivatives = nullptr)
      : SimpleInit(kType, start_values, start_derivatives) {}
};

/// Use these types to initialize their corresponding MotivatorXfs using vector
/// types instead of float arrays.
typedef SimpleInitTemplate<ConstInit, MathFuVectorConverter, 1>
    ConstInit1f;
typedef SimpleInitTemplate<ConstInit, MathFuVectorConverter, 2>
    ConstInit2f;
typedef SimpleInitTemplate<ConstInit, MathFuVectorConverter, 3>
    ConstInit3f;
typedef SimpleInitTemplate<ConstInit, MathFuVectorConverter, 4>
    ConstInit4f;

}  // namespace motive

#endif  // MOTIVE_CONST_INIT_H_
