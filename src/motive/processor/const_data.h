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

#ifndef MOTIVE_CONST_DATA_H_
#define MOTIVE_CONST_DATA_H_

#include "motive/const_init.h"
#include "motive/math/curve_util.h"
#include "motive/simple_processor_template.h"

namespace motive {

struct ConstData {
  ConstData() : velocity(0.0f) {}
  explicit ConstData(const SimpleInit& init, MotiveIndex index)
      : velocity(init.start_derivatives[index]) {}

  float velocity;
};

}  // namespace motive

#endif  // MOTIVE_CONST_DATA_H_
