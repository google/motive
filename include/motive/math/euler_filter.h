// Copyright 2018 Google Inc. All rights reserved.
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

#ifndef MOTIVE_MATH_EULER_FILTER_H_
#define MOTIVE_MATH_EULER_FILTER_H_

#include "mathfu/glsl_mappings.h"

namespace motive {

// Given two Euler angle representations of a rotation, returns a set of Euler
// angles equivalent to |value| such that the individual X, Y, and Z components
// are as close to |prev| as possible. This function should be called once per
// sample when creating rotation animation curves using the zero vector as the
// first |prev| and the previous return value as subsequent |prev|'s.
mathfu::vec3 EulerFilter(const mathfu::vec3& value, const mathfu::vec3& prev);

}  // namespace motive

#endif  // MOTIVE_MATH_EULER_FILTER_H_
