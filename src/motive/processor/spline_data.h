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

#ifndef MOTIVE_SPLINE_DATA_H_
#define MOTIVE_SPLINE_DATA_H_

#include "motive/engine.h"
#include "motive/spline_init.h"
#include "motive/math/bulk_spline_evaluator.h"

namespace motive {

struct SplineData {
  SplineData() : local_spline(nullptr) {}

  // If we own the spline, recycle it in the spline pool.
  CompactSpline* local_spline;
};

}  // namespace motive

#endif  // MOTIVE_SPLINE_DATA_H_
