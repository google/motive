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

#ifndef MOTIVE_SPLINE_INIT_H_
#define MOTIVE_SPLINE_INIT_H_

#include "motive/simple_init_template.h"

namespace motive {

/// @class SplineInit
/// @brief Initialize a MotivatorNf to follow a spline.
///
/// Call MotivatorNf::SetSplines() to follow predefined splines,
/// or call MotivatorNf::SetTargets() to dynamically generate a spline that
/// travels through several key points.
class SplineInit : public MotivatorInit {
 public:
  MOTIVE_INTERFACE();

  SplineInit() : MotivatorInit(kType) {}

  /// @param range If using modular arithmetic, the normalized range.
  ///              If not using modular arithmetic, pass in an invalid range
  ///              such as Range().
  explicit SplineInit(const Range& range) : MotivatorInit(kType), range_(range) {}

  const Range& range() const { return range_; }
  void set_range(const Range& r) { range_ = r; }

 private:
  /// If using modular arithmetic, the normalized range.
  /// For example, for angles, the normalized range can be (pi, +pi].
  /// Whenever a new spline segment is started, the internal logic resets
  /// the value to the normalized range. Note, however, that it is possible
  /// for the value to escape the normalized range. That is,
  /// MotivatorNf::Value() may be outside of `range_`, though it will always
  /// be close enough to normalize efficiently with
  /// Range::NormalizeCloseValue().
  ///
  /// If not using modular arithmetic, set to an invalid range and ignored.
  Range range_;
};

}  // namespace motive

#endif  // MOTIVE_SPLINE_INIT_H_
