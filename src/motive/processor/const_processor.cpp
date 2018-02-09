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

#include "motive/const_init.h"
#include "motive/math/curve_util.h"
#include "motive/processor/const_data.h"
#include "motive/simple_processor_template.h"

namespace motive {

// The following "Simple" functions are called by the SimpleProcessorTemplate.
// The current `value` is stored external to ConstData, so it is passed in.

// Velocity never changes.
static inline float SimpleVelocity(const ConstData& d, float /*value*/) {
  return d.velocity;
}

// The target value the initial value, since value is constant.
static inline float SimpleTargetValue(const ConstData& /*d*/, float value) {
  return value;
}

// The target velocity is the initial velocity, since velocity is constant.
static inline float SimpleTargetVelocity(const ConstData& d, float /*value*/) {
  return d.velocity;
}

static inline float SimpleDifference(const ConstData& d, float value) {
  return SimpleTargetValue(d, value) - value;
}

// Since we're constant, we're always at our target.
static inline MotiveTime SimpleTargetTime(const ConstData& /*d*/) {
  return static_cast<MotiveTime>(0);
}

class ConstMotiveProcessor : public SimpleProcessorTemplate<ConstData> {
 public:
  virtual ~ConstMotiveProcessor() {}

  virtual void AdvanceFrame(MotiveTime /*delta_time*/) {
    Defragment();
  }

  virtual MotivatorType Type() const { return ConstInit::kType; }
  virtual int Priority() const { return 1; }

  virtual MotiveCurveShape MotiveShape(MotiveIndex /*index*/) const {
    //TODO(jsanmiya): Find a way to store this shape.
    return MotiveCurveShape();
  }
};

MOTIVE_INSTANCE(ConstInit, ConstMotiveProcessor);

}  // namespace motive
