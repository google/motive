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

#include "motive/spring_init.h"
#include "motive/math/curve_util.h"
#include "motive/processor/spring_data.h"
#include "motive/simple_processor_template.h"

namespace motive {

// The following functions are called from SimpleProcessorTemplate.
static inline float SimpleVelocity(const SpringData& d, float /*value*/) {
  return d.q.DerivativeWithContext(d.elapsed_time, d.c);
}

static inline float SimpleTargetValue(const SpringData& d, float /*value*/) {
  return d.q.target();
}

static inline float SimpleTargetVelocity(const SpringData& /*d*/,
                                         float /*value*/) {
  return 0.0f;
}

static inline float SimpleDifference(const SpringData& d, float value) {
  return SimpleTargetValue(d, value) - value;
}

static inline MotiveTime SimpleTargetTime(const SpringData& d) {
  return static_cast<MotiveTime>(d.target_time - d.elapsed_time);
}

class SpringMotiveProcessor : public SimpleProcessorTemplate<SpringData> {
 public:
  virtual ~SpringMotiveProcessor() {}

  virtual void AdvanceFrame(MotiveTime delta_time) {
    Defragment();

    // Loop through every motivator, one at a time.
    // At some point we can write an assembly language function to process
    // these in parallel.
    for (size_t i = 0; i < data_.size(); ++i) {
      SpringData& d = data_[i];

      // Advance the time and then update the current value.
      d.elapsed_time += static_cast<float>(delta_time);
      d.q.IncrementContext(d.elapsed_time, &d.c);
      values_[i] = d.q.EvaluateWithContext(d.elapsed_time, d.c);
    }
  }

  virtual MotivatorType Type() const { return SpringInit::kType; }
  virtual int Priority() const { return 1; }

  virtual void SetTargetWithShape(MotiveIndex index, MotiveDimension dimensions,
                                  const float* target_values,
                                  const float* /*target_velocities*/,
                                  const MotiveCurveShape& shape) {
    for (MotiveDimension i = 0; i < dimensions; ++i) {
      int processor_index = index + i;
      SpringData& d = Data(processor_index);

      // Initialize curve to go from current to target.
      d.q = QuadraticSpring(Value(processor_index), Velocity(processor_index),
                            target_values[i], shape.typical_delta_value,
                            shape.typical_total_time, shape.bias);
      d.c = d.q.CalculateContext(0.0f);
      d.elapsed_time = 0.0f;
    }
  }

  virtual MotiveCurveShape MotiveShape(MotiveIndex /*index*/) const {
    // TODO(jsanmiya): We'll be removing MotiveShape in the next change.
    return MotiveCurveShape();
  }
};

MOTIVE_INSTANCE(SpringInit, SpringMotiveProcessor);

}  // namespace motive
