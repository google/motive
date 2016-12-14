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

#include "motive/engine.h"
#include "motive/init.h"
#include "motive/math/curve_util.h"

namespace motive {

static const float kDerivativeEpsilon = 0.000001f;

struct EaseInEaseOutData {
  EaseInEaseOutData()
      : q_start_time(0.0f), target_time(0.0f), elapsed_time(0.0f) {}

  // Create a straight line with the start value and derivative for q.
  EaseInEaseOutData(const EaseInEaseOutInit& init,
                    MotiveIndex current_dimension)
      : q(QuadraticEaseInEaseOut(
            QuadraticCurve(QuadraticInitWithOrigin(
                init.start_values[current_dimension],
                init.start_derivatives[current_dimension], 0.0f)),
            0.0f)),
        q_start_time(0.0f),
        target_time(0.0f),
        elapsed_time(0.0f) {}

  // Currently active curve.
  QuadraticEaseInEaseOut q;

  // Shape that holds the bias, typical y-distance that should be traveled, and
  // typical time it takes to travel typical delta value.
  MotiveCurveShape shape;

  // Time at which we started on current curve.
  float q_start_time;

  // Target time of the first curve generated.
  float target_time;

  // Time since the last call to SetTarget().
  float elapsed_time;
};

class EaseInEaseOutMotiveProcessor : public MotiveProcessorNf {
 public:
  virtual ~EaseInEaseOutMotiveProcessor() {}

  virtual void AdvanceFrame(MotiveTime delta_time) {
    Defragment();

    // Loop through every motivator one at a time.
    for (size_t i = 0; i < data_.size(); ++i) {
      EaseInEaseOutData& d = data_[i];

      // Advance the time and then update the current value.
      d.elapsed_time += static_cast<float>(delta_time);

      float q_time = d.elapsed_time - d.q_start_time;

      // If we go past the end value, with a non-zero derivative and there's
      // no instruction to go to another target, make it so that our curve is
      // adjusted to hit target value with a zero derivative.
      if (q_time >= d.q.total_x()) {
        float target_value = d.q.Evaluate(d.q.total_x());
        float target_velocity = d.q.Derivative(d.q.total_x());
        d.q_start_time += d.q.total_x();
        q_time = d.elapsed_time - d.q_start_time;
        const bool ends_with_nonzero_derivative =
            std::fabs(target_velocity) > kDerivativeEpsilon;
        if (ends_with_nonzero_derivative) {
          // Create curve to hit target value with zero derivative.
          float start_second_derivative_abs = 0.0f;
          float end_second_derivative_abs = 0.0f;
          CalculateSecondDerivativesFromTypicalCurve(
              d.shape.typical_delta_value, d.shape.typical_total_time,
              d.shape.bias, &start_second_derivative_abs,
              &end_second_derivative_abs);
          d.q = CalculateQuadraticEaseInEaseOut(
              target_value, target_velocity, start_second_derivative_abs,
              target_value, 0.0f, end_second_derivative_abs,
              d.shape.typical_delta_value, d.shape.typical_total_time);
        } else {
          // Curve is a flat line at target_value.
          d.q = QuadraticEaseInEaseOut(QuadraticCurve(0.0f, 0.0f, target_value),
                                       std::numeric_limits<float>::infinity());
        }
      }
      values_[i] = d.q.Evaluate(q_time);
    }
  }

  virtual MotivatorType Type() const { return EaseInEaseOutInit::kType; }
  virtual int Priority() const { return 1; }

  // Accessors to allow the user to get and set simluation values.
  virtual const float* Values(MotiveIndex index) const {
    return &values_[index];
  }

  virtual void Velocities(MotiveIndex index, MotiveDimension dimensions,
                          float* out) const {
    for (MotiveDimension i = 0; i < dimensions; ++i) {
      const EaseInEaseOutData& d = Data(index + i);
      out[i] = d.q.Derivative(d.elapsed_time);
    }
  }

  virtual void TargetValues(MotiveIndex index, MotiveDimension dimensions,
                            float* out) const {
    for (MotiveDimension i = 0; i < dimensions; ++i) {
      const EaseInEaseOutData& d = Data(index + i);
      out[i] = d.q.Evaluate(d.q.total_x());
    }
  }

  virtual void TargetVelocities(MotiveIndex index, MotiveDimension dimensions,
                                float* out) const {
    for (MotiveDimension i = 0; i < dimensions; ++i) {
      const EaseInEaseOutData& d = Data(index + i);
      out[i] = d.q.Derivative(d.q.total_x());
    }
  }

  virtual void Differences(MotiveIndex index, MotiveDimension dimensions,
                           float* out) const {
    for (MotiveDimension i = 0; i < dimensions; ++i) {
      const EaseInEaseOutData& d = Data(index + i);
      out[i] = d.q.Evaluate(d.q.total_x()) - values_[i];
    }
  }

  virtual MotiveTime TargetTime(MotiveIndex index,
                                MotiveDimension dimensions) const {
    MotiveTime greatest = std::numeric_limits<MotiveTime>::min();
    for (MotiveDimension i = 0; i < dimensions; ++i) {
      const EaseInEaseOutData& d = Data(index + i);
      greatest =
          std::max(greatest,
                   static_cast<MotiveTime>(d.target_time - d.elapsed_time));
    }
    return greatest;
  }

  virtual void SetTargetWithShape(MotiveIndex index, MotiveDimension dimensions,
                                  const float* target_values,
                                  const float* target_velocities,
                                  const MotiveCurveShape& shape) {
    for (MotiveDimension i = 0; i < dimensions; ++i) {
      int processor_index = index + i;
      EaseInEaseOutData& d = Data(processor_index);

      // Initialize curve to go from current to target.
      float start_second_derivative_abs = 0.0f;
      float end_second_derivative_abs = 0.0f;
      CalculateSecondDerivativesFromTypicalCurve(
          shape.typical_delta_value, shape.typical_total_time, shape.bias,
          &start_second_derivative_abs, &end_second_derivative_abs);
      d.q = CalculateQuadraticEaseInEaseOut(
          Value(processor_index),  Velocity(processor_index),
          start_second_derivative_abs, target_values[i], target_velocities[i],
          end_second_derivative_abs, shape.typical_delta_value,
          shape.typical_total_time);
      d.target_time = d.q.total_x();
      d.q_start_time = 0.0f;
      d.elapsed_time = 0.0f;
      d.shape = shape;
    }
  }

  virtual MotiveCurveShape MotiveShape(MotiveIndex index) const {
    const EaseInEaseOutData& d = Data(index);
    return d.shape;
  }

 protected:
  virtual void InitializeIndices(const MotivatorInit& init, MotiveIndex index,
                                 MotiveDimension dimensions,
                                 MotiveEngine* /*engine*/) {
    const EaseInEaseOutInit& ease_in_ease_out_init =
        static_cast<const EaseInEaseOutInit&>(init);
    for (MotiveDimension i = 0; i < dimensions; ++i) {
      const MotiveIndex processor_index = i + index;
      Data(processor_index) = EaseInEaseOutData(ease_in_ease_out_init, i);
      values_[processor_index] = ease_in_ease_out_init.start_values[i];
    }
  }

  virtual void RemoveIndices(MotiveIndex index, MotiveDimension dimensions) {
    for (MotiveIndex i = index; i < index + dimensions; ++i) {
      Data(i) = EaseInEaseOutData();
      values_[i] = 0.0f;
    }
  }

  virtual void MoveIndices(MotiveIndex old_index, MotiveIndex new_index,
                           MotiveDimension dimensions) {
    MotiveIndex old_i = old_index;
    MotiveIndex new_i = new_index;
    for (MotiveDimension i = 0; i < dimensions; ++i, ++new_i, ++old_i) {
      data_[new_i] = data_[old_i];
      values_[new_i] = values_[old_i];
    }
  }

  virtual void SetNumIndices(MotiveIndex num_indices) {
    data_.resize(num_indices);
    values_.resize(num_indices);
  }

  const EaseInEaseOutData& Data(MotiveIndex index) const {
    assert(ValidIndex(index));
    return data_[index];
  }

  EaseInEaseOutData& Data(MotiveIndex index) {
    assert(ValidIndex(index));
    return data_[index];
  }

  std::vector<EaseInEaseOutData> data_;
  std::vector<float> values_;
};

MOTIVE_INSTANCE(EaseInEaseOutInit, EaseInEaseOutMotiveProcessor);

}  // namespace motive
