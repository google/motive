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
      : start_second_derivative_abs(0.0f),
        end_second_derivative_abs(0.0f),
        typical_delta_value(0.0f),
        typical_total_time(0.0f),
        q_start_time(0.0f),
        target_time(0.0f),
        elapsed_time(0.0f) {}

  explicit EaseInEaseOutData(const EaseInEaseOutInit& init)
      : typical_delta_value(init.typical_delta_value()),
        typical_total_time(init.typical_total_time()),
        elapsed_time(0.0f) {
    assert(init.typical_delta_value() > 0.0f &&
           init.typical_total_time() > 0.0f);
    CalculateSecondDerivativesFromTypicalCurve(
        init.typical_delta_value(), init.typical_total_time(), init.bias(),
        &start_second_derivative_abs, &end_second_derivative_abs);
  }

  // Currently active curve.
  QuadraticEaseInEaseOut q;

  // Calculated start and end second derivatives
  // for the desired values.
  float start_second_derivative_abs;
  float end_second_derivative_abs;

  // Typical y-distance that should be traveled.
  float typical_delta_value;

  // Typical time it takes to travel typical delta value.
  float typical_total_time;

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

      // If we go past the end value,
      // with a non-zero derivative and there's
      // no instruction to go to another target,
      // make it so that our curve is adjusted
      // to hit target value with a zero derivative.
      if (q_time >= d.q.total_x()) {
        float target_value = d.q.Evaluate(d.q.total_x());
        float target_velocity = d.q.Derivative(d.q.total_x());
        d.q_start_time += d.q.total_x();
        q_time = d.elapsed_time - d.q_start_time;
        const bool ends_with_nonzero_derivative =
            std::fabs(target_velocity) > kDerivativeEpsilon;
        if (ends_with_nonzero_derivative) {
          // Create curve to hit target value with zero derivative.
          d.q = CalculateQuadraticEaseInEaseOut(
              target_value, target_velocity, d.start_second_derivative_abs,
              target_value, 0.0f, d.end_second_derivative_abs,
              d.typical_delta_value, d.typical_total_time);
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

  virtual MotiveTime TargetTime(MotiveIndex index) const {
    const EaseInEaseOutData& d = Data(index);
    return static_cast<MotiveTime>(d.target_time - d.elapsed_time);
  }

  virtual void SetTargets(MotiveIndex index, MotiveDimension dimensions,
                          const MotiveTarget1f* ts) {
    const MotiveTarget1f* t = ts;
    float current_value = 0.0f;
    float current_velocity = 0.0f;
    float target_value = 0.0f;
    float target_velocity = 0.0f;
    for (MotiveIndex i = index; i < index + dimensions; ++i, ++t) {
      EaseInEaseOutData& d = Data(i);

      // A 'time' of 0 means that we're setting the current values.
      const MotiveNode1f& current = t->Node(0);
      if (current.time == 0) {
        current_value = current.value;
        current_velocity = current.velocity;
      } else {
        current_value = d.q.Evaluate(d.elapsed_time);
        current_velocity = d.q.Derivative(d.elapsed_time);
      }

      // A 'time' > 0 means that we're setting the target values.
      // We can also use the second node to set target values, if it exists.
      const MotiveNode1f* target =
          current.time == 0 ? (t->num_nodes() > 1 ? &t->Node(1) : nullptr)
                            : &t->Node(0);
      if (target != nullptr) {
        target_value = target->value;
        target_velocity = target->velocity;
      } else {
        target_value = values_[i];
        target_velocity = current_velocity;
      }

      // Initialize curve to go from current to target.
      d.elapsed_time = 0.0f;
      d.q = CalculateQuadraticEaseInEaseOut(
          current_value, current_velocity, d.start_second_derivative_abs,
          target_value, target_velocity, d.end_second_derivative_abs,
          d.typical_delta_value, d.typical_total_time);
      d.target_time = d.q.total_x();
      d.q_start_time = 0.0f;
      values_[i] = current_value;
    }
  }

 protected:
  virtual void InitializeIndices(const MotivatorInit& init, MotiveIndex index,
                                 MotiveDimension dimensions,
                                 MotiveEngine* /*engine*/) {
    for (MotiveIndex i = index; i < index + dimensions; ++i) {
      Data(i) = EaseInEaseOutData(static_cast<const EaseInEaseOutInit&>(init));
      values_[i] = 0.0f;
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
