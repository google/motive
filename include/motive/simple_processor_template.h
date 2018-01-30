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

#ifndef MOTIVE_SIMPLE_PROCESSOR_TEMPLATE_H_
#define MOTIVE_SIMPLE_PROCESSOR_TEMPLATE_H_

#include "motive/engine.h"
#include "motive/simple_init_template.h"
#include "motive/vector_processor.h"

namespace motive {

template <class T>
class SimpleProcessorTemplate : public MotiveProcessorNf {
  template <typename F>
  void GatherFloats(MotiveIndex index, MotiveDimension dimensions, float* out,
                    F value_from_data_fn) const {
    for (MotiveDimension i = 0; i < dimensions; ++i) {
      const MotiveIndex data_idx = index + i;
      const T& d = Data(data_idx);
      out[i] = value_from_data_fn(d, values_[data_idx]);
    }
  }

 public:
  virtual ~SimpleProcessorTemplate() {}

  // Accessors to allow the user to get and set simluation values.
  virtual const float* Values(MotiveIndex index) const {
    return &values_[index];
  }

  virtual void Velocities(MotiveIndex index, MotiveDimension dimensions,
                          float* out) const {
    GatherFloats(index, dimensions, out, [](const T& d, float value) {
      return SimpleVelocity(d, value);
    });
  }

  virtual void TargetValues(MotiveIndex index, MotiveDimension dimensions,
                            float* out) const {
    GatherFloats(index, dimensions, out, [](const T& d, float value) {
      return SimpleTargetValue(d, value);
    });
  }

  virtual void TargetVelocities(MotiveIndex index, MotiveDimension dimensions,
                                float* out) const {
    GatherFloats(index, dimensions, out, [](const T& d, float value) {
      return SimpleTargetVelocity(d, value);
    });
  }

  virtual void Differences(MotiveIndex index, MotiveDimension dimensions,
                           float* out) const {
    GatherFloats(index, dimensions, out, [](const T& d, float value) {
      return SimpleDifference(d, value);
    });
  }

  virtual MotiveTime TargetTime(MotiveIndex index,
                                MotiveDimension dimensions) const {
    MotiveTime greatest = std::numeric_limits<MotiveTime>::min();
    for (MotiveDimension i = 0; i < dimensions; ++i) {
      const T& d = Data(index + i);
      greatest = std::max(greatest, SimpleTargetTime(d));
    }
    return greatest;
  }

 protected:
  virtual void InitializeIndices(const MotivatorInit& init, MotiveIndex index,
                                 MotiveDimension dimensions,
                                 MotiveEngine* /*engine*/) {
    const SimpleInit& simple_init = static_cast<const SimpleInit&>(init);
    for (MotiveDimension i = 0; i < dimensions; ++i) {
      const MotiveIndex processor_index = i + index;
      Data(processor_index) = T(simple_init, i);
      values_[processor_index] = simple_init.start_values[i];
    }
  }

  virtual void RemoveIndices(MotiveIndex index, MotiveDimension dimensions) {
    for (MotiveIndex i = index; i < index + dimensions; ++i) {
      Data(i) = T();
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

  const T& Data(MotiveIndex index) const {
    assert(ValidIndex(index));
    return data_[index];
  }

  T& Data(MotiveIndex index) {
    assert(ValidIndex(index));
    return data_[index];
  }

  std::vector<T> data_;
  std::vector<float> values_;
};

}  // namespace motive

#endif  // MOTIVE_SIMPLE_PROCESSOR_TEMPLATE_H_
