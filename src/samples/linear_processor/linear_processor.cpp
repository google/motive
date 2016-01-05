// Copyright 2015 Google Inc. All rights reserved.
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
#include "motive/math/curve.h"

using motive::MotivatorInit;
using motive::MotiveProcessorNf;
using motive::MotiveDimension;
using motive::MotiveEngine;
using motive::MotiveIndex;
using motive::MotiveTime;
using motive::MotiveTarget1f;
using motive::MotivatorType;
using motive::MotiveNode1f;
using motive::Motivator1f;
using mathfu::vec2;

//! [Own Processor LinearInit]
// The init structure is the only code that should be exposed externally.
// When you call Motivator::Initialize() with initialization parameters of
// type LinearInit, the initialization call will be routed to
// LinearMotiveProcessor::InitializeIndex().
//
class LinearInit : public MotivatorInit {
 public:
  // Defines 'kType' and other functions necessary to register this type of
  // motivator.
  MOTIVE_INTERFACE();

  // This motivator type is rather boring. It has no configuration parameters
  // at all. We could add some extra parameters here, though, if we like.
};
//! [Own Processor LinearInit]

//! [Own Processor LinearMotiveProcessor]
class LinearMotiveProcessor : public MotiveProcessorNf {
 public:
  virtual ~LinearMotiveProcessor() {}

  // Once per frame, the MotiveEngine calls this function. In this one call,
  // we advance the simulation of _all_ linear Motivators. Bulk processing
  // allows us to use SIMD or multi-threading when appropriate.
  virtual void AdvanceFrame(MotiveTime delta_time) {
    // We could optimize this using SIMD.
    for (size_t i = 0; i < data_.size(); ++i) {
      LinearData& d = data_[i];
      float& value = values_[i];

      // Advance the value by linearly interpolating towards the target.
      if (d.target_time > 0.0f) {
        const float percent_complete = delta_time / d.target_time;
        value = mathfu::Lerp(value, d.target_value, percent_complete);
      } else {
        value = d.target_value;
      }

      // Decrement the target time.
      d.target_time -= delta_time;
    }
  }

  virtual MotivatorType Type() const { return LinearInit::kType; }
  virtual int Priority() const { return 0; }

  // Accessors to allow the user to get and set simulation values.
  virtual const float* Values(MotiveIndex index) const {
    return &values_[index];
  }
  virtual void Velocities(MotiveIndex index, MotiveIndex count,
                          float* out) const {
    const LinearData* d = &Data(index);
    const float* values = &values_[index];
    for (MotiveDimension i = 0; i < count; ++i, ++d, ++values) {
      out[i] = d->target_time <= 0.0f ? 0.0f : (d->target_value - *values) /
                                                   d->target_time;
    }
  }
  virtual void TargetValues(MotiveIndex index, MotiveIndex count,
                            float* out) const {
    const LinearData* d = &Data(index);
    for (MotiveDimension i = 0; i < count; ++i, ++d) {
      out[i] = d->target_value;
    }
  }
  virtual void TargetVelocities(MotiveIndex /*index*/, MotiveIndex count,
                                float* out) const {
    for (MotiveDimension i = 0; i < count; ++i) {
      out[i] = 0.0f;
    }
  }
  virtual void Differences(MotiveIndex index, MotiveIndex count,
                           float* out) const {
    const LinearData* d = &Data(index);
    const float* values = &values_[index];
    for (MotiveDimension i = 0; i < count; ++i, ++d, ++values) {
      out[i] = d->target_value - *values;
    }
  }
  virtual MotiveTime TargetTime(MotiveIndex index) const {
    return static_cast<MotiveTime>(Data(index).target_time);
  }

  // Target values are set in bulk. Please see MotiveTarget1f for a description
  // of the format. It's basically an array of way points. In our case, we're
  // only interested in (at most) two way points: current and target.
  virtual void SetTargets(MotiveIndex index, MotiveIndex count,
                          const MotiveTarget1f* ts) {
    for (int i = 0; i < count; ++i) {
      LinearData& d = Data(index + i);
      const MotiveTarget1f& t = ts[i];

      // If the first node specifies time=0, that means we want to override the
      // current values with the values specified in the first node.
      const MotiveNode1f& node0 = t.Node(0);
      const bool override_current = node0.time == 0;
      if (override_current) {
        values_[index + i] = node0.value;
      }

      // If the first node specifies time > 0, that means we want to override
      // the
      // target values with it. Or, if two nodes are specified, we use the
      // second for the target values.
      const MotiveNode1f* target_node =
          override_current ? (t.num_nodes() > 1 ? &t.Node(1) : nullptr)
                           : &node0;
      if (target_node != nullptr) {
        d.target_value = target_node->value;
        d.target_time = static_cast<float>(target_node->time);
      }
    }
  }

 protected:
  struct LinearData {
    float target_value;
    float target_time;

    LinearData() { Reset(); }
    void Reset() {
      target_value = 0.0f;
      target_time = 0.0f;
    }
  };

  // When an Motivator is initialized with LinearInit, this function will
  // be called. We allocate data for the new Motivator.
  virtual void InitializeIndex(const MotivatorInit& init, MotiveIndex index,
                               MotiveEngine* /*engine*/) {
    (void)init;
    assert(init.type() == LinearInit::kType);
    Data(index).Reset();
    values_[index] = 0;
  }

  // This function is called when an index is removed. We don't have to do
  // anything, but for ease of debugging, we call reset.
  virtual void RemoveIndex(MotiveIndex index) {
    Data(index).Reset();
    values_[index] = 0;
  }

  // The base class endeavors to keep our data contiguous in memory, so
  // whenever Defragment() is called, we may shuffle some indices around.
  virtual void MoveIndex(MotiveIndex old_index, MotiveIndex new_index) {
    data_[new_index] = data_[old_index];
    values_[new_index] = values_[old_index];
  }

  // When new Motivators are being added or removed, this function may be
  // called. As we grow, resize() might cause reallocs to occur, which is
  // slow. However, once we reach the high-water mark, reallocs will stop.
  // This is a reasonable tradeoff between the benefits and slowdowns of
  // dynamically growing arrays.
  virtual void SetNumIndices(MotiveIndex num_indices) {
    data_.resize(num_indices);
    values_.resize(num_indices);
  }

  // Handy accessors that double-check the validity of 'index'.
  const LinearData& Data(MotiveIndex index) const {
    assert(ValidIndex(index));
    return data_[index];
  }

  LinearData& Data(MotiveIndex index) {
    assert(ValidIndex(index));
    return data_[index];
  }

  // Contiguous array of data. During Defragment(), we plug the holes from
  // indices that have been removed. This allows us to process data by
  // streaming it in, and maximize memory bandwidth.
  std::vector<LinearData> data_;
  std::vector<float> values_;
};

MOTIVE_INSTANCE(LinearInit, LinearMotiveProcessor);
//! [Own Processor LinearMotiveProcessor]

int main() {
  //! [Own Processor Register]
  LinearInit::Register();
  //! [Own Processor Register]

  MotiveEngine engine;

  //! [Own Processor Create Instance]
  // Move from 10 --> -5 in 100 internal time units.
  const MotiveTarget1f target = motive::CurrentToTarget1f(10, 0, -5, 0, 100);

  // Create the one dimensional motivator of type Linear by passing in
  // LinearInit.
  Motivator1f linear_motivator(LinearInit(), &engine, target);
  //! [Own Processor Create Instance]

  //! [Own Processor Advance Simulation]
  // Advance the simulation one tick at a time by calling engine.AdvanceFrame().
  std::vector<vec2> points;
  points.reserve(target.EndTime() + 1);
  for (MotiveTime t = 0; t <= target.EndTime(); ++t) {
    points.push_back(vec2(static_cast<float>(t), linear_motivator.Value()));
    engine.AdvanceFrame(1);
  }
  printf("\n%s",
         motive::Graph2DPoints(&points[0], static_cast<int>(points.size()))
             .c_str());
  //! [Own Processor Advance Simulation]

  return 0;
}
