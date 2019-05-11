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

#ifndef MOTIVE_VECTOR_PROCESSOR_H_
#define MOTIVE_VECTOR_PROCESSOR_H_

#include "motive/processor.h"

namespace motive {

/// @class MotiveProcessorNf
/// @brief Interface for motivator types that drive a single float value.
///
/// That is, for MotiveProcessors that interface with MotivatorNf or
/// MotivatorXf.
class MotiveProcessorNf : public MotiveProcessor {
 public:
  // Convenience functions for getting a single value. Prefer calling the
  // bulk values, especially when inside a loop. They avoid the virtual
  // function call overhead, and offer more opportunities for optimizations.
  float Value(MotiveIndex index) const { return Values(index)[0]; }
  float Velocity(MotiveIndex index) const {
    float v;
    Velocities(index, 1, &v);
    return v;
  }
  float Direction(MotiveIndex index) const {
    float v;
    Directions(index, 1, &v);
    return v;
  }
  float TargetValue(MotiveIndex index) const {
    float v;
    TargetValues(index, 1, &v);
    return v;
  }
  float TargetVelocity(MotiveIndex index) const {
    float v;
    TargetVelocities(index, 1, &v);
    return v;
  }
  float Difference(MotiveIndex index) const {
    float v;
    Differences(index, 1, &v);
    return v;
  }

  virtual const float* Values(MotiveIndex index) const = 0;
  virtual void Velocities(MotiveIndex index, MotiveDimension dimensions,
                          float* out) const = 0;
  virtual void Directions(MotiveIndex index, MotiveDimension dimensions,
                          float* out) const {
    Velocities(index, dimensions, out);
  }
  virtual void TargetValues(MotiveIndex index, MotiveDimension dimensions,
                            float* out) const = 0;
  virtual void TargetVelocities(MotiveIndex index, MotiveDimension dimensions,
                                float* out) const = 0;
  virtual void Differences(MotiveIndex index, MotiveDimension dimensions,
                           float* out) const = 0;

  virtual MotiveTime TargetTime(MotiveIndex index,
                                MotiveDimension dimensions) const = 0;
  virtual MotiveTime SplineTime(MotiveIndex /*index*/) const { return 0; }

  virtual MotiveCurveShape MotiveShape(MotiveIndex /*index*/) const {
    return MotiveCurveShape();
  }

  // At least one of SetTargets, SetTargetWithShape, or SetSplines should be
  // implemented by the derived class. Otherwise, there will be no way to drive
  // the Motivator towards a target.
  //
  // Set the current and future values that we want the Motivator to achieve.
  virtual void SetTargets(MotiveIndex /*index*/, MotiveDimension /*dimensions*/,
                          const MotiveTarget1f* /*ts*/) {}

  // Set the target we want the Motivator to achieve and describe the curve
  // shape it should use to get there.
  virtual void SetTargetWithShape(MotiveIndex /*index*/,
                                  MotiveDimension /*dimensions*/,
                                  const float* /*target_values*/,
                                  const float* /*target_velocities*/,
                                  const MotiveCurveShape& /*shape*/) {}

  // Drive the Motivator by following splines specified in the playback.
  virtual void SetSplines(MotiveIndex /*index*/, MotiveDimension /*dimensions*/,
                          const CompactSpline* /*splines*/,
                          const SplinePlayback& /*playback*/) {}

  // Gather the splines currently being played back. If dimension is not being
  // driven by a spline, returns nullptr at that dimension.
  virtual void Splines(MotiveIndex /*index*/, MotiveIndex count,
                       const CompactSpline** splines) const {
    for (MotiveIndex i = 0; i < count; ++i) splines[i] = nullptr;
  }

  // For each i from 0..dimensions-1, drive the value with with splines[i]
  // when splines[i] != NULL, and with targets[i] otherwise.
  virtual void SetSplinesAndTargets(MotiveIndex /*index*/,
                                    MotiveDimension /*dimensions*/,
                                    const CompactSpline* const* /*splines*/,
                                    const SplinePlayback& /*playback*/,
                                    const MotiveTarget1f* /*targets*/) {}

  virtual void SetSplineTime(MotiveIndex /*index*/,
                             MotiveDimension /*dimensions*/,
                             MotiveTime /*time*/) {}
  virtual void SetSplinePlaybackRate(MotiveIndex /*index*/,
                                     MotiveDimension /*dimensions*/,
                                     float /*playback_rate*/) {}
  virtual void SetSplineRepeating(MotiveIndex /*index*/,
                                  MotiveDimension /*dimensions*/,
                                  bool /*repeat*/) {}
};

}  // namespace motive

#endif  // MOTIVE_VECTOR_PROCESSOR_H_
