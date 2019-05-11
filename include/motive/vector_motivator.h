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

#ifndef MOTIVE_VECTOR_MOTIVATOR_H_
#define MOTIVE_VECTOR_MOTIVATOR_H_

#include "motive/motivator.h"
#include "motive/vector_processor.h"

namespace motive {

/// @class MotivatorNfBase
/// @brief Animate an array of N-floats, where N is set by the `dimension` in
///        the constructor.
class MotivatorNf : public Motivator {
 public:
  /// Motivator is created in a reset state. When in the reset state,
  /// it is not being driven, and Value(), Velocity(), etc. cannot be called.
  MotivatorNf() {}

  /// Initialize to the motion algorithm specified by `init`.
  /// Current and target values are not set.
  MotivatorNf(const MotivatorInit& init, MotiveEngine* engine,
              MotiveDimension dimensions)
      : Motivator(init, engine, dimensions) {}

  /// Initialize to the motion algorithm specified by `init`.
  /// Set current and target values as specified by `ts`.
  MotivatorNf(const MotivatorInit& init, MotiveEngine* engine,
              MotiveDimension dimensions, const MotiveTarget1f* ts)
      : Motivator(init, engine, dimensions) {
    SetTargets(ts);
  }

  /// Initialize this Motivator to the type specified in init.type.
  /// @param init Defines the type and initial state of the Motivator.
  /// @param engine The engine that will update this Motivator when
  ///               engine->AdvanceFrame() is called.
  /// @param dimensions The number of slots to occupy. For example, a 3D
  ///                   position would occupy three slots.
  void Initialize(const MotivatorInit& init, MotiveEngine* engine,
                  MotiveDimension dimensions) {
    InitializeWithDimension(init, engine, dimensions);
  }

  /// Initialize to the motion algorithm specified by `init`.
  /// Set current and target values as specified by `targets`, an array of
  /// length `dimensions`.
  void InitializeWithTargets(const MotivatorInit& init, MotiveEngine* engine,
                             MotiveDimension dimensions,
                             const MotiveTarget1f* targets) {
    Initialize(init, engine, dimensions);
    SetTargets(targets);
  }

  /// Initialize to the motion algorithm specified by `init`.
  /// Set target values and velocities as specified by 'target_values' and
  /// 'target_velocities', arrays of length `dimensions`, with shape specified
  /// by 'shape'.
  void InitializeWithTargetShape(const MotivatorInit& init,
                                 MotiveEngine* engine,
                                 MotiveDimension dimensions,
                                 const MotiveCurveShape& shape,
                                 const float* target_values,
                                 const float* target_velocities) {
    InitializeWithDimension(init, engine, dimensions);
    SetTargetWithShape(target_values, target_velocities, shape);
  }

  /// Initialize to the motion algorithm specified by `init`.
  /// Set movement to follow the curves specified in `splines`, an array of
  /// length `dimensions, and modified by `playback`.
  void InitializeWithSplines(const MotivatorInit& init, MotiveEngine* engine,
                             MotiveDimension dimensions,
                             const CompactSpline* splines,
                             const SplinePlayback& playback) {
    Initialize(init, engine, dimensions);
    SetSplines(splines, playback);
  }

  // Get array of length `dimensions`.
  const float* Values() const { return Processor().Values(index_); }
  void Velocities(float* out) const {
    return Processor().Velocities(index_, Dimensions(), out);
  }
  void Directions(float* out) const {
    return Processor().Directions(index_, Dimensions(), out);
  }
  void TargetValues(float* out) const {
    return Processor().TargetValues(index_, Dimensions(), out);
  }
  void TargetVelocities(float* out) const {
    return Processor().TargetVelocities(index_, Dimensions(), out);
  }
  void Differences(float* out) const {
    return Processor().Differences(index_, Dimensions(), out);
  }

  /// Returns time remaining until target is reached.
  /// The unit of time is determined by the calling program.
  MotiveTime TargetTime() const {
    return Processor().TargetTime(index_, Dimensions());
  }

  /// Returns the shape of the current curve.
  MotiveCurveShape MotiveShape() const {
    return Processor().MotiveShape(index_);
  }

  /// Returns the current time (i.e. the x-value) in the current spline.
  /// If Motivator is not being driven by a spline, returns 0.
  /// Whenever SetSpline() is called, this value will be reset to the
  /// start_time specified in SplinePlayback. Every time
  /// MotiveEngine::AdvanceFrame() is called, it will increment by
  /// `delta_time` * `playback_rate`. If the SplinePlayback has repeat=true,
  /// then SplineTime() will periodically loop back to time 0.
  MotiveTime SplineTime() const { return Processor().SplineTime(index_); }

  /// Gather pointers to the splines currently being played, on each dimension.
  /// @param splines Output array of length Dimensions().
  void Splines(const CompactSpline** splines) const {
    Processor().Splines(index_, Dimensions(), splines);
  }

  /// Follow the curve specified in `spline`. Overrides the existing current
  /// value.
  /// @param spline The spline to follow. Array of length Dimensions().
  /// @param playback The time into the splines to initiate playback,
  ///                 the blend time to the splines, and whether to repeat
  ///                 from the beginning after the end of the spline is reached.
  void SetSpline(const CompactSpline& spline, const SplinePlayback& playback) {
    assert(Dimensions() == 1);
    Processor().SetSplines(index_, Dimensions(), &spline, playback);
  }

  /// Follow the curves specified in `splines`. Overrides the existing current
  /// value.
  /// @param splines The splines that the curves should follow.
  ///                Array of length Dimensions().
  /// @param playback The time into the splines to initiate playback,
  ///                 the blend time to the splines, and whether to repeat
  ///                 from the beginning after the end of the spline is reached.
  void SetSplines(const CompactSpline* splines,
                  const SplinePlayback& playback) {
    Processor().SetSplines(index_, Dimensions(), splines, playback);
  }

  /// Seek to a specific time in the spline.
  /// @param time The time (in the spline's x-axis) to seek to.
  void SetSplineTime(MotiveTime time) {
    Processor().SetSplineTime(index_, Dimensions(), time);
  }

  /// Set rate at which we consume the spline set in SetSpline().
  ///     0   ==> paused
  ///     0.5 ==> half speed (slow motion)
  ///     1   ==> authored speed
  ///     2   ==> double speed (fast forward)
  void SetSplinePlaybackRate(float playback_rate) {
    Processor().SetSplinePlaybackRate(index_, Dimensions(), playback_rate);
  }

  /// Set the repeat state for the spline set in SetSpline().
  void SetSplineRepeating(bool repeat) {
    Processor().SetSplineRepeating(index_, Dimensions(), repeat);
  }

  /// Set the target and (optionally the current) motivator values.
  /// Use this call to procedurally drive the Motivator towards a specific
  /// target. The Motivator will transition smoothly to the new target.
  /// @param targets The targets that each value should achieve.
  ///                An array of length Dimensions().
  void SetTargets(const MotiveTarget1f* targets) {
    Processor().SetTargets(index_, Dimensions(), targets);
  }

  /// Set the target values, velocities, and curve shape for the motivator.
  /// Procedurally drive the Motivator to 'target_values' and
  /// 'target_velocities' following a curve defined by 'shape'.
  /// Setting the target with the shape makes it so that a time does not need
  /// to be specified, as it will be calculated. In contrast, if the time needed
  /// to achieve a value is to be user-provided, @ref SetTarget should be used
  /// instead.
  /// @param target_value Array of target values with length Dimensions.
  /// @param target_velocity Array of target velocities with length Dimensions.
  /// @param shape The shape of the curve we'll create, as determined by the
  ///              curve's typical delta value, typical total time, and bias.
  void SetTargetWithShape(const float* target_values,
                          const float* target_velocities,
                          const MotiveCurveShape& shape) {
    Processor().SetTargetWithShape(index_, Dimensions(), target_values,
                                   target_velocities, shape);
  }

  /// Drive some channels with splines and others with targets.
  /// For i between 0 and Dimensions()-1, if splines[i] != NULL drive
  /// channel i with splines[i]. Otherwise, drive channel i with targets[i].
  /// @param splines Array of pointers to splines, length Dimensions().
  ///                Pointers can be NULL.
  /// @param playback Various parameters for `splines`.
  /// @param targets Array of targets that are used when splines are not
  ///                specified. Length Dimensions().
  void SetSplinesAndTargets(const CompactSpline* const* splines,
                            const SplinePlayback& playback,
                            const MotiveTarget1f* targets) {
    Processor().SetSplinesAndTargets(index_, Dimensions(), splines, playback,
                                     targets);
  }

 protected:
  MotiveProcessorNf& Processor() {
    return *static_cast<MotiveProcessorNf*>(processor_);
  }
  const MotiveProcessorNf& Processor() const {
    return *static_cast<const MotiveProcessorNf*>(processor_);
  }
};

/// @class MotivatorXfTemplate
/// @brief Animate a vector of floating-point values.
///
/// Wraps `MotivatorNf` to return by value, instead of using `out` arrays.
/// For `kDimension = 1`, the vector type is just `float`.
/// For `kDimension > 1`, the vector type is determined by `VectorConverter`.
/// We use `MathFuVectorConverter` below to create 2, 3, and 4 dimensional
/// motivators (see Motivator2f, Motivator3f, and Motivator4f).
/// You can use your own VectorConverter to create Motivators of that match
/// your vector types.
///
template <class VectorConverter, MotiveDimension kDimensionsParam>
class MotivatorXfTemplate : public MotivatorNf {
 public:
  typedef VectorConverter C;
  static const MotiveDimension kDimensions = kDimensionsParam;
  typedef typename VectorT<C, kDimensions>::type Vec;
  typedef typename MotiveTargetT<kDimensions>::type Target;
  typedef MotiveTargetBuilderTemplate<C, kDimensions> TargetBuilder;

  /// Motivator is created in a reset state. When in the reset state,
  /// it is not being driven, and Value(), Velocity(), etc. cannot be called.
  MotivatorXfTemplate() {}

  /// Initialize to the motion algorithm specified by `init`.
  /// Current and target values are not set.
  MotivatorXfTemplate(const MotivatorInit& init, MotiveEngine* engine)
      : MotivatorNf(init, engine, kDimensions) {}

  /// Initialize to the motion algorithm specified by `init`.
  /// Set current and target values as specified by `t`.
  MotivatorXfTemplate(const MotivatorInit& init, MotiveEngine* engine,
                      const Target& t)
      : MotivatorNf(init, engine, kDimensions) {
    SetTarget(t);
  }

  /// Initialize to the motion algorithm specified by `init`.
  void Initialize(const MotivatorInit& init, MotiveEngine* engine) {
    InitializeWithDimension(init, engine, kDimensions);
  }

  /// Initialize to the motion algorithm specified by `init`.
  /// Set current and target values as specified by `t`.
  void InitializeWithTarget(const MotivatorInit& init, MotiveEngine* engine,
                            const Target& t) {
    InitializeWithDimension(init, engine, kDimensions);
    SetTarget(t);
  }

  /// Initialize to the motion algorithm specified by `init`.
  /// Set target values and velocities as specified by 'target_values' and
  /// 'target_velocities', arrays of length `dimensions`, with shape specified
  /// by 'shape'.
  void InitializeWithTargetShape(const MotivatorInit& init,
                                 MotiveEngine* engine,
                                 MotiveDimension dimensions,
                                 const MotiveCurveShape& shape,
                                 const Vec& target_values,
                                 const Vec& target_velocities) {
    InitializeWithDimension(init, engine, dimensions);
    SetTargetWithShape(target_values, target_velocities, shape);
  }

  /// Returns the current motivator value. The current value is updated when
  /// engine->AdvanceFrame() is called on the `engine` that initialized this
  /// Motivator.
  /// Note that the "Vec()" parameter is just a syntactic hack used to access
  /// the correct overloaded function in the processor.
  Vec Value() const { return C::FromPtr(Processor().Values(index_), Vec()); }

  /// Returns the current rate of change of this motivator. For example,
  /// if this Motivator is being driven by a spline, returns the derivative
  /// at the current time in the spline curve.
  Vec Velocity() const {
    Vec r;
    Processor().Velocities(index_, kDimensions, C::ToPtr(r));
    return r;
  }

  /// Returns the velocity when playback rate is 1. Useful to know the
  /// direction of a multi-dimensional motivator, even when playback rate
  /// is 0.
  Vec Direction() const {
    Vec r;
    Processor().Directions(index_, kDimensions, C::ToPtr(r));
    return r;
  }

  /// Returns the value this Motivator is driving towards.
  /// If being driven by a spline, returns the value at the end of the spline.
  Vec TargetValue() const {
    Vec r;
    Processor().TargetValues(index_, kDimensions, C::ToPtr(r));
    return r;
  }

  /// Returns the rate-of-change of this Motivator once it reaches
  /// TargetValue().
  Vec TargetVelocity() const {
    Vec r;
    Processor().TargetVelocities(index_, kDimensions, C::ToPtr(r));
    return r;
  }

  /// Returns TargetValue() minus Value(). If we're driving a
  /// modular type (e.g. an angle), this may not be the naive subtraction.
  /// For example, if TargetValue() = 170 degrees, Value() = -170 degrees,
  /// then Difference() = -20 degrees.
  Vec Difference() const {
    Vec r;
    Processor().Differences(index_, kDimensions, C::ToPtr(r));
    return r;
  }

  /// Set the target and (optionally the current) motivator values.
  /// Use this call to procedurally drive the Motivator towards a specific
  /// target. The Motivator will transition smoothly to the new target.
  /// You can change the target value every frame if you like, and the
  /// Motivator value should behave calmly but responsively, with the
  /// movement qualities of the underlying MotiveProcessor.
  /// Note that the underlying MotiveProcessor is allowed to ignore
  /// parts of `t` that are irrelevent to its algorithm.
  /// Note also that if the time needed to achieve a value is to be
  /// user-provided, this function should be used. If the time should
  /// be calculated instead of user-specified, @ref SetTargetWithShape
  /// should be used instead.
  /// @param t A set of waypoints to hit, optionally including the current
  ///          value. If the current value is not included, maintain the
  ///          existing current value.
  void SetTarget(const Target& t) {
    Processor().SetTargets(index_, kDimensions, t.targets());
  }

  /// Set the target values, velocity, and curve shape for the motivator.
  /// Use this call to procedurally drive the Motivator towards that target.
  /// Setting the target with the shape makes it so that a time does not need
  /// to be specified, as it will be calculated. In contrast, if the time needed
  /// to achieve a value is to be user-provided, @ref SetTarget should be used
  /// instead.
  /// @param target_value The target value to hit.
  /// @param target_velocity The velocity with which to hit the target value.
  /// @param shape The shape of the curve we'll create, such as the curve's
  ///              typical delta value, typical total time, and bias.
  void SetTargetWithShape(const Vec& target_value, const Vec& target_velocity,
                          const MotiveCurveShape& shape) {
    Processor().SetTargetWithShape(index_, kDimensions, C::ToPtr(target_value),
                                   C::ToPtr(target_velocity), shape);
  }

  MotiveDimension Dimensions() const { return kDimensions; }
};

// These Motivator types use mathfu in their external API.
typedef MotivatorXfTemplate<MathFuVectorConverter, 1> Motivator1f;
typedef MotivatorXfTemplate<MathFuVectorConverter, 2> Motivator2f;
typedef MotivatorXfTemplate<MathFuVectorConverter, 3> Motivator3f;
typedef MotivatorXfTemplate<MathFuVectorConverter, 4> Motivator4f;

}  // namespace motive

#endif  // MOTIVE_VECTOR_MOTIVATOR_H_
