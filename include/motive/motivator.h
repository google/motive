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

#ifndef MOTIVE_MOTIVATOR_H
#define MOTIVE_MOTIVATOR_H

#include "motive/processor.h"

namespace motive {

class MotiveEngine;

/// @class Motivator
/// @brief Drives a value towards a target value, or along a path.
///
/// The value can be one-dimensional (e.g. a float), or multi-dimensional
/// (e.g. a matrix). The dimension is determined by the sub-class:
/// Motivator1f drives a float, MatrixMotivator4f drives a 4x4 float matrix.
///
/// Although you can instantiate a Motivator, you probably will not, since
/// there is no mechanism to read data out of a Motivator. Generally, you
/// will instantiate a derived class like Motivator1f, which provides
/// accessor functions.
///
/// The way a Motivator's value moves towards its target is determined by the
/// **type** of a motivator. The type is specified in Motivator::Initialize().
///
/// Note that a Motivator does not store any data itself. It is a handle into
/// a MotiveProcessor. Each MotiveProcessor holds all data for motivators
/// of its **type**.
///
/// Only one Motivator can reference a specific index in a MotiveProcessor.
/// Therefore, when you copy a Motivator, the original motivator will become
/// invalid.
///
class Motivator {
 public:
  Motivator() : processor_(nullptr), index_(kMotiveIndexInvalid) {}

  /// Transfer ownership of `original` motivator to `this` motivator.
  /// `original` motivator is reset and must be initialized again before being
  /// read. We want to allow moves primarily so that we can have vectors of
  /// Motivators.
  ///
  /// Note: This should be a move constructor instead of a copy constructor.
  ///       However, VS2010~2012 requires move constructors to exist in any
  ///       class that has a move constructed member. That would be a burden
  ///       for users of Motivator, so we chose to be practical here instead of
  ///       pedantically correct. We use the copy constructor and copy operator
  ///       to do move behavior.
  ///       See http://en.cppreference.com/w/cpp/language/move_operator
  Motivator(const Motivator& original) {
    if (original.Valid()) {
      original.processor_->TransferMotivator(original.index_, this);
    } else {
      processor_ = nullptr;
      index_ = kMotiveIndexInvalid;
    }
  }

  /// Allow Motivators to be moved. `original` is reset.
  /// See the copy constructor for details.
  Motivator& operator=(const Motivator& original) {
    Invalidate();
    if (original.processor_ != nullptr) {
      original.processor_->TransferMotivator(original.index_, this);
    }
    return *this;
  }

  /// Remove ourselves from the MotiveProcessor when we're deleted.
  ~Motivator() { Invalidate(); }

  /// Detatch this Motivator from its MotiveProcessor. Functions other than
  /// Initialize() and Valid() can no longer be called afterwards.
  void Invalidate() {
    if (processor_ != nullptr) {
      processor_->RemoveMotivator(index_);
    }
  }

  /// Return true if this Motivator is currently being driven by a
  /// MotiveProcessor. That is, if it has been successfully initialized.
  bool Valid() const { return processor_ != nullptr; }

  /// Check consistency of internal state. Useful for debugging.
  /// If this function ever returns false, there has been some sort of memory
  /// corruption or similar bug.
  bool Sane() const {
    return (processor_ == nullptr && index_ == kMotiveIndexInvalid) ||
           (processor_ != nullptr && processor_->ValidMotivator(index_, this));
  }

  /// Return the type of Motivator we've been initilized to.
  /// A Motivator can take on any type that matches its dimension.
  /// The Motivator's type is determined by the `init` param in Initialize().
  MotivatorType Type() const { return processor_->Type(); }

  /// The number of basic values that this Motivator is driving. For example,
  /// a 3D position would return 3, since it drives three floats. A single
  /// 4x4 matrix would return 1, since it's driving one matrix. The basic
  /// value is determined by the MotiveProcessor backing this motivator.
  MotiveDimension Dimensions() const { return processor_->Dimensions(index_); }

 protected:
  Motivator(const MotivatorInit& init, MotiveEngine* engine,
            MotiveDimension dimensions)
      : processor_(nullptr), index_(kMotiveIndexInvalid) {
    InitializeWithDimension(init, engine, dimensions);
  }

  void InitializeWithDimension(const MotivatorInit& init, MotiveEngine* engine,
                               MotiveDimension dimensions);

  /// The MotiveProcessor uses the functions below. It does not modify data
  /// directly.
  friend class MotiveProcessor;

  /// These should only be called by MotiveProcessor!
  void Init(MotiveProcessor* processor, MotiveIndex index) {
    processor_ = processor;
    index_ = index;
  }
  void Reset() { Init(nullptr, kMotiveIndexInvalid); }
  const MotiveProcessor* Processor() const { return processor_; }

  /// All calls to an Motivator are proxied to an MotivatorProcessor. Motivator
  /// data and processing is centralized to allow for scalable optimizations
  /// (e.g. SIMD or parallelization).
  MotiveProcessor* processor_;

  /// A MotiveProcessor processes one MotivatorType, and hosts every Motivator
  /// of that type. This index here uniquely identifies this Motivator to the
  /// MotiveProcessor.
  MotiveIndex index_;
};

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

/// @class MatrixMotivator4fTemplate
/// @brief Drive a 4x4 float matrix from a series of basic transformations.
///
/// The underlying basic transformations can be animated with
/// SetChildTarget1f(), and set to fixed values with SetChildValue1f() and
/// SetChildValue3f().
///
/// Internally, we use mathfu::mat4 as our matrix type and mathfu::vec3 as
/// our vector type, but external we allow any matrix type to be specified
/// via the VectorConverter template parameter.
///
template <class VectorConverter>
class MatrixMotivator4fTemplate : public Motivator {
 public:
  typedef VectorConverter C;
  typedef typename VectorConverter::Matrix4 Mat4;
  typedef typename VectorConverter::Vector3 Vec3;
  typedef MotivatorXfTemplate<C, 1> Mot1f;

  MatrixMotivator4fTemplate() {}
  MatrixMotivator4fTemplate(const MotivatorInit& init, MotiveEngine* engine)
      : Motivator(init, engine, 1) {}

  /// Initialize to the type specified by `init`.
  void Initialize(const MotivatorInit& init, MotiveEngine* engine) {
    InitializeWithDimension(init, engine, 1);
  }

  /// Return the current value of the Motivator. The processor returns a
  /// vector-aligned matrix, so the cast should be valid for any user-defined
  /// matrix type.
  const Mat4& Value() const {
    // TODO: Return by value here. Necessary for matrix formats that are not
    //       byte-wise compatible. For example, row-major matrices.
    return reinterpret_cast<const Mat4&>(Processor().Value(index_));
  }

  /// Return the translation component of the matrix.
  /// The matrix is a 3D affine transform, so the translation component is the
  /// fourth column.
  Vec3 Position() const {
    const mathfu::vec3 position =
        Processor().Value(index_).TranslationVector3D();
    return C::FromPtr(&position[0], Vec3());
  }

  /// Return the time remaining in the current spline animation.
  /// Time units are defined by the user.
  MotiveTime TimeRemaining() const { return Processor().TimeRemaining(index_); }

  /// Query the number of matrix operations. This equals the number of
  /// operations in the `init` initializer.
  int NumChildren() const { return Processor().NumChildren(index_); }

  /// Return the current value of the `child_index`th basic transform that
  /// drives this matrix.
  /// @param child_index The index into MatrixInit::ops(). The ops() array
  ///                    is a series of basic transformation operations that
  ///                    compose this matrix. Each basic transformation has
  ///                    a current value. We gather this value here.
  float ChildValue1f(MotiveChildIndex child_index) const {
    float r;
    Processor().ChildValues(index_, child_index, 1, &r);
    return r;
  }

  /// Returns the current values of the basic transforms at indices
  /// (child_index, child_index + 1, child_index + 2).
  /// Useful when you drive all the (x,y,z) components of a translation, scale,
  /// or rotation.
  /// @param child_index The first index into MatrixInit::ops(). The value at
  ///                    this index is returned in the x component of Vec3.
  ///                    y gets child_index + 1's value, and
  ///                    z gets child_index + 2's value.
  Vec3 ChildValue3f(MotiveChildIndex child_index) const {
    Vec3 r;
    Processor().ChildValues(index_, child_index, 3, C::ToPtr(r));
    return r;
  }

  /// Returns the Motivator1f that's driving this child, if it's driven by
  /// a Motivator1f. Otherwise, returns nullptr.
  const Mot1f* ChildMotivator1f(MotiveChildIndex child_index) const {
    return static_cast<const Mot1f*>(
        Processor().ChildMotivator1f(index_, child_index));
  }

  /// Set the target the 'child_index'th basic transform.
  /// Each basic transform can be driven by a child motivator.
  /// This call lets us control those child motivators.
  /// @param child_index The index into the MatrixInit::ops() that was passed
  ///                    into Initialize(). This operation must have been
  ///                    initialized with a MotivatorInit, not a constant value.
  /// @param t The target values for the basic transform to animate towards.
  ///          Also, optionally, the current value for it to jump to.
  void SetChildTarget1f(MotiveChildIndex child_index, const MotiveTarget1f& t) {
    Processor().SetChildTarget1f(index_, child_index, t);
  }

  /// Set the constant value of a child. Each basic matrix transformation
  /// can be driven by a constant value instead of a Motivator.
  /// This call lets us set those constant values.
  /// @param child_index The index into the MatrixInit::ops() that was passed
  ///                    into Initialize(). This operation must have been
  ///                    initialized with a constant value, not a MotivatorInit.
  /// @param value The new constant value of this operation.
  void SetChildValue1f(MotiveChildIndex child_index, float value) {
    Processor().SetChildValues(index_, child_index, 1, &value);
  }

  /// Set the constant values of the basic transforms at indices
  /// (child_index, child_index + 1, child_index + 2).
  /// @param child_index The first index into MatrixInit::ops(). The constant
  ///                    value at this index is set to the x component of
  ///                    `value`.
  ///                    child_index + 1's constant is set to value.y, and
  ///                    child_index + 2's constant is set to value.z.
  /// @param value The new constant value for this 3-dimensional operation.
  void SetChildValue3f(MotiveChildIndex child_index, const Vec3& value) {
    Processor().SetChildValues(index_, child_index, 3, C::ToPtr(value));
  }

  /// Match existing MatrixOps with those in `ops` and smoothly transition
  /// to the new parameters in `ops`.
  void BlendToOps(const MatrixOpArray& ops, const SplinePlayback& playback) {
    Processor().BlendToOps(index_, ops, playback);
  }

  void SetPlaybackRate(float playback_rate) {
    Processor().SetPlaybackRate(index_, playback_rate);
  }

 private:
  MatrixProcessor4f& Processor() {
    return *static_cast<MatrixProcessor4f*>(processor_);
  }
  const MatrixProcessor4f& Processor() const {
    return *static_cast<const MatrixProcessor4f*>(processor_);
  }
};

class RigMotivator : public Motivator {
 public:
  RigMotivator() {}
  RigMotivator(const MotivatorInit& init, MotiveEngine* engine)
      : Motivator(init, engine, 1) {}

  /// Initialize to the type specified by `init`. The only type defined
  /// within Motive for `init` is RigInit, but you can register your
  /// own RigProcessor classes if you like.
  void Initialize(const MotivatorInit& init, MotiveEngine* engine) {
    InitializeWithDimension(init, engine, 1);
  }

  /// Blend from the current state to the animation specified in `anim`.
  /// Blend time is specified in `anim` itself.
  /// If the current state is unspecified because no animation
  /// has yet been played, snap to `anim`.
  void BlendToAnim(const RigAnim& anim, const SplinePlayback& playback) {
    Processor().BlendToAnim(index_, anim, playback);
  }

  void SetPlaybackRate(float playback_rate) {
    Processor().SetPlaybackRate(index_, playback_rate);
  }

  /// Returns array of matricies: one for each bone position. The matrices are
  /// all in the space of the root bones. That is, the bone hierarchy has been
  /// flattened.
  const mathfu::AffineTransform* GlobalTransforms() const {
    return Processor().GlobalTransforms(index_);
  }

  /// Returns the shell aniamtion that defines this rig. It contains all the
  /// bones and operations-on-those-bones that can be animated.
  ///
  /// Distinction,
  /// Rig: defines the bone heirarchy.
  /// Defining animation: defines the bone heirarchy + operations on each bone.
  /// Operations-on-bone: one of MatrixOperationType, for example, a rotation
  ///     about the x-axis, or a translation along the y-axis. Animations are
  ///     composed of several such operations on each bone. Not every animation
  ///     has all the operations, however. The defining animation is the union
  ///     of all possible operations on each bone.
  const RigAnim* DefiningAnim() const {
    return Processor().DefiningAnim(index_);
  }

  /// Return the time remaining in the current spline animation.
  /// Time units are defined by the user.
  MotiveTime TimeRemaining() const { return Processor().TimeRemaining(index_); }

  std::string CsvHeaderForDebugging() const {
    return Processor().CsvHeaderForDebugging(index_);
  }
  std::string CsvValuesForDebugging() const {
    return Processor().CsvValuesForDebugging(index_);
  }
  std::string LocalTransformsForDebugging(BoneIndex bone) const {
    return Processor().LocalTransformsForDebugging(index_, bone);
  }

 private:
  RigProcessor& Processor() { return *static_cast<RigProcessor*>(processor_); }
  const RigProcessor& Processor() const {
    return *static_cast<const RigProcessor*>(processor_);
  }
};

// These Motivator types use mathfu in their external API.
typedef MotivatorXfTemplate<MathFuVectorConverter, 1> Motivator1f;
typedef MotivatorXfTemplate<MathFuVectorConverter, 2> Motivator2f;
typedef MotivatorXfTemplate<MathFuVectorConverter, 3> Motivator3f;
typedef MotivatorXfTemplate<MathFuVectorConverter, 4> Motivator4f;
typedef MatrixMotivator4fTemplate<MathFuVectorConverter> MatrixMotivator4f;

}  // namespace motive

#endif  // MOTIVE_MOTIVATOR_H
