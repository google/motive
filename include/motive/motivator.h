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
/// Motivator1f drives a float, MotivatorMatrix4f drives a 4x4 float matrix.
///
/// Although you can instandiate a Motivator, you probably will not, since
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

  /// Allow Motivators to be copied.
  /// Transfer ownership of motivator to new motivator. Old motivator is reset
  /// and must be initialized again before being read.
  /// We want to allow copies primarily so that we can have vectors of
  /// Motivators.
  Motivator(const Motivator& original) {
    if (original.Valid()) {
      original.processor_->TransferMotivator(original.index_, this);
    } else {
      processor_ = nullptr;
      index_ = kMotiveIndexInvalid;
    }
  }

  /// Allow Motivators to be copied. `original` is reset.
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
  /// Also check for a consistent internal state.
  bool Valid() const {
    return processor_ != nullptr && processor_->ValidMotivator(index_, this);
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

  /// Initialize this Motivator to the type specified in init.type.
  /// @param init Defines the type and initial state of the Motivator.
  /// @param engine The engine that will update this Motivator when
  ///               engine->AdvanceFrame() is called.
  /// @param dimensions The number of slots to occupy. For example, a 3D
  ///                   position would occupy three slots.
  void InitializeWithDimension(const MotivatorInit& init, MotiveEngine* engine,
                               MotiveDimension dimensions);

  /// The MotiveProcessor uses the functions below. It does not modify data
  /// directly.
  friend MotiveProcessor;

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

/// @class Motivator1f
/// @brief Drive a single float value towards a target, or along a spline.
///
/// The current and target values and velocities can be specified by SetTarget()
/// or SetSpline().
///
template <class VectorConverter, int kDimensionsT>
class MotivatorVectorTemplate : public Motivator {
 public:
  static const int kDimensions = kDimensionsT;
  typedef VectorConverter C;
  typedef typename fpl::ExternalVectorT<C, kDimensions>::type ExT;
  typedef typename fpl::InternalVectorT<kDimensions>::type InT;
  typedef typename MotiveTargetT<kDimensions>::type Target;
  typedef fpl::SplinePlaybackN<kDimensions> Spline;
  typedef MotiveTargetBuilderTemplate<C, kDimensions> TargetBuilder;

  /// Motivator is created in a reset state. When in the reset state,
  /// it is not being driven, and Value(), Velocity(), etc. cannot be called.
  MotivatorVectorTemplate() {}

  /// Initialize to the type specified by `init`. Current and target values
  /// are not set.
  MotivatorVectorTemplate(const MotivatorInit& init, MotiveEngine* engine)
      : Motivator(init, engine, kDimensions) {}

  /// Initialize to the type specified by `init`. Set current and target values
  /// as specified by `t`.
  MotivatorVectorTemplate(const MotivatorInit& init, MotiveEngine* engine,
                          const Target& t)
      : Motivator(init, engine, kDimensions) {
    SetTarget(t);
  }

  /// Initialize to the type specified by `init`.
  void Initialize(const MotivatorInit& init, MotiveEngine* engine) {
    InitializeWithDimension(init, engine, kDimensions);
  }

  /// Initialize to the type specified by `init`. Set current and target values
  /// as specified by `t`.
  void InitializeWithTarget(const MotivatorInit& init, MotiveEngine* engine,
                            const Target& t) {
    Initialize(init, engine);
    SetTarget(t);
  }

  /// Returns the current motivator value. The current value is updated when
  /// engine->AdvanceFrame() is called on the `engine` that initialized this
  /// Motivator.
  /// Note that the "InT()" parameter is just a syntactic hack used to access
  /// the correct overloaded function in the processor.
  ExT Value() const { return C::To(Processor().ValueT(index_, InT())); }

  /// Returns the current rate of change of this motivator. For example,
  /// if this Motivator is being driven by a spline, returns the derivative
  /// at the current time in the spline curve.
  ExT Velocity() const { return C::To(Processor().VelocityT(index_, InT())); }

  /// Returns the value this Motivator is driving towards.
  /// If being driven by a spline, returns the value at the end of the spline.
  ExT TargetValue() const {
    return C::To(Processor().TargetValueT(index_, InT()));
  }

  /// Returns the rate-of-change of this Motivator once it reaches
  /// TargetValue().
  ExT TargetVelocity() const {
    return C::To(Processor().TargetVelocityT(index_, InT()));
  }

  /// Returns TargetValue() minus Value(). If we're driving a
  /// modular type (e.g. an angle), this may not be the naive subtraction.
  /// For example, if TargetValue() = 170 degrees, Value() = -170 degrees,
  /// then Difference() = -20 degrees.
  ExT Difference() const {
    return C::To(Processor().DifferenceT(index_, InT()));
  }

  /// Returns time remaining until target is reached.
  /// The unit of time is determined by the calling program.
  MotiveTime TargetTime() const { return Processor().TargetTime(index_); }

  /// Returns the current time (i.e. the x-value) in the current spline.
  /// If Motivator is not being driven by a spline, returns 0.
  /// Whenever SetSpline() is called, this value will be reset to the
  /// start_time specified in SplinePlayback. Every time
  /// MotiveEngine::AdvanceFrame() is called, it will increment by
  /// `delta_time` * `playback_rate`. If the SplinePlayback has repeat=true,
  /// then SplineTime() will periodically loop back to time 0.
  MotiveTime SplineTime() const { return Processor().SplineTime(index_); }

  /// Set the target and (optionally the current) motivator values.
  /// Use this call to procedurally drive the Motivator towards a specific
  /// target. The Motivator will transition smoothly to the new target.
  /// You can change the target value every frame if you like, and the
  /// Motivator value should behave calmly but responsively, with the
  /// movement qualities of the underlying MotiveProcessor.
  /// Note that the underlying MotiveProcessor is allowed to ignore
  /// parts of `t` that are irrelevent to its algorithm.
  /// @param t A set of waypoints to hit, optionally including the current
  ///          value. If the current value is not included, maintain the
  ///          existing current value.
  void SetTarget(const Target& t) { Processor().SetTarget(index_, t); }

  /// Follow the curve specified in `s`. Overrides the existing current value.
  /// @param s The spline to follow, the time in that spline to initiate
  ///          playback, and whether to repeat from the beginning after the
  ///          end of the spline is reached.
  void SetSpline(const Spline& s) { Processor().SetSpline(index_, s); }

  /// Set rate at which we consume the spline set in SetSpline().
  ///     0   ==> paused
  ///     0.5 ==> half speed (slow motion)
  ///     1   ==> authored speed
  ///     2   ==> double speed (fast forward)
  void SetSplinePlaybackRate(float playback_rate) {
    Processor().SetSplinePlaybackRate(index_, playback_rate);
  }

 private:
  MotiveProcessorVector& Processor() {
    return *static_cast<MotiveProcessorVector*>(processor_);
  }
  const MotiveProcessorVector& Processor() const {
    return *static_cast<const MotiveProcessorVector*>(processor_);
  }
};

/// @class MotivatorMatrix4fTemplate
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
class MotivatorMatrix4fTemplate : public Motivator {
  typedef VectorConverter C;
  typedef typename VectorConverter::ExternalMatrix4 Mat4;
  typedef typename VectorConverter::ExternalVector3 Vec3;

 public:
  MotivatorMatrix4fTemplate() {}
  MotivatorMatrix4fTemplate(const MotivatorInit& init, MotiveEngine* engine)
      : Motivator(init, engine, 1) {}

  /// Initialize to the type specified by `init`.
  void Initialize(const MotivatorInit& init, MotiveEngine* engine) {
    InitializeWithDimension(init, engine, 1);
  }

  /// Return the current value of the Motivator. The processor returns a
  /// vector-aligned matrix, so the cast should be valid for any user-defined
  /// matrix type.
  const Mat4& Value() const { return C::To(Processor().Value(index_)); }

  /// Return the translation component of the matrix.
  /// The matrix is a 3D affine transform, so the translation component is the
  /// fourth column.
  Vec3 Position() const {
    return C::To(Processor().Value(index_).TranslationVector3D());
  }

  /// Return the current value of the `child_index`th basic transform that
  /// drives this matrix.
  /// @param child_index The index into MatrixInit::ops(). The ops() array
  ///                    is a series of basic transformation operations that
  ///                    compose this matrix. Each basic transformation has
  ///                    a current value. We gather this value here.
  float ChildValue1f(MotiveChildIndex child_index) const {
    return Processor().ChildValue1f(index_, child_index);
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
    return C::To(Processor().ChildValue3f(index_, child_index));
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
    Processor().SetChildValue1f(index_, child_index, value);
  }

  /// Set the constant values of the basic transforms at indices
  /// (child_index, child_index + 1, child_index + 2).
  /// @param child_index The first index into MatrixInit::ops(). The constant
  ///                    value at this index is set to the x component of
  ///                    `value`.
  ///                    child_index + 1's constant is set to value.y, and
  ///                    child_index + 2's constant is set to value.z.
  void SetChildValue3f(MotiveChildIndex child_index, const Vec3& value) {
    Processor().SetChildValue3f(index_, child_index, C::From(value));
  }

 private:
  MotiveProcessorMatrix4f& Processor() {
    return *static_cast<MotiveProcessorMatrix4f*>(processor_);
  }
  const MotiveProcessorMatrix4f& Processor() const {
    return *static_cast<const MotiveProcessorMatrix4f*>(processor_);
  }
};

// These Motivator types use mathfu in their external API.
typedef MotivatorVectorTemplate<fpl::PassThroughVectorConverter, 1> Motivator1f;
typedef MotivatorVectorTemplate<fpl::PassThroughVectorConverter, 2> Motivator2f;
typedef MotivatorVectorTemplate<fpl::PassThroughVectorConverter, 3> Motivator3f;
typedef MotivatorVectorTemplate<fpl::PassThroughVectorConverter, 4> Motivator4f;
typedef MotivatorMatrix4fTemplate<fpl::PassThroughVectorConverter>
    MotivatorMatrix4f;

}  // namespace motive

#endif  // MOTIVE_MOTIVATOR_H
