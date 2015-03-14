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

// Motivator
// ========
// An Motivator drives a value towards a target.
//
// The value can be one-dimensional (e.g. a float), or multi-dimensional
// (e.g. a matrix). The dimension is determined by the sub-class:
// Motivator1f drives a float, MotivatorMatrix4f drives a 4x4 float matrix.
// The Motivator's current value can be queried with Value().
//
// The way an Motivator's value moves towards its target is determined by the
// **type** of an motivator. The type is specified in Motivator::Initialize().
//
// Note that an Motivator does not store any data itself. It is a handle into
// an MotiveProcessor. Each MotiveProcessor holds all data for motivators
// of a given **type**. Only one Motivator can hold a handle to specific data.
// Therefore, you can copy an Motivator, but the original motivator will become
// invalid.
//
class Motivator {
 public:
  Motivator() : processor_(nullptr), index_(kMotiveIndexInvalid) {}
  Motivator(const MotivatorInit& init, MotiveEngine* engine)
      : processor_(nullptr), index_(kMotiveIndexInvalid) {
    Initialize(init, engine);
  }

  // Allow Motivators to be copied so that they can be put into vectors.
  // Transfer ownership of motivator to new motivator. Old motivator is reset
  // and
  // can no longer be used.
  Motivator(const Motivator& original) {
    if (original.Valid()) {
      original.processor_->TransferMotivator(original.index_, this);
    } else {
      processor_ = nullptr;
      index_ = kMotiveIndexInvalid;
    }
  }
  Motivator& operator=(const Motivator& original) {
    Invalidate();
    original.processor_->TransferMotivator(original.index_, this);
    return *this;
  }

  // Remove ourselves from the MotiveProcessor when we're deleted.
  ~Motivator() { Invalidate(); }

  // Initialize this Motivator to the type specified in init.type.
  void Initialize(const MotivatorInit& init, MotiveEngine* engine);

  // Detatch this Motivator from its MotiveProcessor. Functions other than
  // Initialize can no longer be called after Invalidate has been called.
  void Invalidate() {
    if (processor_ != nullptr) {
      processor_->RemoveMotivator(index_);
    }
  }

  // Return true if this Motivator is currently being driven by an
  // MotiveProcessor. That is, it has been successfully initialized.
  // Also check for a consistent internal state.
  bool Valid() const {
    return processor_ != nullptr && processor_->ValidMotivator(index_, this);
  }

  // Return the type of Motivator we've been initilized to.
  // An Motivator can take on any type.
  MotivatorType Type() const { return processor_->Type(); }

  // The number of floats (or doubles) that this Motivator is driving.
  // For example, if this Motivator is driving a position in 3D space, then
  // we will return 3 here.
  int Dimensions() const { return processor_->Dimensions(); }

 protected:
  // The MotiveProcessor uses the functions below. It does not modify data
  // directly.
  friend MotiveProcessor;

  // These should only be called by MotiveProcessor!
  void Init(MotiveProcessor* processor, MotiveIndex index) {
    processor_ = processor;
    index_ = index;
  }
  void Reset() { Init(nullptr, kMotiveIndexInvalid); }
  const MotiveProcessor* Processor() const { return processor_; }

  // All calls to an Motivator are proxied to an MotivatorProcessor. Motivator
  // data and processing is centralized to allow for scalable optimizations
  // (e.g. SIMD or parallelization).
  MotiveProcessor* processor_;

  // An MotiveProcessor processes one MotivatorType, and hosts every Motivator
  // of
  // that type. The id here uniquely identifies this Motivator to the
  // MotiveProcessor.
  MotiveIndex index_;
};

// Drive a float value towards a target.
//
// The current and target values and velocities can be specified by SetTarget()
// or SetSpline().
class Motivator1f : public Motivator {
 public:
  Motivator1f() {}
  Motivator1f(const MotivatorInit& init, MotiveEngine* engine)
      : Motivator(init, engine) {}
  Motivator1f(const MotivatorInit& init, MotiveEngine* engine,
              const MotiveTarget1f& t)
      : Motivator(init, engine) {
    SetTarget(t);
  }
  void InitializeWithTarget(const MotivatorInit& init, MotiveEngine* engine,
                            const MotiveTarget1f& t) {
    Initialize(init, engine);
    SetTarget(t);
  }

  // Return current motivator values.
  float Value() const { return Processor().Value(index_); }
  float Velocity() const { return Processor().Velocity(index_); }
  float TargetValue() const { return Processor().TargetValue(index_); }
  float TargetVelocity() const { return Processor().TargetVelocity(index_); }

  // Returns TargetValue() minus Value(). If we're driving a
  // modular type (e.g. an angle), this may not be the naive subtraction.
  float Difference() const { return Processor().Difference(index_); }

  // Returns time remaining until target is reached.
  MotiveTime TargetTime() const { return Processor().TargetTime(index_); }

  // Set current motivator values in the processor. Processors may choose to
  // ignore whichever values make sense for them to ignore.
  void SetTarget(const MotiveTarget1f& t) { Processor().SetTarget(index_, t); }
  void SetSpline(const fpl::SplinePlayback& s) {
    Processor().SetSpline(index_, s);
  }

 private:
  MotiveProcessor1f& Processor() {
    return *static_cast<MotiveProcessor1f*>(processor_);
  }
  const MotiveProcessor1f& Processor() const {
    return *static_cast<const MotiveProcessor1f*>(processor_);
  }
};

// Drive a 4x4 float matrix from a series of basic transformations.
//
// The underlying basic transformations can be altered with SetChildTarget1f()
// and SetChildValue1f().
//
// Internally, we use mathfu::mat4 as our matrix type, but external we allow
// any matrix type to be specified via the Matrix4f template parameter.
//
template <class VectorConverter>
class MotivatorMatrix4fTemplate : public Motivator {
  typedef VectorConverter C;
  typedef typename VectorConverter::ExternalMatrix4 Mat4;
  typedef typename VectorConverter::ExternalVector3 Vec3;

 public:
  MotivatorMatrix4fTemplate() {}
  MotivatorMatrix4fTemplate(const MotivatorInit& init, MotiveEngine* engine)
      : Motivator(init, engine) {}

  // Return the current value of the Motivator. The processor returns a
  // vector-aligned matrix, so the cast should be valid for any user-defined
  // matrix type.
  const Mat4& Value() const { return C::To(Processor().Value(index_)); }
  Vec3 Position() const {
    return C::To(Processor().Value(index_).TranslationVector3D());
  }

  float ChildValue1f(MotiveChildIndex child_index) const {
    return Processor().ChildValue1f(index_, child_index);
  }
  Vec3 ChildValue3f(MotiveChildIndex child_index) const {
    return C::To(Processor().ChildValue3f(index_, child_index));
  }

  // Set the target for a child motivator. Each basic matrix transformations
  // can be driven by a child motivator. This call lets us control each
  // transformation.
  void SetChildTarget1f(MotiveChildIndex child_index, const MotiveTarget1f& t) {
    Processor().SetChildTarget1f(index_, child_index, t);
  }

  // Set the constant value of a child. Each basic matrix transformation
  // can be driven by a constant value. This call lets us set those constant
  // values.
  void SetChildValue1f(MotiveChildIndex child_index, float value) {
    Processor().SetChildValue1f(index_, child_index, value);
  }
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

// External types are also mathfu in this converter. Create your own converter
// if you'd like to use your own vector types in MotivatorMatrix's external API.
class PassThroughVectorConverter {
 public:
  typedef mathfu::mat4 ExternalMatrix4;
  typedef mathfu::vec3 ExternalVector3;
  static const ExternalMatrix4& To(const mathfu::mat4& m) { return m; }
  static ExternalVector3 To(const mathfu::vec3& v) { return v; }
  static const mathfu::vec3& From(const ExternalVector3& v) { return v; }
};

typedef MotivatorMatrix4fTemplate<PassThroughVectorConverter> MotivatorMatrix4f;

}  // namespace motive

#endif  // MOTIVE_MOTIVATOR_H
