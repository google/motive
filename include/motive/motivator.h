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

#ifndef MOTIVE_MOTIVATOR_H_
#define MOTIVE_MOTIVATOR_H_

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
///
class Motivator {
 public:
  Motivator() : processor_(nullptr), index_(kMotiveIndexInvalid) {}

  /// Transfer ownership of `original` motivator to `this` motivator.
  /// `original` motivator is reset and must be initialized again before being
  /// read. We want to allow moves primarily so that we can have vectors of
  /// Motivators.
  Motivator(Motivator&& original) noexcept {
    if (original.Valid()) {
      original.processor_->TransferMotivator(original.index_, this);
    } else {
      processor_ = nullptr;
      index_ = kMotiveIndexInvalid;
    }
  }

  /// Allow Motivators to be moved. `original` is reset.
  /// See the move constructor for details.
  Motivator& operator=(Motivator&& original) noexcept {
    Invalidate();
    if (original.processor_ != nullptr) {
      original.processor_->TransferMotivator(original.index_, this);
    }
    return *this;
  }

  /// Disallow copying Motivators since only one Motivator can reference a
  /// specific index in a MotiveProcessor.
  Motivator(const Motivator& original) = delete;
  Motivator& operator=(const Motivator& original) = delete;

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
  MotivatorType Type() const {
    return Valid() ? processor_->Type() : kMotivatorTypeInvalid;
  }

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

}  // namespace motive

#endif  // MOTIVE_MOTIVATOR_H_
