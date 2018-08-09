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

#ifndef MOTIVE_PROCESSOR_H_
#define MOTIVE_PROCESSOR_H_

#include <vector>

#include "motive/common.h"
#include "motive/math/compact_spline.h"
#include "motive/math/vector_converter.h"
#include "motive/target.h"
#include "motive/util/index_allocator.h"

namespace motive {

class Motivator;
class MotiveEngine;

/// @class MotiveProcessor
/// @brief A MotiveProcessor processes *all* instances of one type of Motivator.
///
/// Each derivation of MotiveProcessor is one animation algorithm. It holds
/// all the data for all Motivators that are currently using that animation
/// algorithm.
///
/// We pool the processing for potential optimization opportunities. We may have
/// hundreds of smoothly-interpolating one-dimensional Motivators, for example.
/// It's nice to be able to update those 4 or 8 or 16 at a time using SIMD.
/// And it's nice to have the data gathered in one spot if we want to use
/// multiple threads.
///
/// MotiveProcessors exists in the internal API. For the external API, please
/// see Motivator.
///
/// Users can create their own Motivator algorithms by deriving from
/// MotiveProcessor. MotiveProcessors must have a factory that's registered with
/// the MotiveEngine (please see MotiveEngine for details). Once registered,
/// you can use your new Motivator algorithm by calling Motivator::Initialize()
/// with Init::type set to your MotiveProcessor's MotivatorType.
///
/// MotiveProcessors run on mathfu types. Please see the specializations below
/// for MotiveProcessors of various dimensions.
///
class MotiveProcessor {
 public:
  MotiveProcessor()
      : index_allocator_(allocator_callbacks_),
        engine_(nullptr),
        benchmark_id_for_advance_frame_(-1),
        benchmark_id_for_init_(-1) {
    allocator_callbacks_.set_processor(this);
  }
  virtual ~MotiveProcessor();

  /// Instantiate motivator data inside the MotiveProcessor, and initialize
  /// `motivator` as a reference to that data.
  ///
  /// This function should only be called by Motivator::Initialize().
  ///
  /// @param init The initialization parameters for the Motivator. Each
  ///             MotiveProcessor has its own derivation of MotivatorInit,
  ///             and InitializeMotivator will only ever be called with that
  ///             derivation.
  /// @param engine The owner of all the MotiveProcessors. An engine holds at
  ///               most one of any type of MotiveProcessor. The engine can be
  ///               used to create child Motivators that drive `motivator`.
  /// @param motivator The Motivator that is initialized to reference into
  ///                  This MotiveProcessor. The MotiveProcessor also keeps
  ///                  a reference to `motivator` in case it shuffles around
  ///                  internal data.
  /// @param dimensions The number of slots to consume in the MotiveProcessor.
  ///                   For example, a 3D vector would consume three slots in
  ///                   a MotiveProcessor of floats.
  void InitializeMotivator(const MotivatorInit& init, MotiveEngine* engine,
                           Motivator* motivator, MotiveDimension dimensions);

  /// Remove an motivator and return its index to the pile of allocatable
  /// indices.
  ///
  /// This function should only be called by Motivator::Invalidate().
  ///
  /// @param index Reference into the MotiveProcessor's internal arrays.
  void RemoveMotivator(MotiveIndex index);

  /// Transfer ownership of the motivator at `index` to 'new_motivator'.
  /// Resets the Motivator that currently owns `index` and initializes
  /// 'new_motivator'.
  ///
  /// This function should only be called by Motivator's copy operations.
  ///
  /// @param index Reference into the MotiveProcessor's internal arrays.
  /// @param new_motivator The Motivator that is initialized to reference
  ///                      `index`.
  void TransferMotivator(MotiveIndex index, Motivator* new_motivator);

  /// Returns true if `index` is currently driving a motivator. Does not do
  /// any validity checking, however, like ValidMotivatorIndex() does.
  /// @param index Reference into the MotiveProcessor's internal arrays.
  bool IsMotivatorIndex(MotiveIndex index) const;

  /// Returns true if `index` is currently in a block of indices driven by
  /// a motivator.
  /// @param index Reference into the MotiveProcessor's internal arrays.
  bool ValidIndex(MotiveIndex index) const;

  /// Returns true if a Motivator is referencing this index.
  /// That is, if this index is part of a block of indices
  /// (for example a block of 3 indices referenced by a Motivator3f),
  /// then this index is the *first* index in that block.
  bool ValidMotivatorIndex(MotiveIndex index) const;

  /// Returns true if `index` is currently driving `motivator`.
  /// @param index Reference into the MotiveProcessor's internal arrays.
  /// @param motivator Motivator to verify points to `index`.
  bool ValidMotivator(MotiveIndex index, const Motivator* motivator) const {
    return ValidIndex(index) && motivators_[index] == motivator;
  }

  /// Advance the simulation by `delta_time`.
  ///
  /// This function should only be called by MotiveEngine::AdvanceFrame.
  ///
  /// @param delta_time Time since the last call to AdvanceFrame(). Time units
  ///                   are determined by the user.
  virtual void AdvanceFrame(MotiveTime delta_time) = 0;

  /// Should return kType of the MotivatorInit class for the derived processor.
  /// kType is defined by the macro MOTIVE_INTERFACE, which is put in
  /// a processor's MotivatorInit derivation.
  virtual MotivatorType Type() const = 0;

  /// The lower the number, the sooner the MotiveProcessor gets updated.
  /// Should never change. We want a static ordering of processors.
  /// Some MotiveProcessors use the output of other MotiveProcessors, so
  /// we impose a strict ordering here.
  virtual int Priority() const = 0;

  /// The number of slots occupied in the MotiveProcessor. For example,
  /// a position in 3D space would return 3. A single 4x4 matrix would return 1.
  MotiveDimension Dimensions(MotiveIndex index) const {
    return index_allocator_.CountForIndex(index);
  }

  /// Ensure that the internal state is consistent. Call periodically when
  /// debugging problems where the internal state is corrupt.
  void VerifyInternalState() const;

  // For internal use. Called by the MotiveEngine to profile each processor.
  void RegisterBenchmarks();
  int benchmark_id_for_advance_frame() const {
    return benchmark_id_for_advance_frame_;
  }
  int benchmark_id_for_init() const { return benchmark_id_for_init_; }

  /// Sets the MotiveEngine instance that owns this processor. This function
  /// is called by the MotiveEngine at creation time. This function has no
  /// effect if it has been called before on this processor.
  void SetEngine(MotiveEngine* engine);

 protected:
  /// Initialize data at [index, index + dimensions).
  /// The meaning of `index` is determined by the MotiveProcessor
  /// implementation (most likely it is the index into one or more data_ arrays
  /// though). MotiveProcessor tries to keep the `index` as low as possible, by
  /// recycling ones that have been freed, and by providing a Defragment()
  /// function to move later indices to indices that have been freed.
  virtual void InitializeIndices(const MotivatorInit& init, MotiveIndex index,
                                 MotiveDimension dimensions,
                                 MotiveEngine* engine) = 0;

  /// Reset data at [index, index + dimensions).
  /// See comment above InitializeIndex for meaning of `index`.
  /// If your MotiveProcessor stores data in a plain array, you probably have
  /// nothing to do. But if you use dynamic memory per index,
  /// (which you really shouldn't - too slow!), you should deallocate it here.
  /// For debugging, it might be nice to invalidate the data.
  virtual void RemoveIndices(MotiveIndex index, MotiveDimension dimensions) = 0;

  /// Move the data chunk of length `dimensions` from `old_index` into
  /// `new_index`. Used by Defragment().
  /// Note that the index range starting at `new_index` is guaranteed to be
  /// inactive.
  virtual void MoveIndices(MotiveIndex old_index, MotiveIndex new_index,
                           MotiveDimension dimensions) = 0;

  /// Increase or decrease the total number of indices.
  /// If decreased, existing indices >= num_indices should be uninitialized.
  /// If increased, internal arrays should be extended to new_indices, and
  /// new items in the arrays should be initialized as reset.
  virtual void SetNumIndices(MotiveIndex num_indices) = 0;

  /// When an index is moved, the Motivator that references that index is
  /// updated. Can be called at the discretion of your MotiveProcessor,
  /// but normally called at the beginning of your
  /// MotiveProcessor::AdvanceFrame.
  void Defragment() { index_allocator_.Defragment(); }

  /// Return a handle to the MotiveEngine instance that owns this processor.
  MotiveEngine* Engine() { return engine_; }
  const MotiveEngine* Engine() const { return engine_; }

 private:
  typedef IndexAllocator<MotiveIndex> MotiveIndexAllocator;
  typedef MotiveIndexAllocator::IndexRange IndexRange;

  /// Don't notify derived class.
  void RemoveMotivatorWithoutNotifying(MotiveIndex index);

  /// Handle callbacks from IndexAllocator.
  void MoveIndexRangeBase(const IndexRange& source, MotiveIndex target);
  void SetNumIndicesBase(MotiveIndex num_indices);

  /// Proxy callbacks from IndexAllocator into MotiveProcessor.
  class AllocatorCallbacks : public MotiveIndexAllocator::CallbackInterface {
   public:
    AllocatorCallbacks() : processor_(nullptr) {}
    void set_processor(MotiveProcessor* processor) { processor_ = processor; }
    virtual void SetNumIndices(MotiveIndex num_indices) {
      processor_->SetNumIndicesBase(num_indices);
    }
    virtual void MoveIndexRange(const IndexRange& source, MotiveIndex target) {
      processor_->MoveIndexRangeBase(source, target);
    }

   private:
    MotiveProcessor* processor_;
  };

  /// Back-pointer to the Motivators for each index. The Motivators reference
  /// this MotiveProcessor and a specific index into the MotiveProcessor,
  /// so when the index is moved, or when the MotiveProcessor itself is
  /// destroyed, we need to update the Motivator.
  /// Note that we only keep a reference to a single Motivator per index.
  /// When a copy of an Motivator is made, the old Motivator is Reset and the
  /// reference here is updated.
  std::vector<Motivator*> motivators_;

  /// Proxy calbacks into MotiveProcessor. The other option is to derive
  /// MotiveProcessor from IndexAllocator::CallbackInterface, but that would
  /// create a messier API, and not be great OOP.
  /// This member should be initialized before index_allocator_ is initialized.
  AllocatorCallbacks allocator_callbacks_;

  /// When an index is freed, we keep track of it here. When an index is
  /// allocated, we use one off this array, if one exists.
  /// When Defragment() is called, we empty this array by filling all the
  /// unused indices with the highest allocated indices. This reduces the total
  /// size of the data arrays.
  MotiveIndexAllocator index_allocator_;

  /// A handle to the owning MotiveEngine. This is required when new Motivators
  /// are created outside of typical initialization times.
  MotiveEngine* engine_;

  int benchmark_id_for_advance_frame_;
  int benchmark_id_for_init_;
};

/// Static functions in MotiveProcessor-derived classes.
typedef MotiveProcessor* MotiveProcessorCreateFn();
typedef void MotiveProcessorDestroyFn(MotiveProcessor* p);

struct MotiveProcessorFunctions {
  MotiveProcessorCreateFn* create;
  MotiveProcessorDestroyFn* destroy;

  MotiveProcessorFunctions(MotiveProcessorCreateFn* create,
                           MotiveProcessorDestroyFn* destroy)
      : create(create), destroy(destroy) {}
};

}  // namespace motive

#endif  // MOTIVE_PROCESSOR_H_
