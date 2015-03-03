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

#include "fplutil/index_allocator.h"
#include "motive/common.h"

namespace fpl {
class CompactSpline;
}

namespace motive {

class Motivator;
class MotiveEngine;
class MotiveTarget1f;

// MotiveProcessor
// ==============
// An MotiveProcessor processes *all* instances of one type of Motivator.
// Or, at least, all instances within a given MotiveEngine.
//
// We pool the processing for potential optimization opportunities. We may have
// hundreds of smoothly-interpolating one-dimensional Motivators, for example.
// It's nice to be able to update those 4 or 8 or 16 at a time using SIMD.
// And it's nice to have the data gathered in one spot if we want to use
// multiple threads.
//
// MotiveProcessors exists in the internal API. For the external API, please see
// Motivator.
//
// Users can create their own Motivator algorithms by deriving from
// MotiveProcessor. MotiveProcessors must have a factory that's registered with
// the MotiveEngine (please see MotiveEngine for details). Once registered,
// you can use your new Motivator algorithm by calling Motivator::Initialize()
// with Init::type set to your MotiveProcessor's MotivatorType.
//
// MotiveProcessors run on mathfu types. Please see the specializations below
// for MotiveProcessors of various dimensions. Note that Motivator expects the
// MotiveProcessor to return a mathfu type, so you shouldn't try to specialize
// MotiveProcessor<> with any of your native types.
//
class MotiveProcessor {
 public:
  MotiveProcessor()
      : allocator_callbacks_(this), index_allocator_(allocator_callbacks_) {}
  virtual ~MotiveProcessor();

  // Instantiate motivator data inside the MotiveProcessor, and initialize
  // 'motivator' as a reference to that data.
  // The 'engine' is required if the MotiveProcessor itself creates child
  // Motivators. This function should only be called by Motivator::Initialize().
  void InitializeMotivator(const MotivatorInit& init, MotiveEngine* engine,
                           Motivator* motivator);

  // Remove an motivator and return its index to the pile of allocatable
  // indices.
  // Should only be called by Motivator::Invalidate().
  void RemoveMotivator(MotiveIndex index);

  // Transfer ownership of the motivator at 'index' to 'new_motivator'.
  // Resets the Motivator that currently owns 'index' and initializes
  // 'new_motivator'.
  // Should only be called by Motivator copy operations.
  void TransferMotivator(MotiveIndex index, Motivator* new_motivator);

  // Returns true if 'index' is currently driving an motivator.
  bool ValidIndex(MotiveIndex index) const;

  // Returns true if 'index' is currently driving 'motivator'.
  bool ValidMotivator(MotiveIndex index, const Motivator* motivator) const {
    return ValidIndex(index) && motivators_[index] == motivator;
  }

  // Advance the simulation by delta_time.
  // Should only be called by MotiveEngine::AdvanceFrame.
  virtual void AdvanceFrame(MotiveTime delta_time) = 0;

  // Return GUID representing the Motivator's type. Must be implemented by
  // derived class.
  virtual MotivatorType Type() const = 0;

  // The number of floats (or doubles) being animated. For example, a position
  // in 3D space would return 3.
  virtual int Dimensions() const = 0;

  // The lower the number, the sooner the MotiveProcessor gets updated.
  // Should never change. We want a static ordering of processors.
  // Some MotiveProcessors use the output of other MotiveProcessors, so
  // we impose a strict ordering here.
  virtual int Priority() const = 0;

 protected:
  // Initialize data at 'index'. The meaning of 'index' is determined by the
  // MotiveProcessor implementation (most likely it is the index into one or
  // more data_ arrays though).
  // MotiveProcessor tries to keep the 'index' as low as possible, by
  // recycling ones that have been freed, and by providing a Defragment()
  // function to move later indices to indices that have been freed.
  virtual void InitializeIndex(const MotivatorInit& init, MotiveIndex index,
                               MotiveEngine* engine) = 0;

  // Reset data at 'index'. See comment above InitializeIndex for meaning of
  // 'index'. If your MotiveProcessor stores data in a plain array, you
  // probably have nothing to do. But if you use dynamic memory per index,
  // (which you really shouldn't - too slow!), you should deallocate it here.
  // For debugging, it might be nice to invalidate the data.
  virtual void RemoveIndex(MotiveIndex index) = 0;

  // Move the data at 'old_index' into 'new_index'. Used by Defragment().
  // Note that 'new_index' is guaranteed to be inactive.
  virtual void MoveIndex(MotiveIndex old_index, MotiveIndex new_index) = 0;

  virtual void SetNumIndices(MotiveIndex num_indices) = 0;

  void MoveIndexBase(MotiveIndex old_index, MotiveIndex new_index);
  void SetNumIndicesBase(MotiveIndex num_indices);

  // When an index is moved, the Motivator that references that index is
  // updated.
  // Can be called at the discretion of your MotiveProcessor, but normally
  // called
  // at the beginning of your MotiveProcessor::AdvanceFrame.
  void Defragment() { index_allocator_.Defragment(); }

 private:
  // Proxy callbacks from IndexAllocator into MotiveProcessor.
  class AllocatorCallbacks
      : public fpl::IndexAllocator<MotiveIndex>::CallbackInterface {
   public:
    AllocatorCallbacks(MotiveProcessor* processor) : processor_(processor) {}
    virtual void SetNumIndices(MotiveIndex num_indices) {
      processor_->SetNumIndicesBase(num_indices);
    }
    virtual void MoveIndex(MotiveIndex old_index, MotiveIndex new_index) {
      processor_->MoveIndexBase(old_index, new_index);
    }

   private:
    MotiveProcessor* processor_;
  };

  // Back-pointer to the Motivators for each index. The Motivators reference
  // this
  // MotiveProcessor and a specific index into the MotiveProcessor, so when the
  // index is moved, or when the MotiveProcessor itself is destroyed, we need
  // to update the Motivator.
  // Note that we only keep a reference to a single Motivator per index. When
  // a copy of an Motivator is made, the old Motivator is Reset and the
  // reference
  // here is updated.
  std::vector<Motivator*> motivators_;

  // Proxy calbacks into MotiveProcessor. The other option is to derive
  // MotiveProcessor from IndexAllocator::CallbackInterface, but that would
  // create a messier API, and not be great OOP.
  // This member should be initialized before index_allocator_ is initialized.
  AllocatorCallbacks allocator_callbacks_;

  // When an index is freed, we keep track of it here. When an index is
  // allocated, we use one off this array, if one exists.
  // When Defragment() is called, we empty this array by filling all the
  // unused indices with the highest allocated indices. This reduces the total
  // size of the data arrays.
  fpl::IndexAllocator<MotiveIndex> index_allocator_;
};

// Interface for motivator types that drive a single float value.
// That is, for MotiveProcessors that interface with Motivator1f's.
class MotiveProcessor1f : public MotiveProcessor {
 public:
  virtual int Dimensions() const { return 1; }

  // Get current motivator values from the processor.
  virtual float Value(MotiveIndex index) const = 0;
  virtual float Velocity(MotiveIndex index) const = 0;
  virtual float TargetValue(MotiveIndex index) const = 0;
  virtual float TargetVelocity(MotiveIndex index) const = 0;
  virtual float Difference(MotiveIndex index) const = 0;
  virtual MotiveTime TargetTime(MotiveIndex index) const = 0;

  // At least one of these should be implemented. Otherwise, there will be
  // no way to drive the Motivator towards a target.
  virtual void SetTarget(MotiveIndex /*index*/, const MotiveTarget1f& /*t*/) {}
  virtual void SetWaypoints(MotiveIndex /*index*/,
                            const fpl::CompactSpline& /*waypoints*/,
                            float /*start_time*/) {}
};

// Interface for motivator types that drive a 4x4 float matrix.
// That is, for MotiveProcessors that interface with MotivatorMatrix4f's.
class MotiveProcessorMatrix4f : public MotiveProcessor {
 public:
  virtual int Dimensions() const { return 16; }

  // Get the current matrix value from the processor.
  virtual const mathfu::mat4& Value(MotiveIndex index) const = 0;

  // Get current values of the components that create the matrix.
  virtual float ChildValue1f(MotiveIndex index,
                             MotiveChildIndex child_index) const = 0;
  virtual mathfu::vec3 ChildValue3f(MotiveIndex index,
                                    MotiveChildIndex child_index) const {
    return mathfu::vec3(ChildValue1f(index, child_index),
                        ChildValue1f(index, child_index + 1),
                        ChildValue1f(index, child_index + 2));
  }

  // Set child values. Matrices are composed from child components.
  virtual void SetChildTarget1f(MotiveIndex /*index*/,
                                MotiveChildIndex /*child_index*/,
                                const MotiveTarget1f& /*t*/) {}
  virtual void SetChildValue1f(MotiveIndex /*index*/,
                               MotiveChildIndex /*child_index*/,
                               float /*value*/) {}
  virtual void SetChildValue3f(MotiveIndex index, MotiveChildIndex child_index,
                               const mathfu::vec3& value) {
    for (int i = 0; i < 3; ++i) {
      SetChildValue1f(index, child_index + i, value[i]);
    }
  }
};

// Static functions in MotiveProcessor-derived classes.
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
