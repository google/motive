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

#include "motive/processor.h"
#include "motive/motivator.h"
#include "motive/util/benchmark.h"

namespace motive {

MotiveProcessor::~MotiveProcessor() {
  // Reset all of the Motivators that we're currently driving.
  // We don't want any of them to reference us after we've been destroyed.
  for (MotiveIndex index = 0; index < index_allocator_.num_indices();
       index += Dimensions(index)) {
    if (motivators_[index] != nullptr) {
      RemoveMotivatorWithoutNotifying(index);
    }
  }

  // Sanity-check: Ensure that we have no more active Motivators.
  assert(index_allocator_.Empty());
}

void MotiveProcessor::VerifyInternalState() const {
#if MOTIVE_VERIFY_INTERNAL_STATE
  // Check the validity of the IndexAllocator.
  index_allocator_.VerifyInternalState();

  // Check the validity of each Motivator.
  MotiveIndex len = static_cast<MotiveIndex>(motivators_.size());
  for (MotiveIndex i = 0; i < len; i += Dimensions(i)) {
    // If a Motivator is nullptr, its index should not be allocated.
    assert((motivators_[i] == nullptr && !index_allocator_.ValidIndex(i)) ||
           motivators_[i]->Valid());

    if (motivators_[i] == nullptr) continue;

    // All back pointers for a motivator should be the same.
    const MotiveDimension dimensions = Dimensions(i);
    for (MotiveIndex j = i + 1; j < i + dimensions; ++j) {
      assert(motivators_[i] == motivators_[j]);
    }

    // A Motivator should be referenced once.
    for (MotiveIndex j = i + dimensions; j < len; j += Dimensions(j)) {
      assert(motivators_[i] != motivators_[j]);
    }
  }
#endif  // MOTIVE_VERIFY_INTERNAL_STATE
}

void MotiveProcessor::InitializeMotivator(const MotivatorInit& init,
                                          MotiveEngine* engine,
                                          Motivator* motivator,
                                          MotiveDimension dimensions) {
  const motive::Benchmark b(benchmark_id_for_init());

  // Assign an 'index' to reference the new Motivator. All interactions between
  // the Motivator and MotiveProcessor use this 'index' to identify the data.
  const MotiveIndex index = index_allocator_.Alloc(dimensions);

  // Keep a pointer to the Motivator around. We may Defragment() the indices and
  // move the data around. We also need remove the Motivator when we're
  // destroyed.
  for (MotiveDimension i = 0; i < dimensions; ++i) {
    motivators_[index + i] = motivator;
  }

  // Initialize the motivator to point at our MotiveProcessor.
  motivator->Init(this, index);

  // Call the MotiveProcessor-specific initialization routine.
  InitializeIndices(init, index, dimensions, engine);

  VerifyInternalState();
}

// Don't notify derived classes. Useful in the destructor, since derived classes
// have already been destroyed.
void MotiveProcessor::RemoveMotivatorWithoutNotifying(MotiveIndex index) {
  // Ensure the Motivator no longer references us.
  motivators_[index]->Reset();

  // Ensure we no longer reference the Motivator.
  const MotiveDimension dimensions = Dimensions(index);
  for (MotiveDimension i = 0; i < dimensions; ++i) {
    motivators_[index + i] = nullptr;
  }

  // Recycle 'index'. It will be used in the next allocation, or back-filled in
  // the next call to Defragment().
  index_allocator_.Free(index);
}

void MotiveProcessor::RemoveMotivator(MotiveIndex index) {
  assert(ValidMotivatorIndex(index));

  // Call the MotiveProcessor-specific remove routine.
  RemoveIndices(index, Dimensions(index));

  // Need this version since the destructor can't call the pure virtual
  // RemoveIndex() above.
  RemoveMotivatorWithoutNotifying(index);

  VerifyInternalState();
}

void MotiveProcessor::TransferMotivator(MotiveIndex index,
                                        Motivator* new_motivator) {
  assert(ValidMotivatorIndex(index));

  // Ensure old Motivator does not reference us anymore. Only one Motivator is
  // allowed to reference 'index'.
  Motivator* old_motivator = motivators_[index];
  old_motivator->Reset();

  // Set up new_motivator to reference 'index'.
  new_motivator->Init(this, index);

  // Update our reference to the unique Motivator that references 'index'.
  const MotiveDimension dimensions = Dimensions(index);
  for (MotiveDimension i = 0; i < dimensions; ++i) {
    motivators_[index + i] = new_motivator;
  }

  VerifyInternalState();
}

void MotiveProcessor::SetEngine(MotiveEngine* engine) {
  if (engine_ != nullptr) {
    engine_ = engine;
  }
}

bool MotiveProcessor::IsMotivatorIndex(MotiveIndex index) const {
  return motivators_[index] != nullptr &&
         (index == 0 || motivators_[index - 1] != motivators_[index]);
}

bool MotiveProcessor::ValidIndex(MotiveIndex index) const {
  return index < index_allocator_.num_indices() &&
         motivators_[index] != nullptr &&
         motivators_[index]->Processor() == this;
}

bool MotiveProcessor::ValidMotivatorIndex(MotiveIndex index) const {
  return ValidIndex(index) && IsMotivatorIndex(index);
}

void MotiveProcessor::SetNumIndicesBase(MotiveIndex num_indices) {
  // When the size decreases, we don't bother reallocating the size of the
  // 'motivators_' vector. We want to avoid reallocating as much as possible,
  // so we let it grow to its high-water mark.
  //
  // TODO: Ideally, we should reserve approximately the right amount of storage
  // for motivators_. That would require adding a user-defined initialization
  // parameter.
  motivators_.resize(num_indices);

  // Call derived class.
  SetNumIndices(num_indices);
}

void MotiveProcessor::MoveIndexRangeBase(const IndexRange& source,
                                         MotiveIndex target) {
  // Reinitialize the motivators to point to the new index.
  const MotiveIndex index_diff = target - source.start();
  for (MotiveIndex i = source.start(); i < source.end(); i += Dimensions(i)) {
    motivators_[i]->Init(this, i + index_diff);
  }

  // Tell derivated class about the move.
  MoveIndices(source.start(), target, source.Length());

  // Reinitialize the motivator pointers.
  for (MotiveIndex i = source.start(); i < source.end(); ++i) {
    // Assert we're moving something valid onto something invalid.
    assert(motivators_[i] != nullptr && motivators_[i + index_diff] == nullptr);

    // Move our internal data too.
    motivators_[i + index_diff] = motivators_[i];
    motivators_[i] = nullptr;
  }
}

void MotiveProcessor::RegisterBenchmarks() {
  const std::string class_name(*Type());
  benchmark_id_for_advance_frame_ =
      motive::RegisterBenchmark((class_name + "::AdvanceFrame").c_str());
  benchmark_id_for_init_ =
      motive::RegisterBenchmark((class_name + "::Init").c_str());
}

}  // namespace motive
