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

namespace motive {

MotiveProcessor::~MotiveProcessor() {
  // Reset all of the Motivators that we're currently driving.
  // We don't want any of them to reference us after we've been destroyed.
  for (MotiveIndex index = 0; index < index_allocator_.num_indices(); ++index) {
    if (motivators_[index] != nullptr) {
      RemoveMotivatorWithoutNotifying(index);
    }
  }

  // Sanity-check: Ensure that we have no more active Motivators.
  assert(index_allocator_.Empty());
}

void MotiveProcessor::ValidInternalState() const {
  // Only Motivators at the end should be nullptr. Skip checking those.
  MotiveIndex len = static_cast<MotiveIndex>(motivators_.size());
  for (; len > 0 && motivators_[len - 1] == nullptr; --len) {}

  // Check the validity of each Motivator.
  for (MotiveIndex i = 0; i < len; ++i) {
    // If a motivator is nullptr, its index should not be allocated.
    assert((motivators_[i] == nullptr && !index_allocator_.ValidIndex(i)) ||
           motivators_[i]->Valid());

    if (motivators_[i] == nullptr)
      continue;

    for (MotiveIndex j = i + 1; j < len; ++j) {
      assert(motivators_[i] != motivators_[j]);
    }
  }
}

void MotiveProcessor::InitializeMotivator(const MotivatorInit& init,
                                          MotiveEngine* engine,
                                          Motivator* motivator) {
  // Assign an 'index' to reference the new Motivator. All interactions between
  // the Motivator and MotiveProcessor use this 'index' to identify the data.
  const MotiveIndex index = index_allocator_.Alloc();

  // Keep a pointer to the Motivator around. We may Defragment() the indices and
  // move the data around. We also need remove the Motivator when we're
  // destroyed.
  motivators_[index] = motivator;

  // Initialize the motivator to point at our MotiveProcessor.
  motivator->Init(this, index);

  // Call the MotiveProcessor-specific initialization routine.
  InitializeIndex(init, index, engine);
}

// Don't notify derived classes. Useful in the destructor, since derived classes
// have already been destroyed.
void MotiveProcessor::RemoveMotivatorWithoutNotifying(MotiveIndex index) {
  // Ensure the Motivator no longer references us.
  motivators_[index]->Reset();

  // Ensure we no longer reference the Motivator.
  motivators_[index] = nullptr;

  // Recycle 'index'. It will be used in the next allocation, or back-filled in
  // the next call to Defragment().
  index_allocator_.Free(index);
}

void MotiveProcessor::RemoveMotivator(MotiveIndex index) {
  assert(ValidIndex(index));

  // Call the MotiveProcessor-specific remove routine.
  RemoveIndex(index);

  // Need this version since the destructor can't call the pure virtual
  // RemoveIndex() above.
  RemoveMotivatorWithoutNotifying(index);
}

void MotiveProcessor::TransferMotivator(MotiveIndex index,
                                        Motivator* new_motivator) {
  assert(ValidIndex(index));

  // Ensure old Motivator does not reference us anymore. Only one Motivator is
  // allowed to reference 'index'.
  Motivator* old_motivator = motivators_[index];
  old_motivator->Reset();

  // Set up new_motivator to reference 'index'.
  new_motivator->Init(this, index);

  // Update our reference to the unique Motivator that references 'index'.
  motivators_[index] = new_motivator;
}

bool MotiveProcessor::ValidIndex(MotiveIndex index) const {
  return index < index_allocator_.num_indices() &&
         motivators_[index] != nullptr &&
         motivators_[index]->Processor() == this;
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

void MotiveProcessor::MoveIndexBase(MotiveIndex old_index,
                                    MotiveIndex new_index) {
  // Assert we're moving something valid onto something invalid.
  assert(motivators_[new_index] == nullptr &&
         motivators_[old_index] != nullptr);

  // Reinitialize the motivator to point to the new index.
  Motivator* motivator = motivators_[old_index];
  motivator->Init(this, new_index);

  // Swap the pointer values stored at indices.
  motivators_[new_index] = motivator;
  motivators_[old_index] = nullptr;

  // Call derived class so the derived class can perform similar data movement.
  MoveIndex(old_index, new_index);
}

}  // namespace motive
