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

#include <iomanip>
#include <sstream>

#include "mathfu/constants.h"
#include "motive/engine.h"
#include "motive/rig_init.h"
#include "motive/math/angle.h"
#include "motive/math/bulk_spline_evaluator.h"
#include "motive/processor/rig_data.h"
#include "motive/rig_anim.h"
#include "motive/rig_processor.h"

namespace motive {

// See comments on RigInit for details on this class.
class MotiveRigProcessor : public RigProcessor {
 public:
  MotiveRigProcessor() : time_(0) {}

  virtual ~MotiveRigProcessor() {
    RemoveIndices(0, NumIndices());
  }

  void AdvanceFrame(MotiveTime delta_time) override {
    Defragment();

    // Process the series of matrix operations for each index.
    const MotiveIndex num_indices = NumIndices();
    for (MotiveIndex index = 0; index < num_indices; ++index) {
      RigData& d = Data(index);
      d.UpdateGlobalTransforms();
    }

    // Update our global time. It shouldn't matter if this wraps
    // around, since we only calculate times relative to it.
    time_ += delta_time;
  }

  void BlendToAnim(MotiveIndex index, const RigAnim& anim,
                   const motive::SplinePlayback& playback) override {
    Data(index).BlendToAnim(anim, playback, time_);
  }

  void BlendToAnims(MotiveIndex index, const RigAnim** anims,
                    const SplinePlayback** playbacks, const float* weights,
                    int count) override {
    Data(index).BlendToAnims(anims, playbacks, weights, count, Engine(), time_);
  }

  void SetPlaybackRate(MotiveIndex index, float playback_rate) override {
    Data(index).SetPlaybackRate(playback_rate);
  }

  void SetPlaybackRates(MotiveIndex index, const float* playback_rates,
                        int count) {
    Data(index).SetPlaybackRates(playback_rates, count);
  }

  void SetWeights(MotiveIndex index, const float* weights, int count) {
    Data(index).SetWeights(weights, count);
  }

  MotivatorType Type() const override { return RigInit::kType; }
  int Priority() const override { return 3; }

  const mathfu::AffineTransform* GlobalTransforms(
      MotiveIndex index) const override {
    return Data(index).GlobalTransforms();
  }

  MotiveTime TimeRemaining(MotiveIndex index) const override {
    return Data(index).TimeRemaining();
  }

  MotiveTime ChildTimeRemaining(MotiveIndex index,
                                MotiveIndex child_index) const override {
    return Data(index).ChildTimeRemaining(child_index);
  }

  const RigAnim* DefiningAnim(MotiveIndex index) const override {
    return Data(index).defining_anim();
  }

  const RigAnim* CurrentAnim(MotiveIndex index) const override {
    return Data(index).current_anim();
  }

  std::string CsvHeaderForDebugging(MotiveIndex index) const override {
    return Data(index).CsvHeaderForDebugging();
  }

  std::string CsvValuesForDebugging(MotiveIndex index) const override {
    return Data(index).CsvValuesForDebugging(time_);
  }

  std::string LocalTransformsForDebugging(MotiveIndex index,
                                          BoneIndex bone) const override {
    return Data(index).LocalTransformsForDebugging(bone, time_);
  }

 protected:
  MotiveIndex NumIndices() const {
    return static_cast<MotiveIndex>(data_.size());
  }

  void InitializeIndices(const MotivatorInit& init, MotiveIndex index,
                         MotiveDimension dimensions,
                         MotiveEngine* engine) override {
    // Hold onto the engine for use in BlendToAnims().
    RemoveIndices(index, dimensions);
    auto rig_init = static_cast<const RigInit&>(init);
    for (MotiveIndex i = index; i < index + dimensions; ++i) {
      data_[i] = new RigData(rig_init, time_, engine);
    }
  }

  void RemoveIndices(MotiveIndex index, MotiveDimension dimensions) override {
    for (MotiveIndex i = index; i < index + dimensions; ++i) {
      if (data_[i] == nullptr) continue;
      delete data_[i];
      data_[i] = nullptr;
    }
  }

  void MoveIndices(MotiveIndex old_index, MotiveIndex new_index,
                   MotiveDimension dimensions) override {
    MotiveIndex old_i = old_index;
    MotiveIndex new_i = new_index;
    for (MotiveDimension i = 0; i < dimensions; ++i, ++new_i, ++old_i) {
      data_[new_i] = data_[old_i];
      data_[old_i] = nullptr;
    }
  }

  void SetNumIndices(MotiveIndex num_indices) override {
    // Ensure old items are deleted.
    const MotiveIndex old_num_indices = NumIndices();
    if (old_num_indices > num_indices) {
      RemoveIndices(num_indices, old_num_indices - num_indices);
    }

    // Initialize new items to nullptr.
    data_.resize(num_indices, nullptr);
  }

  const RigData& Data(MotiveIndex index) const {
    assert(ValidIndex(index));
    return *data_[index];
  }

  RigData& Data(MotiveIndex index) {
    assert(ValidIndex(index));
    return *data_[index];
  }

  std::vector<RigData*> data_;
  MotiveTime time_;
};

MOTIVE_INSTANCE(RigInit, MotiveRigProcessor);

}  // namespace motive
