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

#ifndef MOTIVE_RIG_PROCESSOR_H_
#define MOTIVE_RIG_PROCESSOR_H_

#include "motive/processor.h"

namespace motive {

class RigAnim;

class RigProcessor : public MotiveProcessor {
 public:
  /// Returns an array of length `DefiningAnim.NumBones()`.
  /// The i'th element of the array represents the transform from the root
  /// bone to the bone-space on the i'th bone.
  virtual const mathfu::AffineTransform* GlobalTransforms(
      MotiveIndex index) const = 0;

  /// Return the time remaining in the current rig animation.
  virtual MotiveTime TimeRemaining(MotiveIndex index) const = 0;

  /// Return the time remaining in the rig animation at `child_index` based on
  /// the order animations were passed to BlendToAnims(). If `child_index` is
  /// out of range, returns 0.
  /// Time units are defined by the user.
  virtual MotiveTime ChildTimeRemaining(MotiveIndex index,
                                        MotiveIndex child_index) const {
    return 0;
  }

  /// Return the animation that defines the rig.
  virtual const RigAnim* DefiningAnim(MotiveIndex index) const = 0;

  /// Return the animation that is currently playing.
  virtual const RigAnim* CurrentAnim(MotiveIndex index) const = 0;

  /// Smoothly transition to the animation in `anim`.
  virtual void BlendToAnim(MotiveIndex index, const RigAnim& anim,
                           const motive::SplinePlayback& playback) = 0;

  /// Smoothly transition to the animations in `anims`, blending them according
  /// to `weights` and using `playbacks` for each transition. All arguments must
  /// contain at least `count` elements.
  virtual void BlendToAnims(MotiveIndex index, const RigAnim** anims,
                            const SplinePlayback** playbacks,
                            const float* weights, int count) {}

  /// Instantly change the playback speed of this animation. If multiple
  /// animations are running, set the playback rate for all of them.
  virtual void SetPlaybackRate(MotiveIndex index, float playback_rate) = 0;

  /// Instantly change the playback speeds of individual running animations.
  virtual void SetPlaybackRates(MotiveIndex index, const float* playback_rates,
                                int count) {}

  /// Instantly change the weights of running animations.
  virtual void SetWeights(MotiveIndex index, const float* weights, int count) {}

  virtual std::string CsvHeaderForDebugging(MotiveIndex /*index*/) const {
    return std::string();
  }
  virtual std::string CsvValuesForDebugging(MotiveIndex /*index*/) const {
    return std::string();
  }
  virtual std::string LocalTransformsForDebugging(MotiveIndex /*index*/,
                                                  BoneIndex /*bone*/) const {
    return std::string();
  }
};

}  // namespace motive

#endif  // MOTIVE_RIG_PROCESSOR_H_
