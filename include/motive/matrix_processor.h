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

#ifndef MOTIVE_MATRIX_PROCESSOR_H_
#define MOTIVE_MATRIX_PROCESSOR_H_

#include "motive/matrix_op.h"
#include "motive/processor.h"

namespace motive {

class MatrixOpArray;

/// @class MatrixProcessor4f
/// @brief Interface for motivator types that drive a 4x4 float matrix.
/// That is, for MotiveProcessors that interface with MatrixMotivator4f's.
class MatrixProcessor4f : public MotiveProcessor {
 public:
  /// Get the current matrix value from the processor.
  virtual const mathfu::mat4& Value(MotiveIndex index) const = 0;

  /// Get the number of matrix operations performed by this motivator.
  virtual int NumChildren(MotiveIndex index) const = 0;

  /// Get current values of the components that create the matrix.
  virtual void ChildValues(MotiveIndex index, MotiveChildIndex child_index,
                           MotiveChildIndex count, float* values) const = 0;

  /// Get the Motivator1f driving this child, if this child is driven by
  /// a Motivator1f, or nullptr otherwise.
  virtual const Motivator* ChildMotivator1f(
      MotiveIndex index, MotiveChildIndex child_index) const = 0;

  /// Set child values. Matrices are composed from child components.
  virtual void SetChildTarget1f(MotiveIndex /*index*/,
                                MotiveChildIndex /*child_index*/,
                                const MotiveTarget1f& /*t*/) {}
  virtual void SetChildValues(MotiveIndex index, MotiveChildIndex child_index,
                              MotiveChildIndex count, const float* values) = 0;

  /// Smoothly transition to the operations specified in `ops`.
  virtual void BlendToOps(MotiveIndex /*index*/, const MatrixOpArray& /*ops*/,
                          const motive::SplinePlayback& /*playback*/) {}

  /// Instantly change the playback speed of this animation.
  virtual void SetPlaybackRate(MotiveIndex index, float playback_rate) = 0;

  /// Returns the time remaining to reach the end of the animation.  If the
  /// animation is looping, then returns kMotiveTimeEndless.  This function does
  /// not take the PlaybackRate into account.  For example, a 1s animation with
  /// a playback rate of 0.5 will take 2s to finish.  However, TimeRemaining()
  /// at the start of the animation will return 1s.
  virtual MotiveTime TimeRemaining(MotiveIndex index) const = 0;
};

}  // namespace motive

#endif  // MOTIVE_MATRIX_PROCESSOR_H_
