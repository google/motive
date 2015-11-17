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

#include "motive/engine.h"
#include "motive/init.h"
#include "motive/math/bulk_spline_evaluator.h"

namespace motive {

using motive::CompactSpline;
using motive::BulkSplineEvaluator;
using motive::Range;

// Add some buffer to the y-range to allow for intermediate nodes
// that go above or below the supplied nodes.
static const float kYRangeBufferPercent = 1.2f;

// An intermediate node might be inserted to make the cubic curve well
// behaved, so reserve 3 nodes in the spline.
static const int kMaxNodesInLocalSpline = 2 * MotiveTarget1f::kMaxNodes + 1;

struct SmoothData {
  SmoothData() : local_spline(nullptr) {}

  // If we own the spline, recycle it in the spline pool.
  CompactSpline* local_spline;
};

class SmoothMotiveProcessor : public VectorProcessor {
 public:
  virtual ~SmoothMotiveProcessor() {
    for (auto it = spline_pool_.begin(); it != spline_pool_.end(); ++it) {
      delete *(it);
    }
  }

  virtual void AdvanceFrame(MotiveTime delta_time) {
    Defragment();
    interpolator_.AdvanceFrame(static_cast<float>(delta_time));
  }

  virtual MotivatorType Type() const { return SmoothInit::kType; }
  virtual int Priority() const { return 0; }

  // Accessors to allow the user to get and set simluation values.
  virtual float Value1f(MotiveIndex index) const {
    return interpolator_.Y(index);
  }
  virtual float Velocity1f(MotiveIndex index) const {
    return interpolator_.Derivative(index);
  }
  virtual float Direction1f(MotiveIndex index) const {
    return interpolator_.DerivativeWithoutPlayback(index);
  }
  virtual float TargetValue1f(MotiveIndex index) const {
    return interpolator_.EndY(index);
  }
  virtual float TargetVelocity1f(MotiveIndex index) const {
    return interpolator_.EndDerivative(index);
  }
  virtual float Difference1f(MotiveIndex index) const {
    return interpolator_.YDifferenceToEnd(index);
  }
  virtual MotiveTime TargetTime(MotiveIndex index) const {
    return static_cast<MotiveTime>(interpolator_.EndX(index) -
                                   interpolator_.X(index));
  }
  virtual MotiveTime SplineTime(MotiveIndex index) const {
    return static_cast<MotiveTime>(interpolator_.X(index));
  }

  virtual void SetTarget(MotiveIndex index, const MotiveTarget1f& t) {
    SmoothData& d = Data(index);

    // If the first node specifies time=0, that means we want to override the
    // current values with the values specified in the first node.
    const MotiveNode1f& node0 = t.Node(0);
    const bool override_current = node0.time == 0;
    const float start_y = override_current ? node0.value
                                           : interpolator_.NormalizedY(index);
    const float start_derivative =
        override_current ? node0.velocity : Velocity1f(index);
    const int start_node_index = override_current ? 1 : 0;

    // Ensure we have a local spline available, allocated from our pool of
    // splines.
    if (d.local_spline == nullptr) {
      d.local_spline = AllocateSpline();
    }

    // Initialize the compact spline to hold the sequence of nodes in 't'.
    // Add the first node, which has the start condition.
    const float end_x = static_cast<float>(t.EndTime());
    const Range y_range = CalculateYRange(index, t, start_y);
    const float x_granularity = CompactSpline::RecommendXGranularity(end_x);
    d.local_spline->Init(y_range, x_granularity, kMaxNodesInLocalSpline);
    d.local_spline->AddNode(0.0f, start_y, start_derivative);

    // Add subsequent nodes, in turn, taking care to respect the 'direction'
    // request when using modular arithmetic.
    float prev_y = start_y;
    for (int i = start_node_index; i < t.num_nodes(); ++i) {
      const MotiveNode1f& n = t.Node(i);
      const float y = interpolator_.NextY(index, prev_y, n.value, n.direction);
      d.local_spline->AddNode(static_cast<float>(n.time), y, n.velocity,
                              motive::kAddWithoutModification);
      prev_y = y;
    }

    // Point the interpolator at the spline we just created. Always start our
    // spline at time 0.
    interpolator_.SetSpline(index, *d.local_spline, motive::SplinePlayback());
  }

  virtual void SetSpline(MotiveIndex index, const motive::CompactSpline& spline,
                         const motive::SplinePlayback& playback) {
    SmoothData& d = Data(index);

    // Return the local spline to the spline pool. We use external splines now.
    FreeSpline(d.local_spline);
    d.local_spline = nullptr;

    // Initialize spline to follow way points.
    // Snaps the current value and velocity to the way point's start value
    // and velocity.
    interpolator_.SetSpline(index, spline, playback);
  }

  virtual void SetSplineTime(MotiveIndex index, MotiveTime time) {
    const MotiveIndex end_index = index + Dimensions(index);
    for (MotiveIndex i = index; i < end_index; ++i) {
      interpolator_.SetX(i, static_cast<float>(time));
    }
  }

  virtual void SetSplinePlaybackRate(MotiveIndex index, float playback_rate) {
    const MotiveIndex end_index = index + Dimensions(index);
    for (MotiveIndex i = index; i < end_index; ++i) {
      interpolator_.SetPlaybackRate(i, playback_rate);
    }
  }

 protected:
  virtual void InitializeIndex(const MotivatorInit& init, MotiveIndex index,
                               MotiveEngine* engine) {
    (void)engine;
    auto smooth = static_cast<const SmoothInit&>(init);
    interpolator_.SetYRange(index, smooth.range(), smooth.modular());
  }

  virtual void RemoveIndex(MotiveIndex index) {
    // Clear reference to this spline.
    interpolator_.ClearSpline(index);

    // Return the spline to the pool of splines.
    SmoothData& d = Data(index);
    FreeSpline(d.local_spline);
    d.local_spline = nullptr;
  }

  virtual void MoveIndex(MotiveIndex old_index, MotiveIndex new_index) {
    data_[new_index] = data_[old_index];
    interpolator_.MoveIndex(old_index, new_index);
  }

  virtual void SetNumIndices(MotiveIndex num_indices) {
    data_.resize(num_indices);
    interpolator_.SetNumIndices(num_indices);
  }

  const SmoothData& Data(MotiveIndex index) const {
    assert(ValidIndex(index));
    return data_[index];
  }

  SmoothData& Data(MotiveIndex index) {
    assert(ValidIndex(index));
    return data_[index];
  }

  CompactSpline* AllocateSpline() {
    // Only create a new spline if there are no left in the pool.
    if (spline_pool_.empty()) return new CompactSpline();

    // Return a spline from the pool. Eventually we'll reach a high water mark
    // and we will stop allocating new splines.
    CompactSpline* spline = spline_pool_.back();
    spline_pool_.pop_back();
    return spline;
  }

  void FreeSpline(CompactSpline* spline) {
    if (spline != nullptr) {
      spline_pool_.push_back(spline);
    }
  }

  Range CalculateYRange(MotiveIndex index, const MotiveTarget1f& t,
                        float start_y) const {
    if (interpolator_.ModularArithmetic(index)) {
      // For modular splines, we need to expand the spline's y-range to match
      // the number of nodes in the spline. It's possible for the spline to jump
      // up the entire range every node, so the range has to be broad enough
      // to hold it all.
      //
      // Note that we only normalize the first value of the spline, and
      // subsequent values are allowed to curve out of the normalized range.
      const float num_spline_nodes = static_cast<float>(t.num_nodes());
      return interpolator_.ModularRange(index).Lengthen(num_spline_nodes);
    }

    // Calculate the union of the y ranges in the target, then expand it a
    // little to allow for intermediate nodes that jump slightly beyond the
    // union's range.
    return t.ValueRange(start_y).Lengthen(kYRangeBufferPercent);
  }

  // Hold index-specific data, for example a pointer to the spline allocated
  // from 'spline_pool_'.
  std::vector<SmoothData> data_;

  // Holds unused splines. When we need another local spline (because we're
  // supplied with target values but not the actual curve to get there),
  // try to recycle an old one from this pool first.
  std::vector<CompactSpline*> spline_pool_;

  // Perform the spline evaluation, over time. Indices in 'interpolator_'
  // are the same as the MotiveIndex values in this class.
  BulkSplineEvaluator interpolator_;
};

MOTIVE_INSTANCE(SmoothInit, SmoothMotiveProcessor);

}  // namespace motive
