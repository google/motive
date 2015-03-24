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

using fpl::CompactSpline;
using fpl::BulkSplineEvaluator;
using fpl::Range;

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

class SmoothMotiveProcessor : public MotiveProcessor1f {
 public:
  virtual ~SmoothMotiveProcessor() {
    for (auto it = spline_pool_.begin(); it != spline_pool_.end(); ++it) {
      delete *(it);
    }
  }

  virtual void AdvanceFrame(MotiveTime delta_time) {
    Defragment();
    interpolator_.AdvanceFrame(delta_time);
  }

  virtual MotivatorType Type() const { return SmoothInit::kType; }
  virtual int Priority() const { return 0; }

  // Accessors to allow the user to get and set simluation values.
  virtual float Value(MotiveIndex index) const {
    return interpolator_.Y(index);
  }
  virtual float Velocity(MotiveIndex index) const {
    return interpolator_.Derivative(index);
  }
  virtual float TargetValue(MotiveIndex index) const {
    return interpolator_.EndY(index);
  }
  virtual float TargetVelocity(MotiveIndex index) const {
    return interpolator_.EndDerivative(index);
  }
  virtual float Difference(MotiveIndex index) const {
    return interpolator_.YDifferenceToEnd(index);
  }
  virtual MotiveTime TargetTime(MotiveIndex index) const {
    return static_cast<MotiveTime>(interpolator_.EndX(index) -
                                   interpolator_.X(index));
  }

  virtual void SetTarget(MotiveIndex index, const MotiveTarget1f& t) {
    SmoothData& d = Data(index);

    // If the first node specifies time=0, that means we want to override the
    // current values with the values specified in the first node.
    const MotiveNode1f& node0 = t.Node(0);
    const bool override_current = node0.time == 0;
    const float start_y = override_current ? node0.value : Value(index);
    const float start_derivative =
        override_current ? node0.velocity : Velocity(index);
    const float start_node_index = override_current ? 1 : 0;

    // Ensure we have a local spline available, allocated from our pool of
    // splines.
    if (d.local_spline == nullptr) {
      d.local_spline = AllocateSpline();
    }

    // Initialize the compact spline to hold the sequence of nodes in 't'.
    // Add the first node, which has the start condition.
    const float end_x = static_cast<float>(t.EndTime());
    const Range y_range = t.ValueRange(start_y).Lengthen(kYRangeBufferPercent);
    const float x_granularity = CompactSpline::RecommendXGranularity(end_x);
    d.local_spline->Init(y_range, x_granularity, kMaxNodesInLocalSpline);
    d.local_spline->AddNode(0.0f, start_y, start_derivative);

    // Add subsequent nodes, in turn, taking care to respect the 'direction'
    // request when using modular arithmetic.
    float prev_y = start_y;
    for (int i = start_node_index; i < t.num_nodes(); ++i) {
      const MotiveNode1f& n = t.Node(i);
      const float y = interpolator_.NextY(index, prev_y, n.value, n.direction);
      d.local_spline->AddNode(static_cast<float>(n.time), y, n.velocity);
      prev_y = y;
    }

    // Point the interpolator at the spline we just created. Always start our
    // spline at time 0.
    interpolator_.SetSpline(index, fpl::SplinePlayback(*d.local_spline));
  }

  virtual void SetSpline(MotiveIndex index, const fpl::SplinePlayback& s) {
    SmoothData& d = Data(index);

    // Return the local spline to the spline pool. We use external splines now.
    FreeSpline(d.local_spline);

    // Initialize spline to follow way points.
    // Snaps the current value and velocity to the way point's start value
    // and velocity.
    interpolator_.SetSpline(index, s);
  }

 protected:
  virtual void InitializeIndex(const MotivatorInit& init, MotiveIndex index,
                               MotiveEngine* engine) {
    (void)engine;
    auto smooth = static_cast<const SmoothInit&>(init);
    interpolator_.SetYRange(index, smooth.range(), smooth.modular());
  }

  virtual void RemoveIndex(MotiveIndex index) {
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
