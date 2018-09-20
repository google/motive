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
#include "motive/math/bulk_spline_evaluator.h"
#include "motive/math/compact_spline.h"
#include "motive/processor/spline_data.h"
#include "motive/spline_init.h"

namespace motive {

// Add some buffer to the y-range to allow for intermediate nodes
// that go above or below the supplied nodes.
static const float kYRangeBufferPercent = 1.2f;

class SplineMotiveProcessor : public MotiveProcessorNf {
 public:
  virtual ~SplineMotiveProcessor() {
    for (auto it = spline_pool_.begin(); it != spline_pool_.end(); ++it) {
      CompactSpline::Destroy(*it);
    }
  }

  virtual void AdvanceFrame(MotiveTime delta_time) {
    Defragment();
    interpolator_.AdvanceFrame(static_cast<float>(delta_time));
  }

  virtual MotivatorType Type() const { return SplineInit::kType; }
  virtual int Priority() const { return 0; }

  // Accessors to allow the user to get and set simluation values.
  virtual const float* Values(MotiveIndex index) const {
    return interpolator_.Ys(index);
  }
  virtual void Velocities(MotiveIndex index, MotiveDimension dimensions,
                          float* out) const {
    return interpolator_.Derivatives(index, dimensions, out);
  }
  virtual void Directions(MotiveIndex index, MotiveDimension dimensions,
                          float* out) const {
    return interpolator_.DerivativesWithoutPlayback(index, dimensions, out);
  }
  virtual void TargetValues(MotiveIndex index, MotiveDimension dimensions,
                            float* out) const {
    return interpolator_.EndYs(index, dimensions, out);
  }
  virtual void TargetVelocities(MotiveIndex index, MotiveDimension dimensions,
                                float* out) const {
    return interpolator_.EndDerivatives(index, dimensions, out);
  }
  virtual void Differences(MotiveIndex index, MotiveDimension dimensions,
                           float* out) const {
    return interpolator_.YDifferencesToEnd(index, dimensions, out);
  }
  virtual MotiveTime TargetTime(MotiveIndex index,
                                MotiveDimension dimensions) const {
    MotiveTime greatest = std::numeric_limits<MotiveTime>::min();
    for (MotiveDimension i = 0; i < dimensions; ++i) {
      greatest =
          std::max(greatest,
                   static_cast<MotiveTime>(interpolator_.EndX(index + i)
                                           - interpolator_.X(index + i)));
    }
    return greatest;
  }
  virtual MotiveTime SplineTime(MotiveIndex index) const {
    return static_cast<MotiveTime>(interpolator_.X(index));
  }

  virtual void SetTargets(MotiveIndex index, MotiveDimension dimensions,
                          const MotiveTarget1f* ts) {
    for (MotiveDimension i = 0; i < dimensions; ++i) {
      SetTarget(index + i, ts[i]);
    }
  }

  virtual void SetSplines(MotiveIndex index, MotiveDimension dimensions,
                          const CompactSpline* splines,
                          const SplinePlayback& playback) {
    // Return the local splines to the spline pool. We use external splines now.
    for (MotiveDimension i = index; i < index + dimensions; ++i) {
      FreeSplineForIndex(i);
    }

    // Initialize spline to follow way points.
    // Snaps the current value and velocity to the way point's start value
    // and velocity.
    interpolator_.SetSplines(index, dimensions, splines, playback);
  }

  virtual void SetSplinesAndTargets(MotiveIndex index,
                                    MotiveDimension dimensions,
                                    const CompactSpline* const* splines,
                                    const SplinePlayback& playback,
                                    const MotiveTarget1f* targets) {
    // Initialize either with a spline or a target.
    // We initialize one by one instead of in bulk. Not as efficient.
    for (MotiveDimension i = 0; i < dimensions; ++i) {
      if (splines[i] == nullptr) {
        SetTarget(index + i, targets[i]);
      } else {
        FreeSplineForIndex(index + i);
        interpolator_.SetSplines(index + i, 1, splines[i], playback);
      }
    }
  }

  virtual void Splines(MotiveIndex index, MotiveDimension dimensions,
                       const motive::CompactSpline** splines) const {
    // Get splines at index for dimensions.
    interpolator_.Splines(index, dimensions, splines);
  }

  // TODO: Push this loop into BulkSplineInterpolator.
  virtual void SetSplineTime(MotiveIndex index, MotiveDimension dimensions,
                             MotiveTime time) {
    interpolator_.SetXs(index, dimensions, static_cast<float>(time));
  }

  // TODO: Push this loop into BulkSplineInterpolator.
  virtual void SetSplinePlaybackRate(MotiveIndex index,
                                     MotiveDimension dimensions,
                                     float playback_rate) {
    interpolator_.SetPlaybackRates(index, dimensions, playback_rate);
  }

 protected:
  // TODO: Change to CreateSplineToTarget()
  void SetTarget(MotiveIndex index, const MotiveTarget1f& t) {
    SplineData& d = Data(index);

    // If the first node specifies time=0 or there is no valid data in the
    // interpolator, we want to override the current values with the values
    // specified in the first node.
    const MotiveNode1f& node0 = t.Node(0);
    const bool override_current =
        node0.time == 0 || !interpolator_.Valid(index);
    // TODO(b/65298927):  It seems that the animation pipeline can produce data
    // that is out of range.  Instead of just using node0.value directly, if
    // the interpolator is doing modular arithmetic, normalize the y value to
    // the modulator's range.
    const float start_y =
        override_current
            ? (interpolator_.ModularArithmetic(index)
                   ? interpolator_.ModularRange(index).NormalizeWildValue(
                         node0.value)
                   : node0.value)
            : interpolator_.NormalizedY(index);
    const float start_derivative =
        override_current ? node0.velocity : Velocity(index);
    const int start_node_index = override_current ? 1 : 0;

    // Ensure we have a local spline available, allocated from our pool of
    // splines.
    if (d.local_spline == nullptr) {
      // The default number of nodes is enough.
      d.local_spline = AllocateSpline(CompactSpline::kDefaultMaxNodes);
    }

    // Initialize the compact spline to hold the sequence of nodes in 't'.
    // Add the first node, which has the start condition.
    const float end_x = static_cast<float>(t.EndTime());
    const Range y_range = CalculateYRange(index, t, start_y);
    const float x_granularity = CompactSpline::RecommendXGranularity(end_x);
    d.local_spline->Init(y_range, x_granularity);
    d.local_spline->AddNode(0.0f, start_y, start_derivative);

    // Add subsequent nodes, in turn, taking care to respect the 'direction'
    // request when using modular arithmetic.
    float prev_y = start_y;
    for (int i = start_node_index; i < t.num_nodes(); ++i) {
      const MotiveNode1f& n = t.Node(i);
      const float target_y =
          interpolator_.ModularArithmetic(index)
              ? interpolator_.ModularRange(index).NormalizeWildValue(n.value)
              : n.value;
      const float y = interpolator_.NextY(index, prev_y, target_y, n.direction);
      d.local_spline->AddNode(static_cast<float>(n.time), y, n.velocity,
                              motive::kAddWithoutModification);
      prev_y = y;
    }

    // Point the interpolator at the spline we just created. Always start our
    // spline at time 0.
    interpolator_.SetSplines(index, 1, d.local_spline, SplinePlayback());
  }

  virtual void InitializeIndices(const MotivatorInit& init, MotiveIndex index,
                                 MotiveDimension dimensions,
                                 MotiveEngine* /*engine*/) {
    auto spline_init = static_cast<const SplineInit&>(init);
    interpolator_.SetYRanges(index, dimensions, spline_init.range());
  }

  bool SupportsCloning() override { return true; }

  void CloneIndices(MotiveIndex dst, MotiveIndex src,
                    MotiveDimension dimensions,
                    MotiveEngine* /*engine*/) override {
    interpolator_.CopyIndices(
        dst, src, dimensions,
        [this](MotiveIndex index, const CompactSpline* src_spline) {
          CompactSpline* dest_spline = AllocateSpline(src_spline->max_nodes());
          *dest_spline = *src_spline;
          Data(index).local_spline = dest_spline;
          return dest_spline;
        });
  }

  virtual void RemoveIndices(MotiveIndex index, MotiveDimension dimensions) {
    // Clear reference to this spline.
    interpolator_.ClearSplines(index, dimensions);

    // Return splines to the pool of splines.
    for (MotiveIndex i = index; i < index + dimensions; ++i) {
      SplineData& d = Data(i);
      FreeSpline(d.local_spline);
      d.local_spline = nullptr;
    }
  }

  virtual void MoveIndices(MotiveIndex old_index, MotiveIndex new_index,
                           MotiveDimension dimensions) {
    MotiveIndex old_i = old_index;
    MotiveIndex new_i = new_index;
    for (MotiveDimension i = 0; i < dimensions; ++i, ++new_i, ++old_i) {
      data_[new_i] = data_[old_i];
    }
    interpolator_.MoveIndices(old_index, new_index, dimensions);
  }

  virtual void SetNumIndices(MotiveIndex num_indices) {
    data_.resize(num_indices);
    interpolator_.SetNumIndices(num_indices);
  }

  const SplineData& Data(MotiveIndex index) const {
    assert(ValidIndex(index));
    return data_[index];
  }

  SplineData& Data(MotiveIndex index) {
    assert(ValidIndex(index));
    return data_[index];
  }

  CompactSpline* AllocateSpline(CompactSplineIndex max_nodes) {
    // Return a spline from the pool. Eventually we'll reach a high water mark
    // and we will stop allocating new splines. The returned spline must have
    // enough nodes.
    for (size_t i = 0; i < spline_pool_.size(); ++i) {
      CompactSpline* spline = spline_pool_[i];
      if (spline->max_nodes() >= max_nodes) {
        spline_pool_[i] = spline_pool_.back();
        spline_pool_.pop_back();
        return spline;
      }
    }

    // Create a spline with enough nodes otherwise.
    return CompactSpline::Create(max_nodes);
  }

  void FreeSplineForIndex(MotiveIndex index) {
    SplineData& d = Data(index);
    FreeSpline(d.local_spline);
    d.local_spline = nullptr;
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
  std::vector<SplineData> data_;

  // Holds unused splines. When we need another local spline (because we're
  // supplied with target values but not the actual curve to get there),
  // try to recycle an old one from this pool first.
  std::vector<CompactSpline*> spline_pool_;

  // Perform the spline evaluation, over time. Indices in 'interpolator_'
  // are the same as the MotiveIndex values in this class.
  BulkSplineEvaluator interpolator_;
};

MOTIVE_INSTANCE(SplineInit, SplineMotiveProcessor);

}  // namespace motive
