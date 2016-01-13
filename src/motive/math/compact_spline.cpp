// Copyright 2015 Google Inc. All rights reserved.
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

#include <string>
#include <sstream>
#include <vector>
#include "motive/math/bulk_spline_evaluator.h"
#include "motive/math/dual_cubic.h"

namespace motive {

using mathfu::Lerp;
using detail::CompactSplineNode;

// static constants
const size_t CompactSpline::kBaseSize =
    sizeof(CompactSpline) -
    kDefaultMaxNodes * sizeof(detail::CompactSplineNode);
const CompactSplineXGrain CompactSplineNode::kMaxX =
    std::numeric_limits<CompactSplineXGrain>::max();
const CompactSplineYRung CompactSplineNode::kMaxY =
    std::numeric_limits<CompactSplineXGrain>::max();
const CompactSplineAngle CompactSplineNode::kMinAngle =
    std::numeric_limits<CompactSplineAngle>::min();
const float CompactSplineNode::kYScale = 1.0f / static_cast<float>(kMaxY);
const float CompactSplineNode::kAngleScale =
    static_cast<float>(-M_PI / static_cast<double>(kMinAngle));



void CompactSpline::AddNode(const float x, const float y,
                            const float derivative,
                            const CompactSplineAddMethod method) {
  const CompactSplineNode new_node(x, y, derivative, x_granularity_, y_range_);

  // Precondition: Nodes must come *after* the last node.
  // Due to rounding, it's possible that the we have the *same* x as the last
  // node. This is valid and we do not assert, but we do return immediately.
  assert(num_nodes_ == 0 || new_node.x() >= Back().x());
  const bool strictly_after_last_node =
      num_nodes_ == 0 || new_node.x() > Back().x();
  if (!strictly_after_last_node) return;

  // Add a dual-cubic mid-node, if required, to keep cubic curves well behaved.
  if (method == kEnsureCubicWellBehaved && num_nodes_ != 0) {
    const CompactSplineNode& last_node = Back();
    const CubicInit init = CreateCubicInit(last_node, new_node);
    const CubicCurve curve(init);

    // A curve is well behaved if it has uniform curvature.
    if (!curve.UniformCurvature(Range(0.0f, WidthX(last_node, new_node)))) {
      // Find a suitable intermediate node using the math from the Dual Cubics
      // document.
      float mid_x;
      float mid_y;
      float mid_derivative;
      CalculateDualCubicMidNode(init, &mid_x, &mid_y, &mid_derivative);

      // Add the intermediate node, as long as it
      const CompactSplineNode mid_node(last_node.X(x_granularity_) + mid_x,
                                       mid_y, mid_derivative, x_granularity_,
                                       y_range_);
      const bool is_unique_x =
          mid_node.x() != last_node.x() && mid_node.x() != new_node.x();
      if (is_unique_x) {
        AddNodeVerbatim(mid_node);
      }
    }
  }

  // Add the new node.
  AddNodeVerbatim(new_node);
}

float CompactSpline::NodeX(const CompactSplineIndex index) const {
  // Note that, when `index` is before the spline, we return x=0 instead of
  // x=first node's x. This is because logically the spline always starts at
  // x=0, so anything before the first node is in an implicit segment from
  // x=0..first node's x.
  if (index == kAfterSplineIndex) return EndX();
  if (index == kBeforeSplineIndex) return 0.0f;
  assert(index < num_nodes_);
  return nodes_[index].X(x_granularity_);
}

float CompactSpline::NodeY(const CompactSplineIndex index) const {
  if (index == kAfterSplineIndex) return EndY();
  if (index == kBeforeSplineIndex) return StartY();
  assert(index < num_nodes_);
  return nodes_[index].Y(y_range_);
}

float CompactSpline::YCalculatedSlowly(const float x) const {
  const CompactSplineIndex index = IndexForX(x, 0);

  // Handle cases where `x` is outside the spline's domain.
  if (index == kBeforeSplineIndex) return StartY();
  if (index == kAfterSplineIndex) return EndY();

  // Create the cubic curve for `index` and evaluate it.
  const CubicCurve cubic(CreateCubicInit(index));
  const float cubic_x = x - NodeX(index);
  const float y = cubic.Evaluate(cubic_x);
  return y;
}

void CompactSpline::Ys(const float start_x, const float delta_x,
                       const int num_ys, float* ys) const {
  // Use the BulkSplineEvaluator even though we're only evaluating one spline.
  // Still faster, since it doesn't have to recreate the cubic for every x.
  BulkYs(this, 1, start_x, delta_x, num_ys, ys);
}

// static
void CompactSpline::BulkYs(const CompactSpline* const splines,
                           const int num_splines, const float start_x,
                           const float delta_x, const size_t num_ys,
                           float* ys) {
  BulkSplineEvaluator evaluator;

  // Initialize the evaluator with the splines.
  // Note that we set `repeat` = false, so that we can accurately get the last
  // value in the spline.
  const SplinePlayback playback(start_x);
  evaluator.SetNumIndices(num_splines);
  evaluator.SetSplines(0, num_splines, splines, playback);

  // Grab y values, then advance spline evaluation by delta_x.
  // Repeat num_ys times.
  const float* end_y = &ys[num_splines * num_ys];
  for (float* y = ys; y < end_y; y += num_splines) {
    memcpy(y, evaluator.Ys(0), num_splines * sizeof(y[0]));
    evaluator.AdvanceFrame(delta_x);
  }
}

Range CompactSpline::RangeX(const CompactSplineIndex index) const {
  if (index == kBeforeSplineIndex)
    // Return 0.0f for the start of the range instead of -inf.
    // There is an implicit range from the start of the spline (x=0) to the
    // start of the first segment.
    return Range(0.0f, StartX());

  if (index == kAfterSplineIndex)
    return Range(EndX(), std::numeric_limits<float>::infinity());

  return Range(nodes_[index].X(x_granularity_),
               nodes_[index + 1].X(x_granularity_));
}

CompactSplineIndex CompactSpline::IndexForX(
    const float x, const CompactSplineIndex guess_index) const {
  const int quantized_x = CompactSplineNode::QuantizeX(x, x_granularity_);

  // Check bounds first.
  // Return negative if before index 0.
  if (quantized_x < Front().x()) return kBeforeSplineIndex;

  // Return index of the last index if beyond the last index.
  if (quantized_x >= Back().x()) return kAfterSplineIndex;

  // Check the guess value first.
  const CompactSplineXGrain compact_x =
      static_cast<CompactSplineXGrain>(quantized_x);
  if (IndexContainsX(compact_x, guess_index)) return guess_index;

  // Search for it, if the initial guess fails.
  const CompactSplineIndex index = BinarySearchIndexForX(compact_x);
  assert(IndexContainsX(compact_x, index));
  return index;
}

CompactSplineIndex CompactSpline::IndexForXAllowingRepeat(
    const float x, const CompactSplineIndex guess_index,
    const bool repeat, float* final_x) const {
  // Does not repeat, so return the index as is.
  const CompactSplineIndex index = IndexForX(x, guess_index);
  if (!repeat || index != kAfterSplineIndex) {
    *final_x = x;
    return index;
  }

  // Repeats, so wrap `x` back to 0 and find the index again.
  const float repeat_x = x - EndX();
  const CompactSplineIndex repeat_index = IndexForX(repeat_x, 0);
  *final_x = repeat_x;
  return repeat_index;
}

CompactSplineIndex CompactSpline::ClampIndex(const CompactSplineIndex index,
                                             float* x) const {
  if (index == kBeforeSplineIndex) {
    *x = StartX();
    return 0;
  }
  if (index == kAfterSplineIndex) {
    *x = EndX();
    return LastNodeIndex();
  }
  assert(index < num_nodes_);
  return index;
}

bool CompactSpline::IndexContainsX(const CompactSplineXGrain compact_x,
                                   const CompactSplineIndex index) const {
  return index < LastNodeIndex() && nodes_[index].x() <= compact_x &&
         compact_x <= nodes_[index + 1].x();
}

static inline bool CompareSplineNodeX(const CompactSplineXGrain compact_x,
                                      const CompactSplineNode& n) {
  return compact_x < n.x();
}

CompactSplineIndex CompactSpline::BinarySearchIndexForX(
    const CompactSplineXGrain compact_x) const {
  // Binary search nodes by x.
  // TODO OPT: avoid the pointer arithmetic (which is expensive on ARM since it
  // requires an integer division) by searching with indices instead of
  // iterators.
  //     int low = 0;
  //     int hi = max_hi;
  //     while (low + 1 < hi) {
  //       const int mid = (low + hi) / 2;
  //
  //       if (compact_x < nodes_[mid].x()) {
  //         hi = mid;
  //       } else {
  //         low = mid;
  //       }
  //     }
  const auto upper_it = std::upper_bound(nodes_, &nodes_[num_nodes_], compact_x,
                                         CompareSplineNodeX);
  const int low = static_cast<int>(upper_it - nodes_) - 1;
  assert(0 <= low && low < LastNodeIndex());

  // We return the lower index: x is in the segment bt 'index' and 'index' + 1.
  return static_cast<CompactSplineIndex>(low);
}

CubicInit CompactSpline::CreateCubicInit(const CompactSplineIndex index) const {
  // Handle case where we are outside of the interpolatable range.
  if (OutsideSpline(index)) {
    const CompactSplineNode& n = index == kBeforeSplineIndex ? Front() : Back();
    const float constant_y = n.Y(y_range_);
    return CubicInit(constant_y, 0.0f, constant_y, 0.0f, 1.0f);
  }

  // Interpolate between the nodes at 'index' and 'index' + 1.
  return CreateCubicInit(nodes_[index], nodes_[index + 1]);
}

CubicInit CompactSpline::CreateCubicInit(const CompactSplineNode& s,
                                         const CompactSplineNode& e) const {
  return CubicInit(s.Y(y_range_), s.Derivative(), e.Y(y_range_), e.Derivative(),
                   WidthX(s, e));
}

// static
float CompactSpline::RecommendXGranularity(const float max_x) {
  return max_x <= 0.0f ? 1.0f : max_x / CompactSplineNode::MaxX();
}

}  // namespace motive
