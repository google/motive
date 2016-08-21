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

// YsBulkOutput records the evaluated y and derivative values into 2D arrays.
// Arrays are of length num_points * num_splines.
class YsBulkOutput : public CompactSpline::BulkOutput {
 public:
  YsBulkOutput(float* ys, float* derivatives, size_t num_splines)
      : ys(ys), derivatives(derivatives), num_splines(num_splines) {}

  /// Gets the Y and derivative values of the evaluator.
  virtual void AddPoint(int i, const BulkSplineEvaluator& evaluator) {
    assert(num_splines == static_cast<size_t>(evaluator.NumIndices()));

    const size_t offset = i * num_splines;
    float* y = ys + offset;
    memcpy(y, evaluator.Ys(0), num_splines * sizeof(y[0]));

    if (derivatives) {
      float* derivatives_for_i = derivatives + offset;
      for (size_t j = 0; j < num_splines; ++j) {
        derivatives_for_i[j] =
            evaluator.Derivative(static_cast<CompactSplineIndex>(j));
      }
    }
  }

 private:
  float* ys;
  float* derivatives;
  size_t num_splines;
};

void CompactSpline::AddNode(const float x, const float y,
                            const float derivative,
                            const CompactSplineAddMethod method) {
  const CompactSplineNode new_node(x, y, derivative, x_granularity_, y_range_);

  // Precondition: Nodes must come *after* or *at* the last node.
  assert(num_nodes_ == 0 || new_node.x() >= Back().x());

  // Early out when adding the same node.
  const bool same_as_back = num_nodes_ > 0 && Back() == new_node;
  if (same_as_back) return;

  // If we're adding a point at the same x, that means there will be a
  // discontinuity in the curve at x (either in y or derivative).
  const bool discontinuity = num_nodes_ > 0 && Back().x() == new_node.x();
  if (discontinuity) {
    // No point in having three points with the same x, value. Two points makes
    // a discontinuity, but for any more, the middle points will just take up
    // space, so remove it.
    const bool already_ends_in_discontinuity =
        num_nodes_ >= 2 && Back().x() == nodes_[num_nodes_ - 2].x();
    if (already_ends_in_discontinuity) num_nodes_--;
  }

  // Add a dual-cubic mid-node, if required, to keep cubic curves well behaved.
  const bool add_middle_node =
      !discontinuity && method == kEnsureCubicWellBehaved && num_nodes_ != 0;
  if (add_middle_node) {
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

float CompactSpline::CalculatedSlowly(const float x,
                                      const CurveValueType value_type) const {
  const CompactSplineIndex index = IndexForX(x, 0);

  // Handle cases where `x` is outside the spline's domain.
  if (index == kBeforeSplineIndex)
    // The curve is flat outside the bounds, so all derivatives
    // outside the bounds are 0.
    return value_type == kCurveValue ? StartY() : 0.0f;
  if (index == kAfterSplineIndex)
    return value_type == kCurveValue ? EndY() : 0.0f;

  // Create the cubic curve for `index` and evaluate it.
  const CubicCurve cubic(CreateCubicInit(index));
  const float cubic_x = x - NodeX(index);
  return CurveValue<CubicCurve>(cubic, cubic_x, value_type);
}

void CompactSpline::Ys(const float start_x, const float delta_x,
                       const size_t num_points, float* ys,
                       float* derivatives) const {
  // Use the BulkSplineEvaluator even though we're only evaluating one spline.
  // Still faster, since it doesn't have to recreate the cubic for every x.
  BulkYs(this, 1, start_x, delta_x, num_points, ys, derivatives);
}

void CompactSpline::BulkEvaluate(const CompactSpline* const splines,
                                 const size_t num_splines, const float start_x,
                                 const float delta_x, const size_t num_points,
                                 BulkOutput* out) {
  BulkSplineEvaluator evaluator;

  // Initialize the evaluator with the splines.
  // Note that we set `repeat` = false, so that we can accurately get the last
  // value in the spline.
  const SplinePlayback playback(start_x);
  evaluator.SetNumIndices(num_splines);
  evaluator.SetSplines(0, num_splines, splines, playback);

  // Grab y values, then advance spline evaluation by delta_x.
  // Repeat num_points times.
  for (CompactSplineIndex i = 0; i < num_points; ++i) {
    out->AddPoint(i, evaluator);
    evaluator.AdvanceFrame(delta_x);
  }
}

// static
void CompactSpline::BulkYs(const CompactSpline* const splines,
                           const size_t num_splines, const float start_x,
                           const float delta_x, const size_t num_points,
                           float* ys, float* derivatives) {
  YsBulkOutput output(ys, derivatives, num_splines);
  BulkEvaluate(splines, num_splines, start_x, delta_x, num_points, &output);
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
  if (quantized_x > Back().x()) return kAfterSplineIndex;

  // When we are exactly on the last node, we want to return the index of the
  // last segment (i.e. the second last node). This is so that the derivative
  // at the end matches the derivative of the last node, and not 0 (since
  // derivatives beyond the spline are forced to 0).
  if (quantized_x == Back().x()) return std::max(0, num_nodes_ - 2);

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
  const Range x_range(0.0f, EndX());
  const float repeat_x = x_range.NormalizeCloseValue(x);
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
