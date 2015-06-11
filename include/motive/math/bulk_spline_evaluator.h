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

#ifndef MOTIVE_MATH_BULK_SPLINE_EVALUATOR_H_
#define MOTIVE_MATH_BULK_SPLINE_EVALUATOR_H_

#include "motive/math/compact_spline.h"
#include "motive/util/optimizations.h"

namespace fpl {

/// @class BulkSplineEvaluator
/// @brief Traverse through a set of splines in a performant way.
///
/// This class should be used if you have hundreds or more splines that you need
/// to traverse in a uniform manner. It stores the spline data so that this
/// traversal is very fast, when done in bulk, and so we can take advantage of
/// SIMD on supported processors.
///
/// This class maintains a current `x` value for each spline, and a current
/// cubic-curve for the segment of the spline corresponding to that `x`.
/// In AdvanceFrame, the `x`s are incremented. If this increment pushes us to
/// the next segment of a spline, the cubic-curve is reinitialized to the next
/// segment of the spline. The splines are evaluated at the current `x` in bulk.
///
class BulkSplineEvaluator {
 public:
  typedef int Index;

  /// Increase or decrease the total number of indices processed.
  ///
  /// This class holds a set of splines, each is given an index
  /// from 0 to size - 1.
  ///
  /// The number of splines can be increased or decreased with SetNumIndices().
  ///   - splines are allocated or removed at the highest indices
  void SetNumIndices(const Index num_indices);

  /// Move the data at `old_index` into `new_index`.
  ///
  /// Unused indices are still processed every frame. You can fill these index
  /// holes with MoveIndex(), to move items from the last index into the hole.
  /// Once all holes have been moved to the highest indices, you can call
  /// SetNumIndices() to stop processing these highest indices. Note that this
  /// is exactly what fpl::IndexAllocator does. You should use that class to
  /// keep your indices contiguous.
  void MoveIndex(const Index old_index, const Index new_index);

  /// Initialize `index` to clamp or wrap-around the `valid_y` range.
  /// If `modular_arithmetic` is true, values wrap-around when they exceed
  /// the bounds of `valid_y`. If it is false, values are clamped to `valid_y`.
  void SetYRange(const Index index, const Range& valid_y,
                 const bool modular_arithmetic);

  /// Initialize `index` to process `s.spline` starting from `s.start_x`.
  /// The Y() and Derivative() values are immediately available.
  void SetSpline(const Index index, const SplinePlayback& s);

  /// Set conversion rate from AdvanceFrame's delta_x to the speed at which
  /// we traverse the spline.
  ///     0   ==> paused
  ///     0.5 ==> half speed (slow motion)
  ///     1   ==> authored speed
  ///     2   ==> double speed (fast forward)
  void SetPlaybackRate(const Index index, float playback_rate) {
    playback_rates_[index] = playback_rate;
  }

  /// Increment x and update the Y() and Derivative() values for all indices.
  /// Process all indices in bulk to efficiently traverse memory and allow SIMD
  /// instructions to be effective.
  void AdvanceFrame(const float delta_x);

  /// Return true if the spline for `index` has valid spline data.
  bool Valid(const Index index) const;

  /// Return the current x value for the spline at `index`.
  float X(const Index index) const {
    return CubicStartX(index) + cubic_xs_[index];
  }

  /// Return the current y value for the spline at `index`.
  float Y(const Index index) const { return ys_[index]; }

  /// Return the current y value for spline indices
  /// `index` ~ `index + count - 1`.
  void Ys(const Index index, const Index count, float* ys) const {
    assert(Valid(index) && Valid(index + count - 1));
    memcpy(ys, &ys_[index], sizeof(ys[0]) * count);
  }

  /// Return the current slope for the spline at `index`.
  float Derivative(const Index index) const {
    return Cubic(index).Derivative(cubic_xs_[index]);
  }

  /// Return the spline that is currently being traversed at `index`.
  const CompactSpline* SourceSpline(const Index index) const {
    return sources_[index].spline;
  }

  /// Return the raw cubic curve for `index`. Useful if you need to calculate
  /// the second or third derivatives (which are not calculated in
  /// AdvanceFrame), or plot the curve for debug reasons.
  const CubicCurve& Cubic(const Index index) const { return cubics_[index]; }

  /// Return the current x value for the current cubic. Each spline segment
  /// is evaluated as a cubic that starts at x=0.
  float CubicX(const Index index) const { return cubic_xs_[index]; }

  /// Return x-value at the end of the spline.
  float EndX(const Index index) const { return sources_[index].spline->EndX(); }

  /// Return y-value at the end of the spline.
  float EndY(const Index index) const { return sources_[index].spline->EndY(); }

  /// Return slope at the end of the spline.
  float EndDerivative(const Index index) const {
    return sources_[index].spline->EndDerivative();
  }

  /// Return y-distance between current-y and end-y.
  /// If using modular arithmetic, consider both paths to the target
  /// (directly and wrapping around), and return the length of the shorter path.
  float YDifferenceToEnd(const Index index) const {
    return NormalizeY(index, EndY(index) - Y(index));
  }

  /// Apply modular arithmetic to ensure that `y` is within the valid y_range.
  float NormalizeY(const Index index, const float y) const {
    const YRange& r = y_ranges_[index];
    return r.modular_arithmetic ? r.valid_y.Normalize(y) : y;
  }

  /// Helper function to calculate the next y-value in a series of y-values
  /// that are restricted by `direction`. There are always two paths that a y
  /// value can take, in modular arithmetic. This function chooses the correct
  /// one.
  float NextY(const Index index, const float current_y, const float target_y,
              const ModularDirection direction) const {
    const YRange& r = y_ranges_[index];
    if (!r.modular_arithmetic) return target_y;

    /// Calculate the difference from the current-y value for `direction`.
    const float diff = r.valid_y.ModDiff(current_y, target_y, direction);
    return current_y + diff;
  }

 private:
  void InitCubic(const Index index, const float start_x);
  Index NumIndices() const { return static_cast<Index>(sources_.size()); }
  float SplineStartX(const Index index) const {
    return sources_[index].spline->StartX();
  }
  float CubicStartX(const Index index) const {
    const Source& s = sources_[index];
    return s.spline->NodeX(s.x_index);
  }

  // These functions have C and assembly language variants.
  void UpdateCubicXsAndGetMask(const float delta_x, uint8_t* masks);
  void UpdateCubicXsAndGetMask_C(const float delta_x, uint8_t* masks);
  size_t UpdateCubicXs(const float delta_x, Index* indices_to_init);
  size_t UpdateCubicXs_TwoSteps(const float delta_x, Index* indices_to_init);
  size_t UpdateCubicXs_OneStep(const float delta_x, Index* indices_to_init);
  void EvaluateIndex(const Index index);
  void EvaluateCubics();
  void EvaluateCubics_C();

  struct Source {
    /// Pointer to the source spline node. Spline data is owned externally.
    /// We neither allocate or free this pointer here.
    const CompactSpline* spline;

    /// Current index into `spline`. The cubics_ valid is instantiated from
    /// spline[x_index].
    CompactSplineIndex x_index;

    /// If true, start again at the beginning of the spline when we reach
    /// the end.
    bool repeat;

    Source()
        : spline(nullptr), x_index(fpl::kInvalidSplineIndex), repeat(false) {}
  };

  struct YRange {
    YRange() : valid_y(Range::Full()), modular_arithmetic(0) {}

    /// Hold min and max values for the y result, or for the modular range.
    /// Modular ranges are used for things like angles, the wrap around from
    /// -pi to +pi.
    Range valid_y;

    /// True if y values wrap around when they exit the valid_y range.
    /// False if y values clamp to the edges of the valid_y range.
    /// Use a mask for `modular_arithmetic` so that it can be used in `select`
    /// instructions, in SIMD code.
    uint32_t modular_arithmetic;
  };

  // Data is organized in struct-of-arrays format to match the algorithm`s
  // consumption of the data.
  // - The algorithm that updates x values, and detects when we must transition
  //   to the next segment of the spline looks only at data in `cubic_xs_` and
  //   `cubic_x_ends_`.
  // - The algorithm that updates `ys_` looks only at the data in `cubic_xs_`,
  //   `cubics_`, and `y_ranges_`. It writes to `ys_`.
  // These vectors grow when SetNumIndices() is called, but they never shrink.
  // So, we`ll have a few reallocs (which are slow) until the highwater mark is
  // reached. Then the cost of reallocs disappears. In this way we have a
  // reasonable tradeoff between memory conservation and runtime performance.

  /// Source spline nodes and our current index into these splines.
  std::vector<Source> sources_;

  /// Define the valid output values. We can clamp to a range, or wrap around to
  /// a range using modular arithmetic (two modes of operation).
  std::vector<YRange> y_ranges_;

  /// The current `x` value at which `cubics_` are evaluated.
  ///   ys_[i] = cubics_[i].Evaluate(cubic_xs_[i])
  std::vector<float> cubic_xs_;

  /// The last valid x value in `cubics_`.
  std::vector<float> cubic_x_ends_;

  /// Speed at which time flows, relative to the spline's authored rate.
  ///     0   ==> paused
  ///     0.5 ==> half speed (slow motion)
  ///     1   ==> authored speed
  ///     2   ==> double speed (fast forward)
  std::vector<float> playback_rates_;

  /// Currently active segment of sources_.spline.
  /// Instantiated from
  /// sources_[i].spline->CreateInitCubic(sources_[i].x_index).
  std::vector<CubicCurve> cubics_;

  /// Value of the spline at `cubic_xs_`, normalized and clamped to be within
  /// `y_ranges_`. Evaluated in AdvanceFrame.
  std::vector<float> ys_;

  /// Stratch buffer used for internal calculations.
  std::vector<Index> scratch_;

  /// Call the specified optimized functions, when available, instead of the
  /// plain C++ functions. Note that we must perform this check at runtime,
  /// not compile time: some platforms may or may not support all the
  /// instructions used in the optimized functions. For example, some x86
  /// processors support SSE3, others also support SSSE3, and still others
  /// support neither. Therefore, x86 code always includes the C++ functions as
  /// a fallback, and chooses the best functions at runtime.
  ProcessorOptimization optimization_;
};

}  // namespace fpl

#endif  // MOTIVE_MATH_BULK_SPLINE_EVALUATOR_H_
