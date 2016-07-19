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

#ifndef MOTIVE_MATH_COMPACT_SPLINE_H_
#define MOTIVE_MATH_COMPACT_SPLINE_H_

#include "motive/common.h"
#include "motive/math/compact_spline_node.h"
#include "motive/math/curve.h"

namespace motive {

/// @typedef CompactSplineIndex
/// Index into the spline. Some high values have special meaning (see below).
typedef uint16_t CompactSplineIndex;
static const CompactSplineIndex kInvalidSplineIndex =
    static_cast<CompactSplineIndex>(-1);
static const CompactSplineIndex kBeforeSplineIndex =
    static_cast<CompactSplineIndex>(-2);
static const CompactSplineIndex kAfterSplineIndex =
    static_cast<CompactSplineIndex>(-3);

/// Return true if `index` is not an index into the spline.
inline bool OutsideSpline(CompactSplineIndex index) {
  return index >= kAfterSplineIndex;
}

enum CompactSplineAddMethod {
  kAddWithoutModification,  /// Add node straight-up. No changes.
  kEnsureCubicWellBehaved,  /// Insert an intermediate node, if required,
                            /// to ensure cubic splines have uniform curvature.
};

/// @class CompactSpline
/// @brief Represent a smooth curve in a small amount of memory.
///
/// This spline interpolates a series of (x, y, derivative) nodes to create a
/// smooth curve.
///
/// This class holds a series of such nodes, and aids with the construction of
/// that series by inserting extra nodes when extra smoothness is required.
///
/// The data in this class is compacted as quantized values. It's not intended
/// to be read directly. You should use the BulkSplineEvaluator to update
/// and read values from the splines in a performant manner.
class CompactSpline {
 public:
  /// When a `CompactSpline` is created on the stack, it will have this many
  /// nodes. This amount is sufficient for the vast majority of cases where
  /// you are procedurally generating a spline. We used a fixed number instead
  /// of an `std::vector` to avoid dynamic memory allocation.
  static const CompactSplineIndex kDefaultMaxNodes = 7;

  CompactSpline()
      : x_granularity_(0.0f), num_nodes_(0), max_nodes_(kDefaultMaxNodes) {}
  CompactSpline(const Range& y_range, const float x_granularity)
      : max_nodes_(kDefaultMaxNodes) {
    Init(y_range, x_granularity);
  }
  CompactSpline(const CompactSpline& rhs) : max_nodes_(kDefaultMaxNodes) {
    *this = rhs;
  }
  CompactSpline& operator=(const CompactSpline& rhs) {
    assert(rhs.num_nodes_ <= max_nodes_);
    y_range_ = rhs.y_range_;
    x_granularity_ = rhs.x_granularity_;
    num_nodes_ = rhs.num_nodes_;
    memcpy(nodes_, rhs.nodes_, rhs.num_nodes_ * sizeof(nodes_[0]));
    return *this;
  }

  /// The range of values for x and y must be specified at spline creation time
  /// and cannot be changed afterwards. Empties all nodes, if we have any.
  ///
  /// @param y_range The upper and lower bounds for y-values in the nodes.
  ///                The more narrow this is, the better the precision of the
  ///                fixed point numbers. Note that you should add 10% padding
  ///                here, since AddNode may insert a smoothing node that is
  ///                slightly beyond the source y range.
  /// @param x_granularity The minimum increment of x-values. If you're working
  ///                      with a spline changes at most 30 times per second,
  ///                      and your x is in units of 1/1000th of a second, then
  ///                      x_granularity = 33 is a good baseline. You'll
  ///                      probably want granularity around 1/50th of that
  ///                      baseline value, though, since AddNode may insert
  ///                      smoothing nodes at intermediate x's.
  ///                      In our example here, you could set
  ///                      x_granularity near 33 / 50. For ease of debugging,
  ///                      an x_granularity of 0.5 or 1 is probably best.
  void Init(const Range& y_range, const float x_granularity) {
    num_nodes_ = 0;
    y_range_ = y_range;
    x_granularity_ = x_granularity;
  }

  /// Add a node to the end of the spline. Depending on the method, an
  /// intermediate node may also be inserted.
  ///
  /// @param x Must be greater than the x-value of the last spline node. If not,
  ///          this call is a nop.
  /// @param y Must be within the `y_range` specified in Init().
  /// @param derivative No restrictions, but excessively large values may still
  ///                   result in overshoot, even with an intermediate node.
  /// @param method If kAddWithoutModification, adds the node and does nothing
  ///               else. If kEnsureCubicWellBehaved, adds the node and
  ///               (if required) inserts another node in the middle so that
  ///               the individual cubics have uniform curvature.
  ///               Uniform curvature means always curving upward or always
  ///               curving downward. See docs/dual_cubics.pdf for details.
  void AddNode(const float x, const float y, const float derivative,
               const CompactSplineAddMethod method = kEnsureCubicWellBehaved);

  /// Add values without converting them. Useful when initializing from
  /// precalculated data.
  void AddNodeVerbatim(const CompactSplineXGrain x, const CompactSplineYRung y,
                       const CompactSplineAngle angle) {
    AddNodeVerbatim(detail::CompactSplineNode(x, y, angle));
  }

  /// Indicate that we have stopped adding nodes and want to release the
  /// remaining memory. Useful for when we have one giant buffer from which
  /// we want to add many splines of (potentially unknown) various sizes.
  /// We can do something like,
  ///  \code{.cpp}
  ///  size_t CreateSplines(char* memory_buffer, size_t memory_buffer_size) {
  ///    char* buf = memory_buffer;
  ///    const char* end = memory_buffer + memory_buffer_size;
  ///
  ///    while (MoreSplinesToCreate()) {
  ///      // Allocate a spline that can hold as many nodes as buf can hold.
  ///      CompactSpline* spline =
  ///          CompactSpline::CreateInPlaceMaxNodes(buf, end - buf);
  ///
  ///      while (MoreNodesToAdd()) {
  ///        // Ensure we haven't reached the end of the buffer.
  ///        if (spline->num_splines() == spline->max_splines()) break;
  ///
  ///        // ... spline creation logic ...
  ///        spline->AddNode(...);
  ///      }
  ///
  ///      // Shrink `spline` to be the size that it actually is.
  ///      spline->Finalize();
  ///
  ///      // Advance pointer so next spline starts where this one ends.
  ///      buf += spline->Size();
  ///    }
  ///
  ///    // Return the total bytes consumed from `memory_buffer`.
  ///    return end - buf;
  ///  }
  ///  \endcode
  void Finalize() {
    max_nodes_ = num_nodes_;
  }

  /// Remove all nodes from the spline.
  void Clear() { num_nodes_ = 0; }

  /// Returns the memory occupied by this spline.
  size_t Size() const { return Size(max_nodes_); }

  /// Use on an array of splines created by CreateArrayInPlace().
  /// Returns the next spline in the array.
  CompactSpline* Next() { return NextAtIdx(1); }
  const CompactSpline* Next() const { return NextAtIdx(1); }

  /// Use on an array of splines created by CreateArrayInPlace().
  /// Returns the idx'th spline in the array.
  CompactSpline* NextAtIdx(int idx) {
    // Use union to avoid potential aliasing bugs.
    union {
      CompactSpline* spline;
      uint8_t* ptr;
    } p;
    p.spline = this;
    p.ptr += idx * Size();
    return p.spline;
  }
  const CompactSpline* NextAtIdx(int idx) const {
    return const_cast<CompactSpline*>(this)->NextAtIdx(idx);
  }

  /// Return index of the first node before `x`.
  /// If `x` is before the first node, return kBeforeSplineIndex.
  /// If `x` is past the last node, return kAfterSplineIndex.
  ///
  /// @param x x-value in the spline. Most often, the x-axis represents time.
  /// @param guess_index Best guess at what the index for `x` will be.
  ///                    Often the caller will be traversing from low to high x,
  ///                    so a good guess is the index after the current index.
  ///                    If you have no idea, set to 0.
  CompactSplineIndex IndexForX(const float x,
                               const CompactSplineIndex guess_index) const;

  /// If `repeat` is true, loop to x = 0 when `x` >= EndX().
  /// If `repeat` is false, same as IndexForX().
  CompactSplineIndex IndexForXAllowingRepeat(
      const float x, const CompactSplineIndex guess_index,
      const bool repeat, float* final_x) const;

  /// Returns closest index between 0 and NumNodes() - 1.
  /// Clamps `x` to a value in the range of index.
  /// `index` must be a valid value: i.e. kBeforeSplineIndex, kAfterSplineIndex,
  ///  or between 0..NumNodes()-1.
  CompactSplineIndex ClampIndex(const CompactSplineIndex index, float* x) const;

  // First and last x, y, and derivatives in the spline.
  float StartX() const { return Front().X(x_granularity_); }
  float StartY() const { return Front().Y(y_range_); }
  float StartDerivative() const { return nodes_[0].Derivative(); }

  float EndX() const { return Back().X(x_granularity_); }
  float EndY() const { return Back().Y(y_range_); }
  float EndDerivative() const { return Back().Derivative(); }
  float NodeX(const CompactSplineIndex index) const;
  float NodeY(const CompactSplineIndex index) const;
  float NodeDerivative(const CompactSplineIndex index) const {
    assert(index < num_nodes_);
    return nodes_[index].Derivative();
  }
  float LengthX() const { return EndX() - StartX(); }
  Range RangeX() const { return Range(StartX(), EndX()); }
  const Range& RangeY() const { return y_range_; }

  /// Evaluate spline at `x`. This function is somewhat slow because it
  /// must find the node for `x` and create the cubic before the returned
  /// y can be evaluated.
  /// If calling from inside a loop, replace the loop with one call to Ys(),
  /// which is significantly faster.
  float YCalculatedSlowly(const float x) const;

  /// Fast evaluation of a subset of the x-domain of the spline.
  /// Spline is evaluated from `start_x` and subsequent intervals of `delta_x`.
  /// Evaluated values are returned in `ys`.
  void Ys(const float start_x, const float delta_x, const int num_ys,
          float* ys) const;

  /// The start and end x-values covered by the segment after `index`.
  Range RangeX(const CompactSplineIndex index) const;

  /// Initialization parameters for a cubic curve that starts at `index` and
  /// ends at `index` + 1. Or a constant curve if `index` is kBeforeSplineIndex
  /// or kAfterSplineIndex.
  CubicInit CreateCubicInit(const CompactSplineIndex index) const;

  /// Returns the number of nodes in this spline.
  CompactSplineIndex num_nodes() const { return num_nodes_; }
  CompactSplineIndex max_nodes() const { return max_nodes_; }

  /// Return const versions of internal values. For serialization.
  const detail::CompactSplineNode* nodes() const { return nodes_; }
  const Range& y_range() const { return y_range_; }
  float x_granularity() const { return x_granularity_; }

  /// @param buffer chunk of memory of size CompactSpline::Size(max_nodes)
  static CompactSpline* CreateInPlace(CompactSplineIndex max_nodes,
                                      void* buffer) {
    CompactSpline* spline = new (buffer) CompactSpline();
    spline->max_nodes_ = max_nodes;
    return spline;
  }

  /// Returns `num_splines` placed contiguous in memory.
  /// Each spline is the same size. Access the next Spline with Next().
  /// @param buffer chuck of memory of size
  ///               CompactSpline::Size(max_nodes) * num_splines
  static CompactSpline* CreateArrayInPlace(CompactSplineIndex max_nodes,
                                           int num_splines, void* buffer) {
    const size_t size = Size(max_nodes);
    uint8_t* b = reinterpret_cast<uint8_t*>(buffer);
    for (int i = 0; i < num_splines; ++i) {
      CreateInPlace(max_nodes, b);
      b += size;
    }
    return reinterpret_cast<CompactSpline*>(buffer);
  }

  /// Allocate memory for a spline using global `new`.
  /// @param max_nodes The maximum number of nodes that this spline class
  ///                  can hold. Memory is allocated so that these nodes are
  ///                  held contiguously in memory with the rest of the
  ///                  class.
  static CompactSpline* Create(CompactSplineIndex max_nodes) {
    uint8_t* buffer = new uint8_t[Size(max_nodes)];
    return CreateInPlace(max_nodes, buffer);
  }

  /// Deallocate the splines memory using global `delete`.
  /// Be sure to call this for every spline returned from Create().
  static void Destroy(CompactSpline* spline) {
    if (spline == nullptr) return;
    // By design, spline does not have a destructor.
    delete[] reinterpret_cast<uint8_t*>(spline);
  }

  /// Allocate an array of splines, contiguous in memory, each of which can
  /// hold up to `max_nodes`. Use the global `new` operator to allocate the
  /// memory buffer.
  ///
  /// This function is useful when passing several-dimensions-worth of splines
  /// to MotivatorNf::SetSplines(), for example Motivator3f::SetSplines() takes
  /// an array of three splines, like this function returns.
  static CompactSpline* CreateArray(CompactSplineIndex max_nodes,
                                    int num_splines) {
    uint8_t* buffer = new uint8_t[Size(max_nodes) * num_splines];
    return CreateArrayInPlace(max_nodes, num_splines, buffer);
  }

  /// Frees the memory allocated with CreateArray() using global `delete`.
  static void DestroyArray(CompactSpline* splines, int /*num_splines*/) {
    if (splines == nullptr) return;
    // By design, spline does not have a destructor.
    delete[] reinterpret_cast<uint8_t*>(splines);
  }

  /// Returns the size, in bytes, of a CompactSpline class with `max_nodes`
  /// nodes.
  ///
  /// This function is useful when you want to provide your own memory buffer
  /// for splines, and then pass that buffer into CreateInPlace(). Your memory
  /// buffer must be at least Size().
  static size_t Size(CompactSplineIndex max_nodes) {
    // Total size of the class must be rounded up to the nearest alignment
    // so that arrays of the class are properly aligned.
    // Largest type in the class is a float.
    const size_t kAlignMask = sizeof(float) - 1;
    const size_t size =
        kBaseSize + max_nodes * sizeof(detail::CompactSplineNode);
    const size_t aligned = (size + kAlignMask) & ~kAlignMask;
    return aligned;
  }

  /// Returns the size, in bytes, of an array of CompactSplines (as allocated
  /// with CreateArray(), say).
  ///
  /// This function is useful when allocating a buffer for splines on your own,
  /// from which you can then call CreateArrayInPlace().
  static size_t ArraySize(size_t num_splines, size_t num_nodes) {
    return num_splines * kBaseSize +
           num_nodes * sizeof(detail::CompactSplineNode);
  }

  /// Recommend a granularity given a maximal-x value. We want to have the
  /// most precise granularity when quantizing x's.
  static float RecommendXGranularity(const float max_x);

  /// Fast evaluation of several splines.
  /// @param splines input splines of length `num_splines`.
  /// @param num_splines number of splines to evaluate.
  /// @param start_x starting point for every spline.
  /// @param delta_x increment for each output y.
  /// @param num_ys length of the `ys` array.
  /// @param ys two dimensional output array, ys[num_ys][num_splines].
  ///           ys[0] are `splines` evaluated at start_x.
  ///           ys[num_ys - 1] are `splines` evaluated at
  ///           start_x + delta_x * num_ys.
  static void BulkYs(const CompactSpline* const splines, const int num_splines,
                     const float start_x, const float delta_x,
                     const size_t num_ys, float* ys);

  /// Fast evaluation of several splines, with mathfu::VectorPacked interface.
  /// Useful for evaluate three splines which together form a mathfu::vec3,
  /// for instance.
  template <int kDimensions>
  static void BulkYs(const CompactSpline* const splines, const float start_x,
                     const float delta_x, const size_t num_ys,
                     mathfu::VectorPacked<float, kDimensions>* ys) {
    BulkYs(splines, kDimensions, start_x, delta_x, num_ys,
           reinterpret_cast<float*>(ys));
  }

 private:
  static const size_t kBaseSize;

  /// All other AddNode() functions end up calling this one.
  void AddNodeVerbatim(const detail::CompactSplineNode& node) {
    assert(num_nodes_ < max_nodes_);
    nodes_[num_nodes_++] = node;
  }

  /// Returns the index of the last node in the spline.
  CompactSplineIndex LastNodeIndex() const { return num_nodes_ - 1; }

  /// Return true iff `x` is between the the nodes at `index` and `index` + 1.
  bool IndexContainsX(const CompactSplineXGrain compact_x,
                      const CompactSplineIndex index) const;

  /// Search the nodes to find the index of the first node before `x`.
  CompactSplineIndex BinarySearchIndexForX(
      const CompactSplineXGrain compact_x) const;

  /// Return e.x - s.x, converted from quantized to external units.
  float WidthX(const detail::CompactSplineNode& s,
               const detail::CompactSplineNode& e) const {
    return (e.x() - s.x()) * x_granularity_;
  }

  /// Create the initialization parameters for a cubic running from `s` to `e`.
  CubicInit CreateCubicInit(const detail::CompactSplineNode& s,
                            const detail::CompactSplineNode& e) const;

  const detail::CompactSplineNode& Front() const {
    assert(num_nodes_ > 0);
    return nodes_[0];
  }

  const detail::CompactSplineNode& Back() const {
    assert(num_nodes_ > 0);
    return nodes_[num_nodes_ - 1];
  }

  /// Extreme values for y. See comments on Init() for details.
  Range y_range_;

  /// Minimum increment for x. See comments on Init() for details.
  float x_granularity_;

  /// Length of the `nodes_` array.
  CompactSplineIndex num_nodes_;

  /// Maximum length of the `nodes_` array. This may be different from
  /// `kDefaultMaxNodes` if CreateInPlace() was called.
  CompactSplineIndex max_nodes_;

  /// Array of key points (x, y, derivative) that describe the curve.
  /// The curve is interpolated smoothly between these key points.
  /// Key points are stored in quantized form, and converted back to world
  /// co-ordinates by using `y_range_` and `x_granularity_`.
  /// Note: This array can be longer or shorter than kDefaultMaxNodes if
  ///       the class was created with CreateInPlace(). The actual length of
  ///       this array is stored in max_nodes_.
  detail::CompactSplineNode nodes_[kDefaultMaxNodes];
};

/// @class SplinePlayback
/// @brief Parameters to specify how a spline should be traversed.
struct SplinePlayback {
  /// Initialize all channels with same spline.
  /// Especially useful when kDimensions = 1, since there is only one channel.
  explicit SplinePlayback(float start_x = 0.0f, bool repeat = false,
                          float playback_rate = 1.0f, float blend_x = 0.0f)
      : start_x(start_x), blend_x(blend_x), playback_rate(playback_rate),
        repeat(repeat) {}

  /// The starting point from which to play.
  float start_x;

  /// The point at which to be 100% in this spline. We create a smooth spline
  /// from the current state to the spline state that lasts for `blend_x`.
  float blend_x;

  /// The playback rate of the spline. Scales `delta_time` of the update to
  /// to x-axis of `splines`.
  ///     0   ==> paused
  ///     0.5 ==> half speed (slow motion)
  ///     1   ==> authored speed
  ///     2   ==> double speed (fast forward)
  float playback_rate;

  /// If true, start back at the beginning after we reach the end.
  bool repeat;
};

struct SplineState {
  float y;
  float derivative;

  SplineState() : y(0.0f), derivative(0.0f) {}
  SplineState(float y, float derivative) : y(y), derivative(derivative) {}
};

struct SplineBlend {
  SplineState current;
  const CompactSpline* spline;

  SplineBlend() : spline(nullptr) {}
  SplineBlend(const SplineState& current, const CompactSpline& spline)
      : current(current), spline(&spline) {}
};

}  // namespace motive

#endif  // MOTIVE_MATH_COMPACT_SPLINE_H_
