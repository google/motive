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

#ifndef MOTIVE_MATH_RANGE_H_
#define MOTIVE_MATH_RANGE_H_

#include <vector>
#include "mathfu/utilities.h"

namespace motive {

// If using modular arithmetic, there are two paths to the target: one that
// goes directly and one that wraps around. This enum represents differet ways
// to choose the path.
enum ModularDirection {
  kDirectionClosest,
  kDirectionFarthest,
  kDirectionPositive,
  kDirectionNegative,
  kDirectionDirect,
};

/// @class RangeT
/// @brief Represent an interval on a number line.
template <class T>
class RangeT {
 public:
  template <size_t kMaxLen>
  struct TArray {
    size_t len;
    T arr[kMaxLen];
  };

  template <size_t kMaxLen>
  struct RangeArray {
    size_t len;
    RangeT<T> arr[kMaxLen];
  };

  // By default, initialize to an invalid range.
  RangeT() : start_(static_cast<T>(1)), end_(static_cast<T>(0)) {}
  explicit RangeT(const T point) : start_(point), end_(point) {}
  RangeT(const T start, const T end) : start_(start), end_(end) {}

  /// A range is valid if it contains at least one number.
  bool Valid() const { return start_ <= end_; }

  /// Returns the mid-point of the range, rounded down for integers.
  /// Behavior is undefined for invalid regions.
  T Middle() const { return (start_ + end_) / static_cast<T>(2); }

  /// Returns the span of the range. Returns 0 when only one number in range.
  /// Behavior is undefined for invalid regions.
  T Length() const { return end_ - start_; }

  /// Returns `x` if it is within the range. Otherwise, returns start_ or end_,
  /// whichever is closer to `x`.
  /// Behavior is undefined for invalid regions.
  T Clamp(const T x) const { return mathfu::Clamp(x, start_, end_); }

  /// Clamp `x` so it is inside the start bound. Can save cycles if you
  /// already know that `x` is inside the end bound.
  T ClampAfterStart(const T x) const { return std::max(x, start_); }

  /// Clamp `x` so it is inside the end bound. Can save cycles if you
  /// already know that `x` is inside the start bound.
  T ClampBeforeEnd(const T x) const { return std::min(x, end_); }

  /// Returns distance outside of the range. If inside the range, returns 0.
  /// Behavior is undefined for invalid regions.
  T DistanceFrom(const T x) const { return fabs(x - Clamp(x)); }

  /// Lerps between the start and the end.
  /// 'percent' of 0 returns start. 'percent' of 1 returns end.
  /// Behavior is undefined for invalid regions.
  T Lerp(const float percent) const {
    return mathfu::Lerp(start_, end_, percent);
  }

  /// Returns percent 0~1, from start to end. *Not* clamped to 0~1.
  /// 0 ==> start;  1 ==> end;  0.5 ==> Middle();  -1 ==> start - Length()
  float Percent(const T x) const { return (x - start_) / Length(); }

  /// Returns percent 0~1, from start to end. Clamped to 0~1.
  /// 0 ==> start or earlier;  1 ==> end or later;  0.5 ==> Middle()
  T PercentClamped(const T x) const {
    return mathfu::Clamp(Percent(x), 0.0f, 1.0f);
  }

  /// Ensure `x` is within the valid constraint range, by subtracting or
  /// adding Length() to it.
  /// `x` must be within +-Length() of the range bounds. This is a reasonable
  /// restriction in most cases (such as after an arithmetic operation).
  /// For cases where `x` may be wildly outside the range, use
  /// NormalizeCloseValue() or NormalizeWildValue() instead.
  T Normalize(T x) const {
    const float normalized = x + ModularAdjustment(x);
    assert(ContainsExcludingStart(normalized));
    return normalized;
  }

  /// Ensure `x` is within the valid constraint range, by subtracting or
  /// adding Length() to it repeatedly.
  /// `x` must be within +-kMaxAdjustments * Length() of the range bounds
  /// for this function to be effective. If its greater, we end up calling
  /// NormalizeWildValue(), so you'd be better off calling NormalizeWildValue()
  /// from the beginning.
  /// This function is intended to be called in situations where `x` is almost
  /// always within one or two lengths of being normalized, so we don't want to
  /// incur the division cost of NormalizeWildValue(). It's still guaranteed
  /// to return a normalized value, however.
  T NormalizeCloseValue(T x) const {
    static const int kMaxAdjustments = 4;

    // Return without change if `x` is already normalized.
    const bool below = x <= start_;
    const bool above = x > end_;
    if (!below && !above) return x;

    // Each time through the loop, we'll adjust by one length closer to the
    // valid interval.
    const T length = Length();
    int num_adjustments = 0;

    if (below) {
      // Keep adding until we're in the range.
      do {
        x += length;
        if (num_adjustments++ > kMaxAdjustments) return NormalizeWildValue(x);
      } while (x <= start_);
    } else {
      // Keep subtracting until we're in the range.
      do {
        x -= length;
        if (num_adjustments++ > kMaxAdjustments) return NormalizeWildValue(x);
      } while (x > end_);
    }
    return x;
  }

  /// Ensure `x` is within the valid constraint range, by subtracting multiples
  /// of Length() from it until it is.
  /// `x` can be any value.
  T NormalizeWildValue(T x) const {
    // Use (expensive) division to determine how many lengths we are away from
    // the normalized range.
    const T length = Length();
    const T units = (x - start_) / length;
    const T whole_units = floor(units);

    // Subtract off those units to get something that (mathematically) should
    // be normalized. Due to Ting point error, it sometimes is slightly
    // outside the bounds, so we need to do a standard normalization afterwards.
    const T close = x - whole_units * length;
    const T normalized = close + ModularAdjustment(close);
    assert(ContainsExcludingStart(normalized));
    return normalized;
  }

  /// Returns:
  ///   Length() if `x` is below the valid range
  ///   -Length() if `x` is above the valid range
  ///   0 if `x` is within the valid range.
  T ModularAdjustment(T x) const {
    const T length = Length();
    const T adjustment = x <= start_ ? length : x > end_ ? -length : 0.0f;
    return adjustment;
  }

  /// In modular arithmetic, you can get from 'a' to 'b' by going directly, or
  /// by wrapping around.
  /// Return the closest difference from 'a' to 'b' under modular arithmetic.
  float ModDiffClose(T a, T b) const { return Normalize(b - a); }

  /// Return the farthest difference from 'a' to 'b' under modular arithmetic.
  float ModDiffFar(T a, T b) const {
    const float length = Length();
    const float close = ModDiffClose(a, b);
    return close >= 0.0f ? close - length : close + length;
  }

  /// Return the difference from 'a' to 'b' under modular arithmetic that is
  /// positive.
  float ModDiffPositive(T a, T b) const {
    const float length = Length();
    const float close = ModDiffClose(a, b);
    return close >= 0.0f ? close : close + length;
  }

  /// Return the difference from 'a' to 'b' under modular arithmetic that is
  /// negative.
  float ModDiffNegative(T a, T b) const {
    const float length = Length();
    const float close = ModDiffClose(a, b);
    return close >= 0.0f ? close - length : close;
  }

  /// Return the difference from 'a' to 'b' that satisfies the 'direction'
  /// criteria.
  float ModDiff(T a, T b, ModularDirection direction) const {
    switch (direction) {
      case kDirectionClosest:
        return ModDiffClose(a, b);
      case kDirectionFarthest:
        return ModDiffFar(a, b);
      case kDirectionPositive:
        return ModDiffPositive(a, b);
      case kDirectionNegative:
        return ModDiffNegative(a, b);
      case kDirectionDirect:
        return b - a;
    }
    assert(false);
    return 0.0f;
  }

  /// Return true if `x` is in [start_, end_], i.e. the **inclusive** range.
  bool Contains(const T x) const { return start_ <= x && x <= end_; }

  /// Return true if `x` is in (start_, end_], i.e. the range that includes the
  /// end bound but not the start bound.
  bool ContainsExcludingStart(const T x) const {
    return start_ < x && x <= end_;
  }

  /// Return true if `x` is in [start_, end_), i.e. the range that includes the
  /// start bound but not the end bound.
  bool ContainsExcludingEnd(const T x) const { return start_ <= x && x < end_; }

  /// Return true if `x` is in (start_, end_), i.e. the **exclusive** range.
  bool StrictlyContains(const T x) const { return start_ < x && x < end_; }

  /// Return true if `x` is in [start_ - tolerance, end_ + tolerance],
  /// where tolerance = Length() * percent.
  bool ContainsWithTolerance(const T x, const T percent) const {
    const float tolerance = Length() * percent;
    return start_ - tolerance <= x && x <= end_ + tolerance;
  }

  /// Swap start and end. When 'a' and 'b' don't overlap, if you invert the
  /// return value of Range::Intersect(a, b), you'll get the gap between
  /// 'a' and 'b'.
  RangeT Invert() const { return RangeT(end_, start_); }

  /// Returns a range that is 'percent' longer. If 'percent' is < 1.0, then
  /// returned range will actually be shorter.
  RangeT Lengthen(const float percent) const {
    const T extra = static_cast<T>(Length() * percent * 0.5f);
    return RangeT(start_ - extra, end_ + extra);
  }

  /// Returns the smallest range that contains both `x` and the range in
  /// `this`.
  RangeT Include(const T x) const {
    return RangeT(std::min<T>(start_, x), std::max<T>(end_, x));
  }

  /// Equality is strict. No epsilon checking here.
  bool operator==(const RangeT& rhs) const {
    return start_ == rhs.start_ && end_ == rhs.end_;
  }
  bool operator!=(const RangeT& rhs) const { return !operator==(rhs); }

  /// Scale by multiplying by a scalar.
  RangeT operator*(const float s) const { return RangeT(s * start_, s * end_); }

  /// Accessors.
  T start() const { return start_; }
  T end() const { return end_; }
  void set_start(const T start) { start_ = start; }
  void set_end(const T end) { end_ = end; }

  /// Return the overlap of 'a' and 'b', or an invalid range if they do not
  /// overlap at all.
  /// When 'a' and 'b' don't overlap at all, calling Invert on the returned
  /// range will give the gap between 'a' and 'b'.
  static RangeT Intersect(const RangeT& a, const RangeT& b) {
    // Possible cases:
    // 1.  |-a---|    |-b---|  ==>  return invalid
    // 2.  |-b---|    |-a---|  ==>  return invalid
    // 3.  |-a---------|       ==>  return b
    //        |-b---|
    // 4.  |-b---------|       ==>  return a
    //        |-a---|
    // 5.  |-a---|             ==>  return (b.start, a.end)
    //        |-b---|
    // 6.  |-b---|             ==>  return (a.start, b.end)
    //        |-a---|
    //
    // All satisfied by,
    //   intersection.start = max(a.start, b.start)
    //   intersection.end = min(a.end, b.end)
    // Note that ranges where start > end are considered invalid.
    return RangeT(std::max(a.start_, b.start_), std::min(a.end_, b.end_));
  }

  /// Return the smallest range that covers all of 'a' and 'b'.
  static RangeT Union(const RangeT& a, const RangeT& b) {
    // Possible cases:
    // 1.  |-a---|    |-b---|  ==>  return (a.start, b.end)
    // 2.  |-b---|    |-a---|  ==>  return (b.start, a.end)
    // 3.  |-a---------|       ==>  return a
    //        |-b---|
    // 4.  |-b---------|       ==>  return b
    //        |-a---|
    // 5.  |-a---|             ==>  return (a.start, b.end)
    //        |-b---|
    // 6.  |-b---|             ==>  return (b.start, a.end)
    //        |-a---|
    //
    // All satisfied by,
    //   intersection.start = min(a.start, b.start)
    //   intersection.end = max(a.end, b.end)
    return RangeT(std::min(a.start_, b.start_), std::max(a.end_, b.end_));
  }

  /// Only keep entries in 'values' if they are in
  /// (range.start - epsition, range.end + epsilon).
  /// Any values that are kept are clamped to 'range'.
  ///
  /// This function is useful when floating point precision error might put a
  /// value slightly outside 'range' even though mathematically it should be
  /// inside 'range'. This often happens with values right on the border of the
  /// valid range.
  static size_t ValuesInRange(const RangeT& range, T epsilon, size_t num_values,
                              T* values) {
    size_t num_returned = 0;
    for (size_t i = 0; i < num_values; ++i) {
      const T value = values[i];
      const T clamped = range.Clamp(value);
      const T dist = fabs(value - clamped);

      // If the distance from the range is small, keep the clamped value.
      if (dist <= epsilon) {
        values[num_returned++] = clamped;
      }
    }
    return num_returned;
  }

  template <size_t kMaxLen>
  static void ValuesInRange(const RangeT& range, T epsilon,
                            TArray<kMaxLen>* values) {
    values->len = ValuesInRange(range, epsilon, values->len, values->arr);
  }

  /// Intersect every element of 'a' with every element of 'b'. Append
  /// intersections to 'intersections'. Note that 'intersections' is not reset
  /// at the start of the call.
  static size_t IntersectRanges(const RangeT* a, size_t len_a, const RangeT* b,
                                size_t len_b, RangeT* intersections,
                                RangeT* gaps = nullptr,
                                size_t* len_gaps = nullptr) {
    size_t num_intersections = 0;
    size_t num_gaps = 0;

    for (size_t i = 0; i < len_a; ++i) {
      for (size_t j = 0; j < len_b; ++j) {
        const RangeT intersection = RangeT::Intersect(a[i], b[j]);
        if (intersection.Valid()) {
          intersections[num_intersections++] = intersection;

        } else if (gaps != nullptr) {
          // Return the gaps, too, if requested. Invert() invalid intersections
          // to get the gap between the ranges.
          gaps[num_gaps++] = intersection.Invert();
        }
      }
    }

    // Set return values.
    if (len_gaps != nullptr) {
      *len_gaps = num_gaps;
    }
    return num_intersections;
  }

  template <size_t kMaxLen>
  static void IntersectRanges(const RangeArray<kMaxLen>& a,
                              const RangeArray<kMaxLen>& b,
                              RangeArray<kMaxLen * kMaxLen>* intersections,
                              RangeArray<kMaxLen * kMaxLen>* gaps = nullptr) {
    const bool use_gaps = gaps != nullptr;
    intersections->len = IntersectRanges(
        a.arr, a.len, b.arr, b.len, intersections->arr,
        use_gaps ? gaps->arr : nullptr, use_gaps ? &gaps->len : nullptr);
  }

  /// Return the index of the longest range in `ranges`.
  static size_t IndexOfLongest(const RangeT* ranges, size_t len) {
    T longest_length = -1.0f;
    size_t longest_index = 0;
    for (size_t i = 0; i < len; ++i) {
      const T length = ranges[i].Length();
      if (length > longest_length) {
        longest_length = length;
        longest_index = i;
      }
    }
    return longest_index;
  }

  template <size_t kMaxLen>
  static size_t IndexOfLongest(const RangeArray<kMaxLen>& ranges) {
    return IndexOfLongest(ranges.arr, ranges.len);
  }

  /// Return the index of the shortest range in `ranges`.
  static size_t IndexOfShortest(const RangeT* ranges, size_t len) {
    T shortest_length = std::numeric_limits<T>::infinity();
    size_t shortest_index = 0;
    for (size_t i = 0; i < len; ++i) {
      const T length = ranges[i].Length();
      if (length < shortest_length) {
        shortest_length = length;
        shortest_index = i;
      }
    }
    return shortest_index;
  }

  template <size_t kMaxLen>
  static size_t IndexOfShortest(const RangeArray<kMaxLen>& ranges) {
    return IndexOfShortest(ranges.arr, ranges.len);
  }

  /// Return the index of the shortest range in `ranges`.
  static T ClampToClosest(T x, const RangeT* ranges, size_t len) {
    T closest_dist = std::numeric_limits<T>::infinity();
    T closest_clamp = x;
    for (size_t i = 0; i < len; ++i) {
      const T clamp = ranges[i].Clamp(x);
      const T dist = fabs(x - clamp);
      if (dist < closest_dist) {
        closest_dist = dist;
        closest_clamp = clamp;
      }
    }
    return closest_clamp;
  }

  template <size_t kMaxLen>
  static T ClampToClosest(T x, const RangeArray<kMaxLen>& ranges) {
    return ClampToClosest(x, ranges.arr, ranges.len);
  }

  /// Returns the range that covers all values in f(array).
  /// f is a lambda to calculate T from S. See Covers() for a simple example.
  /// In general, if your S has a function `T GetValue()`, then your lambda
  /// can look something like,
  ///     const RangeT<T> range = RangeT<T>::CoversLambda(
  ///         s_array, len, [](const S& s) { return s.GetValue(); });
  template <typename S, typename F>
  static RangeT<T> CoversLambda(const S* array, size_t len, const F& f) {
    RangeT<T> r = Empty();
    for (size_t i = 0; i < len; ++i) {
      r = r.Include(f(array[i]));
    }
    return r;
  }

  /// Return the range that covers all values in `array`.
  static RangeT<T> Covers(const T* array, size_t len) {
    return CoversLambda(array, len, [](const T& t) { return t; });
  }

  /// Returns the complete range. Every T is contained in this range.
  static RangeT<T> Full() {
    return RangeT<T>(-std::numeric_limits<T>::infinity(),
                     std::numeric_limits<T>::infinity());
  }

  /// Returns the most empty range possible. The lower bound is
  /// greater than everything, and the upper bound is less than
  /// everything. Useful when finding the min/max values of an
  /// array of numbers.
  static RangeT<T> Empty() {
    return RangeT<T>(std::numeric_limits<T>::infinity(),
                     -std::numeric_limits<T>::infinity());
  }

  /// Returns the range of positive numbers: [0, +infinity].
  static RangeT<T> Positive() {
    return RangeT<T>(0.0f, std::numeric_limits<T>::infinity());
  }

  /// Returns the range of negative numbers.: [-infinity, 0].
  static RangeT<T> Negative() {
    return RangeT<T>(-std::numeric_limits<T>::infinity(), 0.0f);
  }

 private:
  T start_;  // Start of the range. Range is valid if start_ <= end_.
  T end_;    // End of the range. Range is inclusive of start_ and end_.
};

/// Given two numbers, create a range that has the lower one as min,
/// and the higher one as max.
template <class T>
RangeT<T> CreateValidRange(const T a, const T b) {
  return RangeT<T>(std::min<T>(a, b), std::max<T>(a, b));
}

// Instantiate for various scalars.
typedef RangeT<float> RangeFloat;
typedef RangeT<double> RangeDouble;
typedef RangeT<int> RangeInt;
typedef RangeT<unsigned int> RangeUInt;

// Since the float specialization will be most common, we give it a simple name.
typedef RangeFloat Range;

// Useful constants.
static const Range kAngleRange(-static_cast<float>(M_PI),
                               static_cast<float>(M_PI));
static const Range kInvalidRange;

}  // namespace motive

#endif  // MOTIVE_MATH_RANGE_H_
