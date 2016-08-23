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

#ifndef MOTIVE_MATH_CURVE_H_
#define MOTIVE_MATH_CURVE_H_

#include <string>
#include <cstring> // Required for memset, memcpy on Linux
#include "motive/common.h"
#include "motive/math/range.h"

namespace motive {

enum CurveValueType {
  kCurveValue,
  kCurveDerivative,
  kCurveSecondDerivative,
  kCurveThirdDerivative,
};

static const int kDefaultGraphWidth = 80;
static const int kDefaultGraphHeight = 30;
static const mathfu::vec2i kDefaultGraphSize(kDefaultGraphWidth,
                                             kDefaultGraphHeight);

/// 2^22 = the max precision of significand.
static const float kEpsilonPrecision = static_cast<float>(1 << 22);
static const float kEpsilonScale = 1.0f / kEpsilonPrecision;

/// @class QuadraticInitWithStartDerivative
/// @brief Initialization parameters to create a quaternion with
///        start and end values, and start derivative.
/// Start is x = 0. End is x = 1.
struct QuadraticInitWithStartDerivative {
  QuadraticInitWithStartDerivative(const float start_y,
                                   const float start_derivative,
                                   const float end_y)
      : start_y(start_y), start_derivative(start_derivative), end_y(end_y) {}

  float start_y;
  float start_derivative;
  float end_y;
};

/// @class QuadraticInitWithOrigin
/// @brief Initialization parameters to create a quaternion with
///        values and derivatives at x=0.
struct QuadraticInitWithOrigin {
  QuadraticInitWithOrigin(const float y, const float derivative,
                          const float second_derivative)
      : y(y), derivative(derivative), second_derivative(second_derivative) {}

  float y;
  float derivative;
  float second_derivative;
};

/// @class QuadraticInitWithPoint
/// @brief Initialization parameters to create a quaternion with
///        values and derivatives at a specified x.
struct QuadraticInitWithPoint {
  QuadraticInitWithPoint(const float x, const float y_at_x,
                         const float derivative_at_x,
                         const float second_derivative)
      : x(x),
        y_at_x(y_at_x),
        derivative_at_x(derivative_at_x),
        second_derivative(second_derivative) {}

  float x;
  float y_at_x;
  float derivative_at_x;
  float second_derivative;
};

/// @class QuadraticCurve
/// @brief Represent a quadratic polynomial in the form
///        c_[2] * x^2  +  c_[1] * x  +  c_[0]
class QuadraticCurve {
 public:
  static const int kNumCoeff = 3;
  typedef Range::TArray<2> RootsArray;
  typedef Range::RangeArray<2> RangeArray;

  QuadraticCurve() { memset(c_, 0, sizeof(c_)); }
  QuadraticCurve(const float c2, const float c1, const float c0) {
    c_[2] = c2;
    c_[1] = c1;
    c_[0] = c0;
  }
  QuadraticCurve(const float* c) { memcpy(c_, c, sizeof(c_)); }
  QuadraticCurve(const QuadraticCurve& q, const float y_scale) {
    for (int i = 0; i < kNumCoeff; ++i) {
      c_[i] = y_scale * q.c_[i];
    }
  }
  QuadraticCurve(const QuadraticInitWithStartDerivative& init) { Init(init); }
  QuadraticCurve(const QuadraticInitWithOrigin& init) { Init(init); }
  QuadraticCurve(const QuadraticInitWithPoint& init) { Init(init); }
  void Init(const QuadraticInitWithStartDerivative& init);
  void Init(const QuadraticInitWithOrigin& init);
  void Init(const QuadraticInitWithPoint& init);

  /// Add the coefficients of another quadratic curve to this one.
  QuadraticCurve& operator+=(const QuadraticCurve& rhs) {
    for (int i = 0; i < kNumCoeff; ++i) {
      c_[i] += rhs.c_[i];
    }
    return *this;
  }

  /// Subtract the coefficients of another quadratic curve from this one.
  QuadraticCurve& operator-=(const QuadraticCurve& rhs) {
    for (int i = 0; i < kNumCoeff; ++i) {
      c_[i] -= rhs.c_[i];
    }
    return *this;
  }

  /// Shift the curve along the x-axis: x_shift to the left.
  /// That is x_shift becomes the curve's x=0.
  void ShiftLeft(const float x_shift);

  /// Shift the curve along the x-axis: x_shift to the right.
  void ShiftRight(const float x_shift) { ShiftLeft(-x_shift); }

  /// Shift the curve along the y-axis by y_offset: y_offset up the y-axis.
  void ShiftUp(const float y_offset) { c_[0] += y_offset; }

  /// Scale the curve along the y-axis by a factor of y_scale.
  void ScaleUp(const float y_scale) {
    for (int i = 0; i < kNumCoeff; ++i) {
      c_[i] *= y_scale;
    }
  }

  /// Return the quadratic function's value at `x`.
  /// f(x) = c2*x^2 + c1*x + c0
  float Evaluate(const float x) const {
    return (c_[2] * x + c_[1]) * x + c_[0];
  }

  /// Return the quadratic function's slope at `x`.
  /// f'(x) = 2*c2*x + c1
  float Derivative(const float x) const { return 2.0f * c_[2] * x + c_[1]; }

  /// Return the quadratic function's constant second derivative.
  /// f''(x) = 2*c2
  float SecondDerivative() const { return 2.0f * c_[2]; }

  /// Return the quadratic function's constant second derivative.
  /// Even though `x` is unused, we pass it in for consistency with other
  /// curve classes.
  float SecondDerivative(const float /*x*/) const { return SecondDerivative(); }

  /// Return the quadratic function's constant third derivative: 0.
  /// Even though `x` is unused, we pass it in for consistency with other
  /// curve classes.
  /// f'''(x) = 0
  float ThirdDerivative(const float x) const {
    (void)x;
    return 0.0f;
  }

  /// Returns a value below which floating point precision is unreliable.
  /// If we're testing for zero, for instance, we should test against this
  /// value. Pass in the x-range as well, in case that coefficient dominates
  /// the others.
  float EpsilonInRange(const float max_x) const {
    return Epsilon(std::max(std::fabs(max_x), MaxCoeff()));
  }

  /// Returns a value below which floating point precision is unreliable.
  /// If we're testing for zero, for instance, we should test against this
  /// value. We don't consider the x-range here, so this value is useful only
  /// when dealing with the equation itself.
  float EpsilonOfCoefficients() const { return Epsilon(MaxCoeff()); }

  /// Given values in the range of `x`, returns a value below which should be
  /// considered zero.
  float Epsilon(const float x) const {
    return x * kEpsilonScale;
  }

  /// Returns the largest absolute value of the coefficients. Gives a sense
  /// of the scale of the quaternion. If all the coefficients are tiny or
  /// huge, we'll need to renormalize around 1, since we take reciprocals of
  /// the coefficients, and floating point precision is poor for floats.
  float MaxCoeff() const {
    using std::max;
    const QuadraticCurve abs = AbsCoeff();
    return max(max(abs.c_[2], abs.c_[1]), abs.c_[0]);
  }

  /// Used for finding roots, and more.
  /// See http://en.wikipedia.org/wiki/Discriminant
  float Discriminant() const { return c_[1] * c_[1] - 4.0f * c_[2] * c_[0]; }

  /// When Discriminant() is close to zero, set to zero.
  /// Often floating point precision problems can make the discriminant
  /// very slightly non-zero, even though mathematically it should be zero.
  float ReliableDiscriminant(const float epsilon) const;

  /// Return the x at which the derivative is zero.
  float CriticalPoint() const {
    assert(std::fabs(c_[2]) >= EpsilonOfCoefficients());

    /// 0 = f'(x) = 2*c2*x + c1  ==>  x = -c1 / 2c2
    return -(c_[1] / c_[2]) * 0.5f;
  }

  /// Calculate the x-coordinates where this quadratic is zero,
  /// and put them into `roots`, in ascending order.
  /// Returns the 0, 1, or 2, the number of unique values put into `roots`.
  void Roots(RootsArray* roots) const { roots->len = Roots(roots->arr); }

  /// Same as Roots() but do not normalize the quadratic for precision.
  /// This function is not recommended, in general. Normalizing does not take
  /// much time, and floating point precision is something that we only like to
  /// think we can ignore. It may be safe to call this function if you are
  /// certain that your x^2 coefficient is near 1.
  void RootsWithoutNormalizing(RootsArray* roots) const {
    roots->len = RootsWithoutNormalizing(roots->arr);
  }

  /// Calculate the x-coordinates within the range [start_x, end_x] where the
  /// quadratic is zero. Put them into `roots`. Return the number of unique
  /// values put into `roots`.
  void RootsInRange(const Range& x_limits, RootsArray* roots) const {
    roots->len = RootsInRange(x_limits, roots->arr);
  }

  /// Calculate the x-coordinates at the value.
  void XsForValue(float value, RootsArray* roots) const {
    const QuadraticCurve shifted_down(c_[2], c_[1], c_[0] - value);
    shifted_down.Roots(roots);
  }

  /// Get ranges above or below zero. Since a quadratic can cross zero at most
  /// twice, there can be at most two ranges. Ranges are clamped to `x_limits`.
  /// `sign` is used to determine above or below zero only--actual value is
  /// ignored.
  void RangesMatchingSign(const Range& x_limits, const float sign,
                          RangeArray* matching) const {
    matching->len = RangesMatchingSign(x_limits, sign, matching->arr);
  }

  /// Return the ranges on which the quadratic is above zero.
  void RangesAboveZero(const Range& x_limits, RangeArray* matching) const {
    RangesMatchingSign(x_limits, 1.0f, matching);
  }

  /// Return the ranges on which the quadratic is below zero.
  void RangesBelowZero(const Range& x_limits, RangeArray* matching) const {
    RangesMatchingSign(x_limits, -1.0f, matching);
  }

  /// Returns the coefficient for x-to-the-ith -power.
  float Coeff(int i) const { return c_[i]; }

  /// Returns the number of coefficients in this curve.
  int NumCoeff() const { return kNumCoeff; }

  /// Returns the curve f(x / x_scale), where f is the current quadratic.
  /// This stretches the curve along the x-axis by x_scale.
  QuadraticCurve ScaleInX(const float x_scale) const {
    return ScaleInXByReciprocal(1.0f / x_scale);
  }

  /// Returns the curve f(x * x_scale_reciprocal), where f is the current
  /// quadratic.
  /// This stretches the curve along the x-axis by 1/x_scale_reciprocal
  QuadraticCurve ScaleInXByReciprocal(const float x_scale_reciprocal) const {
    return QuadraticCurve(c_[2] * x_scale_reciprocal * x_scale_reciprocal,
                          c_[1] * x_scale_reciprocal, c_[0]);
  }

  /// Returns the curve y_scale * f(x).
  QuadraticCurve ScaleInY(const float y_scale) const {
    return QuadraticCurve(*this, y_scale);
  }

  /// Returns the same curve but with all coefficients in absolute value.
  /// Useful for numerical precision work.
  QuadraticCurve AbsCoeff() const {
    using std::fabs;
    return QuadraticCurve(fabs(c_[2]), fabs(c_[1]), fabs(c_[0]));
  }

  /// Equality. Checks for exact match. Useful for testing.
  bool operator==(const QuadraticCurve& rhs) const;
  bool operator!=(const QuadraticCurve& rhs) const { return !operator==(rhs); }

  /// A string with the equation for this quadratic. Useful for debugging.
  std::string Text() const;

 private:
  // Feel free to expose these versions in the external API if they're useful.
  size_t Roots(float roots[2]) const;
  size_t RootsWithoutNormalizing(float roots[2]) const;
  size_t RootsInRange(const Range& x_limits, float roots[2]) const;
  size_t RangesMatchingSign(const Range& x_limits, const float sign,
                            Range matching[2]) const;

  float c_[kNumCoeff];  /// c_[2] * x^2  +  c_[1] * x  +  c_[0]
};

inline QuadraticCurve operator+(const QuadraticCurve& a,
                                const QuadraticCurve& b) {
  return QuadraticCurve(a.Coeff(2) + b.Coeff(2), a.Coeff(1) + b.Coeff(1),
                        a.Coeff(0) + b.Coeff(0));
}

inline QuadraticCurve operator-(const QuadraticCurve& a,
                                const QuadraticCurve& b) {
  return QuadraticCurve(a.Coeff(2) - b.Coeff(2), a.Coeff(1) - b.Coeff(1),
                        a.Coeff(0) - b.Coeff(0));
}

/// @class CubicInit
/// @brief Initialization parameters to create a cubic curve with start and
///        end y-values and derivatives.
/// Start is x = 0. End is x = width_x.
struct CubicInit {
  CubicInit(const float start_y, const float start_derivative,
            const float end_y, const float end_derivative, const float width_x)
      : start_y(start_y),
        start_derivative(start_derivative),
        end_y(end_y),
        end_derivative(end_derivative),
        width_x(width_x) {}

  // Short-form in comments:
  float start_y;           // y0
  float start_derivative;  // s0
  float end_y;             // y1
  float end_derivative;    // s1
  float width_x;           // w
};

/// @class CubicCurve
/// @brief Represent a cubic polynomial of the form,
///   c_[3] * x^3  +  c_[2] * x^2  +  c_[1] * x  +  c_[0]
class CubicCurve {
 public:
  static const int kNumCoeff = 4;
  CubicCurve() { memset(c_, 0, sizeof(c_)); }
  CubicCurve(const float c3, const float c2, const float c1, const float c0) {
    c_[3] = c3;
    c_[2] = c2;
    c_[1] = c1;
    c_[0] = c0;
  }
  CubicCurve(const float* c) { memcpy(c_, c, sizeof(c_)); }
  CubicCurve(const CubicInit& init) { Init(init); }
  void Init(const CubicInit& init);

  /// Shift the curve along the x-axis: x_shift to the left.
  /// That is x_shift becomes the curve's x=0.
  void ShiftLeft(const float x_shift);

  /// Shift the curve along the x-axis: x_shift to the right.
  void ShiftRight(const float x_shift) { ShiftLeft(-x_shift); }

  /// Shift the curve along the y-axis by y_offset: y_offset up the y-axis.
  void ShiftUp(float y_offset) { c_[0] += y_offset; }

  /// Scale the curve along the y-axis by a factor of y_scale.
  void ScaleUp(float y_scale) {
    for (int i = 0; i < kNumCoeff; ++i) {
      c_[i] *= y_scale;
    }
  }

  /// Return the cubic function's value at `x`.
  /// f(x) = c3*x^3 + c2*x^2 + c1*x + c0
  float Evaluate(const float x) const {
    /// Take advantage of multiply-and-add instructions that are common on FPUs.
    return ((c_[3] * x + c_[2]) * x + c_[1]) * x + c_[0];
  }

  /// Return the cubic function's slope at `x`.
  /// f'(x) = 3*c3*x^2 + 2*c2*x + c1
  float Derivative(const float x) const {
    return (3.0f * c_[3] * x + 2.0f * c_[2]) * x + c_[1];
  }

  /// Return the cubic function's second derivative at `x`.
  /// f''(x) = 6*c3*x + 2*c2
  float SecondDerivative(const float x) const {
    return 6.0f * c_[3] * x + 2.0f * c_[2];
  }

  /// Return the cubic function's constant third derivative.
  /// Even though `x` is unused, we pass it in for consistency with other
  /// curve classes.
  /// f'''(x) = 6*c3
  float ThirdDerivative(const float x) const {
    (void)x;
    return 6.0f * c_[3];
  }

  /// Returns true if always curving upward or always curving downward on the
  /// specified x_limits.
  /// That is, returns true if the second derivative has the same sign over
  /// all of x_limits.
  bool UniformCurvature(const Range& x_limits) const;

  /// Return a value below which floating point precision is unreliable.
  /// If we're testing for zero, for instance, we should test against this
  /// Epsilon().
  float Epsilon() const {
    using std::max;
    using std::fabs;
    const float max_c =
        max(max(max(fabs(c_[3]), fabs(c_[2])), fabs(c_[1])), fabs(c_[0]));
    return max_c * kEpsilonScale;
  }

  /// Returns the coefficient for x to the ith power.
  float Coeff(int i) const { return c_[i]; }

  /// Overrides the coefficent for x to the ith power.
  void SetCoeff(int i, float coeff) { c_[i] = coeff; }

  /// Returns the number of coefficients in this curve.
  int NumCoeff() const { return kNumCoeff; }

  /// Equality. Checks for exact match. Useful for testing.
  bool operator==(const CubicCurve& rhs) const;
  bool operator!=(const CubicCurve& rhs) const { return !operator==(rhs); }

  /// A string with the cubic equation. Useful for debugging.
  std::string Text() const;

 private:
  float c_[kNumCoeff];  /// c_[3] * x^3  +  c_[2] * x^2  +  c_[1] * x  +  c_[0]
};

/// Draw an ASCII-art graph of the array of (x,y) 'points'.
/// The size of the graph in (horizontal characters, vertical lines) is given
/// by 'size'.
std::string Graph2DPoints(const mathfu::vec2* points, const int num_points,
                          const mathfu::vec2i& size = kDefaultGraphSize);

/// Slow function that returns one of the possible values that this curve
/// can evaluate. Useful for debugging.
template <class T>
float CurveValue(const T& curve, const float x,
                 const CurveValueType value_type) {
  switch (value_type) {
    case kCurveValue:
      return curve.Evaluate(x);
    case kCurveDerivative:
      return curve.Derivative(x);
    case kCurveSecondDerivative:
      return curve.SecondDerivative(x);
    case kCurveThirdDerivative:
      return curve.ThirdDerivative(x);
    default:
      assert(false);
  }
  return 0.0f;
}

/// Returns an ASCII-art graph for x in 'x_range' for type T.
template <class T>
std::string GraphCurveOnXRange(const T& curve, const CurveValueType value_type,
                               const Range& x_range,
                               const mathfu::vec2i& size = kDefaultGraphSize) {
  // Gather a collection of (x,y) points to graph.
  const int num_points = size.x();
  std::vector<mathfu::vec2> points(num_points);
  const float inc_x = x_range.Length() / (num_points - 1);
  float x = x_range.start();
  for (int i = 0; i < num_points; ++i) {
    points[i] = mathfu::vec2(x, CurveValue(curve, x, value_type));
    x += inc_x;
  }

  // Output the points in an ASCII-art graph.
  return Graph2DPoints(&points[0], num_points, size);
}

/// Returns an ASCII-art graph from StartX() to EndX() for the requested type.
template <class T>
std::string GraphCurve(const T& curve, const CurveValueType value_type,
                       const mathfu::vec2i& size = kDefaultGraphSize) {
  return curve.Text() + "\n" +
         GraphCurveOnXRange(curve, value_type,
                            Range(curve.StartX(), curve.EndX()), size);
}

}  // namespace motive

#endif  // MOTIVE_MATH_CURVE_H_
