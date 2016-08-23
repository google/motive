// Copyright 2016 Google Inc. All rights reserved.
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

#ifndef MOTIVE_MATH_CURVE_UTIL_H_
#define MOTIVE_MATH_CURVE_UTIL_H_

#include "motive/math/curve.h"

namespace motive {

/// @class QuadraticEaseInEaseOut
/// @brief Represent a curve containing an ease in and ease out
///        quadratic curve.
///
/// This class parameterizes two curves: ease-in followed by ease-out.
/// The ease-in curve ends after intersection_x, and the ease-out curve
/// matches the ease-in curve's value and derivative at that x. The
/// ease-in curve begins at x = 0. The ease-out curve ends at x = total_x.
class QuadraticEaseInEaseOut {
 public:
  QuadraticEaseInEaseOut() : intersection_x_(0.0f), total_x_(0.0f) {}
  QuadraticEaseInEaseOut(const QuadraticCurve& in, const QuadraticCurve& out,
                         float intersection_x, float total_x)
      : in_curve_(in),
        out_curve_(out),
        intersection_x_(intersection_x),
        total_x_(total_x) {}
  QuadraticEaseInEaseOut(const QuadraticCurve& in, float end_x)
      : in_curve_(in),
        out_curve_(in),
        intersection_x_(end_x),
        total_x_(end_x) {}

  /// Return the curve's value at x.
  float Evaluate(float x) const { return Curve(x).Evaluate(x); }

  /// Return the curve's derivative at 'x'.
  float Derivative(float x) const { return Curve(x).Derivative(x); }

  /// Return the curve's second derivative at 'x'
  /// using either the in_curve or out_curve.
  float SecondDerivative(float x) const { return Curve(x).SecondDerivative(x); }

  /// Return the quadratic function's third derivative: 0.
  /// Even though `x` is unused, we pass it in for consistency with other
  /// curve classes.
  float ThirdDerivative(float x) const {
    (void)x;
    return 0.0f;
  }

  /// Return the x at which in_curve and out_curve intersect.
  float intersection_x() const { return intersection_x_; }

  /// Return the x where the curve reaches its end_value.
  float total_x() const { return total_x_; }

  /// Return the in_curve.
  const QuadraticCurve& in_curve() const { return in_curve_; }

  /// Return the out_curve.
  const QuadraticCurve& out_curve() const { return out_curve_; }

  /// Return either the in_curve or the out_curve, depending
  /// on if x is before or after intersection_x.
  const QuadraticCurve& Curve(float x) const {
    return x <= intersection_x_ ? in_curve_ : out_curve_;
  }

  /// Equality. Checks for exact match. Useful for testing.
  bool operator==(const QuadraticEaseInEaseOut& rhs) const {
    return intersection_x_ == rhs.intersection_x_ && total_x_ == rhs.total_x_ &&
           out_curve_ == rhs.out_curve_ && in_curve_ == rhs.in_curve_;
  }
  bool operator!=(const QuadraticEaseInEaseOut& rhs) const {
    return !operator==(rhs);
  }

 private:
  QuadraticCurve in_curve_;
  QuadraticCurve out_curve_;
  float intersection_x_;
  float total_x_;
};

/// @brief Returns a curve that matches start and end values, derivatives, and
/// second derivatives.
///
/// @param start_value Curve's initial value.
/// @param start_derivative Curve's initial derivative.
/// @param start_second_derivative_abs Absolute value of curve's initial second
///                                    derivative.
/// @param end_value Curve's final value.
/// @param end_derivative Curve's desired final derivative. If curve is
///                       overdetermined, this derivative may not be achieved.
/// @param end_second_derivative_abs Absolute value of the float that represents
///                                  the QuadraticEaseInEaseOut's desired final
///                                  second derivative.
/// @param typical_total_x The total_x() value of the returned curve that you
///                        would expect in a typical longer curve. Used to
///                        calculate epsilon value for floating point
///                        operations.
QuadraticEaseInEaseOut CalculateQuadraticEaseInEaseOut(
    float start_value, float start_derivative,
    float start_second_derivative_abs, float end_value, float end_derivative,
    float end_second_derivative_abs, float typical_total_x);

}  // namespace motive

#endif  // MOTIVE_MATH_CURVE_UTIL_H_
