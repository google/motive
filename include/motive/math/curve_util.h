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
/// @param typical_delta_value Used with typical_total_x to calculate epsilon
///                            value for derivatives.
/// @param typical_total_x The total_x() value of the returned curve that you
///                        would expect in a typical longer curve. Used to
///                        calculate epsilon value for floating point
///                        operations.
QuadraticEaseInEaseOut CalculateQuadraticEaseInEaseOut(
    float start_value, float start_derivative,
    float start_second_derivative_abs, float end_value, float end_derivative,
    float end_second_derivative_abs, float typical_delta_value,
    float typical_total_x);

/// @brief Provides an intuitive way to calculate second derivatives for
/// a QuadraticEaseInEaseOut curve.
///
/// People normally think about ease-in ease-out curves as starting and
/// ending with derivative 0. So our `typical` curve takes this shape by
/// traveling a typical total distance, d, in time, u, with start
/// and end derivatives of 0 and a bias.
/// The bias determines the transition between in curve and out curve.
/// The following are examples of what curves would look like with different
/// bias values when end value is greater than start value:
///
///  bias = 0.0 (fly out; i.e. no ease-out, just ease-in):
///    |                                                          *
///    |                                                        **
///    |                                                      **
///    |                                                     *
///    |                                                   **
///    |                                                 **
///    |                                               **
///    |                                             **
///    |                                           **
///    |                                         **
///    |                                       **
///    |                                     **
///    |                                  ***
///    |                               ***
///    |                            ***
///    |                         ***
///    |                     ****
///    |                *****
///    |         *******
///    **********
///
///
///  bias = 0.15:
///    |
///    |                                                  *********
///    |                                           *******
///    |                                       ****
///    |                                   ****
///    |                                ***
///    |                             ***
///    |                           **
///    |                        ***
///    |                      **
///    |                    **
///    |                  **
///    | bias 0.15      **
///    |              **
///    |             *
///    |           **
///    |         **
///    |        *
///    |      **
///    |   ***
///    ****
///
///
///  bias = 0.5:
///    |                                                    *******
///    |                                               *****
///    |                                           ****
///    |                                         **
///    |                                      ***
///    |                                    **
///    |                                  **
///    |                                **
///    |                               *
///    |                             **
///    |       bias 0.5            **
///    |                          *
///    |                        **
///    |                      **
///    |                    **
///    |                 ***
///    |               **
///    |           ****
///    |      *****
///    *******
///
///
///  bias = 0.85:
///    |
///    |                                                       ****
///    |                                                    ***
///    |                                                  **
///    |                                                 *
///    |                                               **
///    |                                             **
///    |                bias 0.85                   *
///    |                                          **
///    |                                        **
///    |                                      **
///    |                                    **
///    |                                  **
///    |                               ***
///    |                             **
///    |                          ***
///    |                       ***
///    |                   ****
///    |               ****
///    |        *******
///    *********
///
///
///  bias = 1.0 (fly in; i.e. no ease-in, just ease-out):
///    |
///    |                                                 **********
///    |                                          *******
///    |                                     *****
///    |                                 ****
///    |                              ***
///    |                           ***
///    |                        ***
///    |                     ***
///    |                   **
///    |                 **
///    |               **
///    |             **
///    |           **
///    |         **
///    |       **
///    |     **
///    |    *
///    |  **
///    |**
///    *
///
/// @param typical_delta_value The typical difference between the
///                            start and end values.
/// @param typical_total_x The typical time it takes to go the typical distance.
/// @param bias Determines how much the curve should ease-in and how much it
///             should ease-out. Should be a value from 0.0 to 1.0.
/// @param start_second_derivative Curve's start second derivative.
/// @param end_second_derivative Curve's end second derivative.
void CalculateSecondDerivativesFromTypicalCurve(
    float typical_delta_value, float typical_total_x, float bias,
    float* start_second_derivative_abs, float* end_second_derivative_abs);

/// @brief Returns a curve that goes to the end value as quickly as possible,
/// using just ease-in.
///
/// This is the curve when bias is 0.0f.
///
/// @param start_value Curve's initial value.
/// @param end_value Curve's end value.
/// @param start_derivative Curve's initial derivative.
/// @param second_derivative Curve's second derivative.
QuadraticEaseInEaseOut CalculateQuadraticFlyOut(float start_value,
                                                float end_value,
                                                float start_derivative,
                                                float second_derivative_abs);

/// @brief Returns a curve that goes to the end value as quickly as possible,
/// using just ease-out.
///
/// This is the curve when bias is 1.0f.
///
/// @param start_value Curve's initial value.
/// @param end_value Curve's end value.
/// @param end_derivative Curve's end derivative.
/// @param second_derivative Curve's second derivative.
QuadraticEaseInEaseOut CalculateQuadraticFlyIn(float start_value,
                                               float end_value,
                                               float end_derivative,
                                               float second_derivative_abs);

/// @class QuadraticSpring
/// @brief An oscillating curve that accelerates quadratically.
///
/// The curve can either,
/// - dampen (as in graph) if bias < 1,
/// - grow if bias > 1,
/// - or have constant amplitude if bias = 1.
///
///                     value
///                       ^
///                       |
///   typical delta value +--___     typical
///                       |     --_  delta x
///                       |        \_   .
///                       |          \  .
///                       |           \ .
///                       |            \.          _--_
///          target value +-------------+---------+----+__+-----> x
///                       |              \_     _/
///                       |                -___-
///
/// When starting from zero velocity, the curve will travel typical_delta_value
/// in typical_delta_x to intercept the target_value for the first time.
///
/// However, the spring motion can start from any value and velocity, and the
/// calculated movement will mimic the typical momement mathematically.
/// The start state is specified in current_value and current_derivative.
///
class QuadraticSpring {
 public:
  /// @brief Describe one portion of the QuadraticSpring curve. Useful for
  ///        quickly evaluating the curve iteratively.
  /// @note This struct should not be used directly. Instead, it should be
  ///       passed into @ref EvaluateWithContext, @ref DerivativeWithContext,
  ///       and @ref SecondDerivativeWithContext.
  struct Context {
    Context()
        : coeff(0.0f), peak(0.0f), peak_x(0.0f), valid_x(0.0f) {}

    // This constructor should only be called internally by
    // @ref CalculateContext and @ref IncrementContext.
    Context(float coeff, float peak, float peak_x, float valid_start_x,
            float valid_end_x)
        : coeff(coeff), peak(peak), peak_x(peak_x),
          valid_x(valid_start_x, valid_end_x) {}

    // This diagram illustrates the meaning of each member variable.
    // Note that this structure is only used internally, and should not be
    // accessed or modified by external code. Explanations given for reference
    // only.
    //
    //      value     C(x) = coeff * x^2 + peak
    //        ^        .
    //        |       .
    //   peak +--___ .
    //        |     --_   peak_x
    //        |        \_   .
    //        |          \  .
    //        |           \ .
    //        |            \.
    //      0 +-------------+---> x
    //
    //        <== valid_x ==>

    // The quadratic coefficient.
    // If negative, curves downwards; if positive, curves upwards.
    // Its absolute value will be either start_away_coeff_,
    // if moving away from the target value, or start_away_coeff_ * bias_,
    // if moving towards.
    float coeff;

    // The amplitude of the current oscilation.
    float peak;

    // The x value of the current oscillation.
    // Either valid_x.start(), if curving towards the target, or valid_x.end(),
    // if curving away.
    // In "internal-x" values, where the start_peak_ is at x = 0.
    float peak_x;

    // The x values that are valid for this segment of the spring curve.
    // In "internal-x" values, where the start_peak_ is at x = 0.
    Range valid_x;
  };

  /// @brief Default constructor creates a curve at constant value 0.0f.
  QuadraticSpring()
      : start_away_coeff_(0.0f),
        start_peak_(0.0f),
        start_x_(0.0f),
        target_(0.0f),
        bias_(0.0f),
        r_(0.0f),
        sqrt_b_(0.0f),
        log_sqrt_b_reciprocal_(0.0f) {}

  /// @brief Creates a curve at constant value `current_value` and
  ///        constant derivative 0.
  /// @param current_value The constant value of the curve.
  explicit QuadraticSpring(float current_value)
      : start_away_coeff_(0.0f),
        start_peak_(0.0f),
        start_x_(0.0f),
        target_(current_value),
        bias_(0.0f),
        r_(0.0f),
        sqrt_b_(0.0f),
        log_sqrt_b_reciprocal_(0.0f) {}

  /// @brief Creates a curve that starts at `current_value` and oscillates
  ///        about `target_value`.
  /// @param current_value The starting value of the curve.
  /// @param current_derivative The starting derivative of the curve.
  /// @param target_value The value we oscillate about.
  /// @param typical_delta_value Together with typical_delta_x, describes how
  ///                            quickly the curve moves to the target.
  ///                            See class description for details.
  /// @param typical_delta_x Together with typical_delta_value, describes how
  ///                        quickly the curve moves to the target.
  ///                        See class description for details.
  /// @param bias Determines how quickly the curve settles down. That is, the
  ///             tightness of the spring. The smaller, the tighter.
  ///              < 1  ==>  dampens down to zero eventually
  ///              = 1  ==>  oscilates with same amplitude indefinitely
  ///              > 1  ==>  amplitude grows with every oscillation
  ///             Each peak has has the magnitude of the previous peak * bias.
  QuadraticSpring(float current_value, float current_derivative,
                  float target_value, float typical_delta_value,
                  float typical_delta_x, float bias);

  /// @brief Ensure Context `c` is valid for `external_x`.
  ///
  /// Most times, `external_x` will already be within the valid x-range of `c`.
  /// When it's not, we advance `c` to the next quadratic in the series of
  /// curves that compose the spring curve. We continue until `external_x` is
  /// again in the valid range of `c`.
  ///
  /// @param external_x The current x value we're evaluating. The expectation is
  ///                   that `external_x` will be incrementing at a reasonably
  ///                   small and steady rate. When this happens, `c` only has
  ///                   to be updated when `external_x` exceeds the portion of
  ///                   the spring curve that `c` describes.
  /// @param c The current evaluation context. It describes one portion of
  ///          the spring curve. After updating `c` with this function, you can
  ///          pass it into @ref EvaluateWithContext and other such functions
  ///          to calculate the curve values quickly.
  void IncrementContext(float external_x, Context* c) const {
    const float x = ToInternalX(external_x);
    if (x > c->valid_x.end()) {
      IncrementContextInternal(x, c);
    }
    assert(ValidX(external_x, *c));
  }

  /// @brief Calculate a Context that describes the portion of the spring curve
  ///        near `external_x`.
  ///
  /// It's significantly faster to call this function only once, and then,
  /// for succeeding xs, advance the Context by calling @ref IncrementContext.
  ///
  /// @param external_x The x value that about which we want the returned
  ///                   Context to be valid.
  /// @returns A Context that describes a portion of the spring curve.
  ///          Use it in calls to @ref EvaluateWithContext, etc.
  Context CalculateContext(float external_x) const;

  /// @brief Calculate the spring curve value at `external_x`.
  ///
  /// @note This function is significantly slower than @ref EvaluateWithContext.
  ///       Consider maintaining a Context externally and then calling
  ///       @ref IncrementContext and @ref EvaluateWithContext instead of
  ///       just Evaluate.
  ///
  /// @param external_x x value since the start of the curve, as specified by
  ///                   `current_value` and `current_derivative` in the
  ///                   constructor.
  /// @returns The spring curve value at `external_x`.
  float Evaluate(float external_x) const {
    const Context c = CalculateContext(external_x);
    return EvaluateWithContext(external_x, c);
  }

  /// @brief Calculate the spring curve derivative at `external_x`.
  ///
  /// @note This function is significantly slower than
  ///       @ref DerivativeWithContext.
  ///       Consider maintaining a Context externally and then calling
  ///       @ref IncrementContext and @ref DerivativeWithContext instead of
  ///       just Derivative.
  ///
  /// @param external_x x value since the start of the curve.
  /// @returns The spring curve derivative at `external_x`.
  float Derivative(float external_x) const {
    const Context c = CalculateContext(external_x);
    return DerivativeWithContext(external_x, c);
  }

  /// @brief Calculate the spring curve second derivative at `external_x`.
  ///
  /// @note This function is significantly slower than
  ///       @ref SecondDerivativeWithContext.
  ///       Consider maintaining a Context externally and then calling
  ///       @ref IncrementContext and @ref SecondDerivativeWithContext
  ///       instead of just SecondDerivative.
  ///
  /// @param external_x x value since the start of the curve.
  /// @returns The spring curve second derivative at `external_x`.
  ///          Note that there are only two possible second derivatives:
  ///          one towards the target and one away from the target.
  ///          This is a concequence of our usage of quadratic functions.
  float SecondDerivative(float external_x) const {
    const Context c = CalculateContext(external_x);
    return SecondDerivativeWithContext(external_x, c);
  }

  /// @brief Quickly calculate the spring curve value at `external_x`.
  ///
  /// @param external_x x value since the start of the curve.
  /// @param c Context that describes the portion of the spring curve around
  ///          `external_x`. Note that you should call @ref IncrementContext
  ///          with this `external_x` and `c` before calling this function.
  /// @returns The spring curve value at `external_x`.
  float EvaluateWithContext(float external_x, const Context& c) const {
    assert(ValidX(external_x, c));
    const float quadratic_x = QuadraticX(external_x, c);
    const float offset = c.coeff * quadratic_x * quadratic_x;
    return target_ + c.peak + offset;
  }

  /// @brief Quickly calculate the spring curve derivative at `external_x`.
  ///
  /// @param external_x x value since the start of the curve.
  /// @param c Context that describes the portion of the spring curve around
  ///          `external_x`. Note that you should call @ref IncrementContext
  ///          with this `external_x` and `c` before calling this function.
  /// @returns The spring curve derivative at `external_x`.
  float DerivativeWithContext(float external_x, const Context& c) const {
    assert(ValidX(external_x, c));
    const float quadratic_x = QuadraticX(external_x, c);
    return 2.0f * c.coeff * quadratic_x;
  }

  /// @brief Quickly calculate the spring curve second derivative at
  ///        `external_x`.
  ///
  /// @param external_x x value since the start of the curve.
  /// @param c Context that describes the portion of the spring curve around
  ///          `external_x`. Note that you should call @ref IncrementContext
  ///          with this `external_x` and `c` before calling this function.
  /// @returns The spring curve second derivative at `external_x`.
  float SecondDerivativeWithContext(float external_x, const Context& c) const {
    assert(ValidX(external_x, c));
    (void)external_x;
    return 2.0f * c.coeff;
  }

  /// @brief Return the spring curve's third derivative.
  /// @returns Return value is always 0 because internally we return quadratics.
  float ThirdDerivative(float external_x) const {
    (void)external_x;
    return 0.0f;
  }

  /// @brief Calculate the x for the `iterations`th peak of the spring curve.
  /// @param iterations The number of iterations to be completed, where an
  ///                   iteration is defined as one peak to the next.
  /// @returns x for the `iterations`th peak.
  float IterationX(float iterations) const;

  /// @brief Get the `target_value` originally passed into the constructor.
  /// @returns The value about which we're oscillating.
  float target() const { return target_; }

 private:
  bool ValidX(float external_x, const Context& c) const {
    const float kErrorToleranceForX = 0.01f;
    return c.valid_x.ContainsWithTolerance(ToInternalX(external_x),
                                           kErrorToleranceForX);
  }
  void IncrementContextInternal(float x, Context* c) const;
  float ToInternalX(float external_x) const { return external_x + start_x_; }
  float ToExternalX(float internal_x) const { return internal_x - start_x_; }
  float QuadraticX(float external_x, const Context& c) const {
    return ToInternalX(external_x) - c.peak_x;
  }

  // Quadratic coefficient when moving away from the target value (i.e. when
  // spring is slowing down). Note that coefficient when moving towards target
  // value is start_away_coeff_ * bias_.
  float start_away_coeff_;

  // Delta between peak and target of the first peak in the curve.
  float start_peak_;

  // Time of the start_peak_, above. Note that this could be a negative x,
  // if the 'current' state in the constructor has the curve already moving
  // towards the target.
  float start_x_;

  // Target value we're oscillating about.
  float target_;

  // Amount that each succeeding peak is dampened by.
  // That is, the second peak will have amplitude start_peak_ * bias_.
  float bias_;

  // Cached variable holding,
  //   R = sqrt(D/a) * (1 + b) / (sqrt(b) - b), when bias != 1 (see equation 5),
  // and
  //   R = sqrt(a / D) / 2, when bias = 1 (see equation 8 in curve_util.cpp).
  float r_;

  // Cached variable holding sqrt(bias_). This is expensive to calculate and
  // commonly used.
  float sqrt_b_;

  // Cached variable holding 1.0f / log(sqrt_b_).
  float log_sqrt_b_reciprocal_;
};

}  // namespace motive

#endif  // MOTIVE_MATH_CURVE_UTIL_H_
