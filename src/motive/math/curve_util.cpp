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

#include "motive/math/curve_util.h"

// Define flags for debugging the math.
#if !defined(NDEBUG)
#define MOTIVE_CURVE_SANITY_CHECKS 1
#endif  // !defined(NDEBUG)

// The scale to be used to determine if number close enough to 0.
static const float kEpsilonXPercent = 0.0001f;

namespace motive {

// Let f(x) and g(x) be quadratic functions with opposite curvature
// (i.e. one curves up and one curves down), and overlapping range.
//
// Find s such that there exists unique u with f(u) = g(u - s).
//
// Two such s's should exist. We return the larger one.
//
//  *                       *
//  * f(x)                  *
//   *                     *
//   *                     *             ***
//    *                   *           *       *
//     *                 *         *             * g(x)
//       *             *         *                 *
//          *       *           *                   *
//              *              *                     *
//                            *                       *
//                            *                       *
//                           *                         *
//                           *                         *
//
//
//  Shift g(x) to the left, here, so that their is a unique intersection.
//
//  *                       *
//  *                       *
//   *                     *
//   *                     *    ***
//    *                   *  *       *
//     *            f(u) **             *
//       *             ** g(u - s)        *
//          *       *  *                   *
//              *     *                     *
//                   *                       *
//       <---------- *                       *
//          s       *                         *
//                  *                         *
static bool CalculateShiftForUniqueIntersection(const QuadraticCurve& f,
                                                const QuadraticCurve& g,
                                                float typical_total_x,
                                                float* intersection_x,
                                                float* shift_x) {
  // Let f(x) = Ax^2 + Bx + C
  //     g(x) = ax^2 + bx + c
  //
  // Fix x at intersection point u and find shift s.
  //
  // Then g(u - s) = a(u - s)^2 + b(u - s) + c
  //               = a(u^2 - 2su + s^2) + b(u - s) + c
  //               = au^2 + bu + c - 2asu + as^2 - bs
  //               = g(u) - 2asu + as^2 - bs
  //
  // So when, f(u) = g(u - s)
  //             0 = f(u) - g(u - s)
  //             0 = f(u) - g(u) + 2asu - as^2 + bs
  //
  // Let h(u) = f(u) - g(u) = pu^2 + qu + r
  // Then,
  //     (1)     0 = pu^2 + (q + 2as)u + (r - as^2 + bs)
  //
  // If we want a unique solution in u, then the discriminant of (1) must be 0.
  //             0 = (q + 2as)^2 - 4p(r - as^2 + bs)
  //             0 = q^2 + 4aqs + 4a^2s^2 - 4pr + 4pas^2 - 4pbs
  //             0 = 4a(a + p)s^2 + 4(aq - pb)s + q^2 - 4pr
  //     (2)     0 = 4aAs^2 + 4(aq - bp)s + q^2 - 4pr
  // We take the larger root of (2) as our shift, since we want to shift g
  // to the right side of f.
  //
  // Since the discriminant of (1) is zero, we can solve easily for u using
  // the quadratic formula (with discriminant removed),
  //             u = -(q + 2as) / 2p
  //
  const QuadraticCurve h = f - g;
  const float a = g.Coeff(2);
  const float b = g.Coeff(1);
  const float p = h.Coeff(2);
  const float q = h.Coeff(1);
  const float r = h.Coeff(0);

  // Calculate s by setting the discriminant of (1) to 0 to generate (2).
  const QuadraticCurve s_quadratic(
      4.0f * a * f.Coeff(2), 4.0f * (a * q - b * p), q * q - 4.0f * p * r);
  QuadraticCurve::RootsArray s_roots;
  s_quadratic.Roots(&s_roots);
  if (s_roots.len == 0) return false;
  const float s = s_roots.arr[s_roots.len - 1];

  // Solve for u using the quadratic formula on (1). It's simpler since the
  // discriminant is 0.
  const float u = -(q + 2.0f * a * s) / (2.0f * p);

  // Calculate epsilon to verify math.
  const float u_epsilon = typical_total_x * kEpsilonXPercent;

#if MOTIVE_CURVE_SANITY_CHECKS
  // Sanity check. This check is just to verify our math. Note that our
  // calculation above is more numerically precise than the Roots() function.
  // Most of the time the quadratic will return only one point
  // since its discriminant will be zero. In that case, the root will be
  // nearly identical. Sometimes numerical precision fails us and we'll get
  // two roots. Both should be reasonably close to u.
  const QuadraticCurve u_quadratic(p, q + 2.0f * a * s, r - a * s * s + b * s);
  const float should_be_zero = u_quadratic.Evaluate(u);
  assert(fabs(should_be_zero) <= u_epsilon);
#endif  // MOTIVE_CURVE_SANITY_CHECKS

  // Set return values. Clamp to 0 if number is very small.
  // This disregards small floating point errors that can add
  // in a tiny positive or negative intersection point, when (mathematically)
  // the intersection point should be 0.
  *intersection_x = fabs(u) < u_epsilon ? 0.0f : u;
  *shift_x = s;
  return true;
}

void CalculateSecondDerivativesFromTypicalCurve(
    float typical_delta_value, float typical_total_x, float bias,
    float* start_second_derivative_abs, float* end_second_derivative_abs) {
  // Let f(x) represent a ease-in curve and g(x) represent an
  // ease-out curve, with respective second_derivatives '2A' and '2a'.
  // Since f(0) = 0 and f'(0) = 0, we get:
  //   f(x) = Ax^2 since f(0) = 0, f'(0) = 0.
  //
  // For s = 0.5f, we have a balanced curve and by symmetry, travel half the
  // distance in half the time:
  //   f(u/2) = d/2
  //   Au^2/4 = d/2
  //   A = 2d / u^2 (1)
  //
  // For s = 1.0f, we travel the full distance in full time and disregard
  // the end second derivative:
  //   f(u) = d
  //   Au^2 = d
  //   A = d/u^2 (2)
  //
  // For 0.5 <= s <= 1, we can scale A linearly from (1) to (2) as s increases,
  // giving:
  //   A = d / (su^2) (3)
  //
  // To calculate a, we let the ease out curve be:
  //   g(x) = ax^2 + bx + c
  //   g'(x) = 2ax + b
  //   g''(x) = 2a
  // Since we know g(u) = d and g'(u) = 0, we get:
  //   0 = 2au + b
  //   b = -2au (4)
  // With:
  //   d = au^2 + bu + c
  //   c = d - au^2 - bu
  //   c = d - au^2 + 2au^2
  //   c = d + au^2 (5)
  //
  // We write g in terms of a by substituting (4) and (5):
  //   g(x) = ax^2 + bx + c
  //   g(x) = ax^2 - 2aux + au^2 + d
  //
  // We solve for 'a' by adjusting the curves so g and f intersect once.
  // They are tangential at the intersection:
  //   f(x) = g(x)
  //   0 = g(x) - f(x)
  //   0 = ax^2 - 2aux + au^2 + d - Ax^2
  //   0 = (a-A)x^2 - 2aux + au^2 + d (6)
  //
  // The discriminant of (6) must be zero for us to have only one solution:
  //   0 = 4a^2u^2 - 4(a-A)(au^2 + d)
  //   0 = 4a^2u^2 - 4a^2u^2 - 4ad + 4Aau^2 + 4Ad
  //   0 = a(-d + Au^2) + Ad
  //   a = Ad / (d - Au^2)   (7)
  //
  // We then substitute (3) to get (7) in terms of s.
  //   a = (d / (su^2))d / (d - (d / (su^2))u^2)
  //   a = (d^2/su^2) / (d - d/s)
  //   a = d / (s - 1)u^2 (8)
  //
  // Additionally, by symmetry, when s < 0.5f, we can
  // use 1 - s with our calculations, and flip our end results and signs so
  // that:
  //   A = -d / (s - 1)u^2   (9)
  //   a = -d / (su^2) (10)
  //
  // Thus, when s >= 0.5f, we get A and a from equations (3) and (8)
  // respectively, and when s < 0.5f, we get A and a from equations
  // (9) and (10) respectively.
  // So, when s >= 0.5f, we get:
  //   start_second_derivative = 2 * A
  //   start_second_derivative = 2 * d / (su^2)  (11)
  //   end_second_derivative = 2 * a
  //   end_second_derivative = 2 * d / (s - 1)u^2   (12)
  //
  // And when s < 0.5f, we get:
  //   start_second_derivative = -2 * d / (s - 1)u^2 (13)
  //   end_second_derivative = -2 * d / (su^2)  (14)
  //
  // This function will return the absolute values of the second derivatives
  // to provide a clearer interface.

  assert(bias >= 0.0f && bias <= 1.0f && typical_total_x > 0.0f &&
         typical_delta_value > 0.0f);
  const bool ease_in_bias = bias >= 0.5f;
  const float s = ease_in_bias ? bias : 1.0f - bias;
  const float typical_total_x_squared = typical_total_x * typical_total_x;
  const float bigger_second_derivative =
      2.0f * typical_delta_value / (s * typical_total_x_squared);
  const float smaller_second_derivative =
      2.0f * typical_delta_value / ((1.0f - s) * typical_total_x_squared);
  *start_second_derivative_abs =
      ease_in_bias ? bigger_second_derivative : smaller_second_derivative;
  *end_second_derivative_abs =
      ease_in_bias ? smaller_second_derivative : bigger_second_derivative;
}

// Calculate a single quadratic curve that goes directly to end_value,
// when the curve is overdetermined and the desired end derivative
// cannot be achieved.
QuadraticEaseInEaseOut CalculateQuadraticFlyOut(float start_value,
                                                float end_value,
                                                float start_derivative,
                                                float second_derivative_abs) {
  const float curvature = end_value >= start_value ? 1.0f : -1.0f;
  const QuadraticCurve fly_out(QuadraticInitWithOrigin(
      start_value, start_derivative, curvature * second_derivative_abs));

  QuadraticCurve::RootsArray end_xs;
  fly_out.XsForValue(end_value, &end_xs);
  // Must be at least one positive end time.
  assert(end_xs.len > 0 &&
         (end_xs.arr[0] >= 0.0f || (end_xs.len == 2 && end_xs.arr[1] >= 0.0f)));
  const float end_x = end_xs.arr[0] >= 0.0f ? end_xs.arr[0] : end_xs.arr[1];
  return QuadraticEaseInEaseOut(fly_out, end_x);
}

QuadraticEaseInEaseOut CalculateQuadraticFlyIn(float start_value,
                                               float end_value,
                                               float end_derivative,
                                               float second_derivative_abs) {
  // We want our fly in curves to have negative curvature.
  const float curvature = start_value >= end_value ? 1.0f : -1.0f;
  QuadraticCurve fly_in(QuadraticInitWithOrigin(
      end_value, end_derivative, curvature * second_derivative_abs));

  // Shift the curve over to the right
  // so that the start value is at the origin.
  QuadraticCurve::RootsArray begin_xs;
  fly_in.XsForValue(start_value, &begin_xs);
  assert(begin_xs.len > 0 && begin_xs.arr[0] <= 0.0f);
  const float begin_x = begin_xs.arr[0];
  fly_in.ShiftRight(-begin_x);

  // Since we shifted our curve, end_x = -begin_x.
  const float end_x = -begin_x;
  return QuadraticEaseInEaseOut(fly_in, end_x);
}

// Returns a QuadraticEaseInEaseOut curve that best matches
// the requested start and end values, derivatives,
// and second derivative.
QuadraticEaseInEaseOut CalculateQuadraticEaseInEaseOut(
    float start_value, float start_derivative,
    float start_second_derivative_abs, float end_value, float end_derivative,
    float end_second_derivative_abs, float typical_delta_value,
    float typical_total_x) {
  // Ensure that both second derivatives are greater than or equal to 0.
  assert(start_second_derivative_abs > 0.0f &&
         end_second_derivative_abs > 0.0f && typical_delta_value > 0.0f &&
         typical_total_x > 0.0f);

  const float value_epsilon = kEpsilonScale * typical_delta_value;
  const float typical_derivative = typical_delta_value / typical_total_x;
  const float derivative_epsilon = kEpsilonScale * typical_derivative;

  bool end_value_eq_start_value =
      std::fabs(end_value - start_value) <= value_epsilon;

  // If the end and start derivative are close to each other
  // and we are at the desired value, return a single
  // point at the value.
  if (end_value_eq_start_value &&
      std::fabs(end_derivative - start_derivative) <= derivative_epsilon) {
    return QuadraticEaseInEaseOut(
        QuadraticInitWithOrigin(start_value, start_derivative, 0), 0.0f);
  }

  // If the end value and start value are equal, the curvature
  // should be positive if start_derivative is negative and
  // negative if it's not.
  float start_curvature = 0.0f;
  if (end_value_eq_start_value) {
    start_curvature = start_derivative < 0.0f ? 1.0f : -1.0f;
  } else {
    start_curvature = end_value >= start_value ? 1.0f : -1.0f;
  }

  // If either second derivative is infinity, calculate the curve
  // using either just the in curve or just the out curve.
  if (std::isinf(start_second_derivative_abs)) {
    return CalculateQuadraticFlyIn(start_value, end_value, end_derivative,
                                   end_second_derivative_abs);
  } else if (std::isinf(end_second_derivative_abs)) {
    return CalculateQuadraticFlyOut(start_value, end_value, start_derivative,
                                    start_second_derivative_abs);
  }

  // Create an in curve and out curve of opposite curvatures with
  // requested values.
  const QuadraticCurve in_curve(
      QuadraticInitWithOrigin(start_value, start_derivative,
                              start_curvature * start_second_derivative_abs));
  QuadraticCurve out_curve(QuadraticInitWithOrigin(
      end_value, end_derivative, -start_curvature * end_second_derivative_abs));
  // Determine the intersection and shift.
  float intersection_x = 0.0f;
  float total_x = 0.0f;
  const bool success = CalculateShiftForUniqueIntersection(
      in_curve, out_curve, typical_total_x, &intersection_x, &total_x);
  assert(success);
  (void)success;

  // If the desired end derivative occurs after intersection or
  // if the intersection is before our start_x, then
  // prioritize achieving the end value rather than the end derivative.
  // At the same time, attempt to get as close to the end derivative as
  // possible.
  if (intersection_x > total_x || intersection_x < 0.0f) {
    const float second_derivative =
        std::max(start_second_derivative_abs, end_second_derivative_abs);
    return CalculateQuadraticFlyOut(start_value, end_value, start_derivative,
                                    second_derivative);
  }

  // Shift the out_curve to be on the right side of in_curve.
  out_curve.ShiftRight(total_x);
  return QuadraticEaseInEaseOut(in_curve, out_curve, intersection_x, total_x);
}

}  // namespace motive
