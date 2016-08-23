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
  // Since the descriminant of (1) is zero, we can solve easily for u using
  // the quadratic formula (with descriminant removed),
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

// Calculate a single quadratic curve that goes directly to end_value,
// when the curve is overdetermined and the desired end derivative
// cannot be achieved.
QuadraticEaseInEaseOut CalculateExtremeQuadraticEaseInEaseOut(
    float start_value, float end_value, float end_derivative,
    float start_derivative, float start_second_derivative_abs,
    float end_second_derivative_abs) {
  float start_sign = end_derivative >= start_derivative ? 1.0f : -1.0f;

  // Select the larger second derivative, which will produce
  // more change in less time.
  const float second_derivative =
      start_sign *
      std::max(start_second_derivative_abs, end_second_derivative_abs);

  const QuadraticCurve curve(QuadraticInitWithOrigin(
      start_value, start_derivative, second_derivative));
  QuadraticCurve::RootsArray end_xs;
  curve.XsForValue(end_value, &end_xs);
  // Must be at least one positive end time.
  assert(end_xs.len > 0 &&
         (end_xs.arr[0] >= 0.0f || (end_xs.len == 2 && end_xs.arr[1] >= 0.0f)));
  const float end_x = end_xs.arr[0] >= 0.0f ? end_xs.arr[0] : end_xs.arr[1];
  return QuadraticEaseInEaseOut(curve, end_x);
}

// Returns a QuadraticEaseInEaseOut curve that best matches
// the requested start and end values, derivatives,
// and second derivative;
QuadraticEaseInEaseOut CalculateQuadraticEaseInEaseOut(
    float start_value, float start_derivative,
    float start_second_derivative_abs, float end_value, float end_derivative,
    float end_second_derivative_abs, float typical_total_x) {
  // Ensure that both second derivatives are greater than or equal to 0.
  assert(start_second_derivative_abs > 0.0f &&
         end_second_derivative_abs > 0.0f);

  float start_sign = end_value >= start_value ? 1.0f : -1.0f;

  // Create an in curve and out curve of opposite curvatures with
  // requested values.
  const QuadraticCurve in_curve(QuadraticInitWithOrigin(
      start_value, start_derivative, start_sign * start_second_derivative_abs));
  QuadraticCurve out_curve(QuadraticInitWithOrigin(
      end_value, end_derivative, -start_sign * end_second_derivative_abs));
  // Determine the intersection and shift.
  float intersection_x = 0.0f;
  float total_x = 0.0f;
  const bool success = CalculateShiftForUniqueIntersection(
      in_curve, out_curve, typical_total_x, &intersection_x, &total_x);
  assert(success);

  // TODO(laijess): Fix to account for floating point errors.
  // If the end and start derivative are of the same sign and
  // we are at the desired value, return a single point at the value.
  if (end_derivative * start_derivative >= 0.0f && end_value == start_value) {
    return QuadraticEaseInEaseOut(
        QuadraticInitWithOrigin(start_value, start_derivative, 0), 0.0f);
  }

  // If the desired end derivative occurs after intersection or
  // if the intersection is before our start_x, then
  // prioritize achieving the end value rather than the end derivative.
  // At the same time, attempt to get as close to the end derivative as
  // possible.
  if (intersection_x > total_x || intersection_x < 0.0f) {
    return CalculateExtremeQuadraticEaseInEaseOut(
        start_value, end_value, end_derivative, start_derivative,
        start_second_derivative_abs, end_second_derivative_abs);
  }

  // Shift the out_curve to be on the right side of in_curve.
  out_curve.ShiftRight(total_x);
  return QuadraticEaseInEaseOut(in_curve, out_curve, intersection_x, total_x);
}

}  // namespace motive
