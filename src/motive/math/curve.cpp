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

#include <cmath>
#include <sstream>
#include <string>
#include <vector>
#include "motive/math/bulk_spline_evaluator.h"
#include "motive/math/float.h"

#ifdef _DEBUG
#define MOTIVE_CURVE_SANITY_CHECKS
#endif // _DEBUG

using mathfu::vec2;
using mathfu::vec2i;

namespace motive {

void QuadraticCurve::Init(const QuadraticInitWithStartDerivative& init) {
  //  f(u) = cu^2 + bu + a
  //  f(0) = a
  //  f'(0) = b
  //  f(1) = c + b + a   ==>   c = f(1) - b - a
  //                             = f(1) - f(0) - f'(0)
  c_[0] = init.start_y;
  c_[1] = init.start_derivative;
  c_[2] = init.end_y - init.start_y - init.start_derivative;
}

void QuadraticCurve::Init(const QuadraticInitWithOrigin& init) {
  //  f(u) = cu^2 + bu + a
  //  f(0) = a
  //  f'(0) = b
  //  f''(0) = 2c  ==>  c = f''(0) / 2
  c_[0] = init.y;
  c_[1] = init.derivative;
  c_[2] = 0.5f * init.second_derivative;
}

void QuadraticCurve::Init(const QuadraticInitWithPoint& init) {
  //  f(u) = cu^2 + bu + a
  //  f'(u) = 2cu + b
  //  f''(u) = 2c
  //     ==>  c = f''(x) / 2
  //     ==>  b = f'(x) - 2cx
  //            = f'(x) - f''(x)*x
  //     ==>  a = f(x) - cx^2 - bx
  //            = f(x) - x(cx + b)
  c_[2] = 0.5f * init.second_derivative;
  c_[1] = init.derivative_at_x - init.second_derivative * init.x;
  c_[0] = init.y_at_x - init.x * (c_[2] * init.x + c_[1]);
}

void QuadraticCurve::ShiftLeft(const float x_shift) {
  // Early out optimization.
  if (x_shift == 0.0f) return;

  // s = x_shift
  // f(x) = cx^2 + bx + a
  // f(x + s) = c(x+s)^2 + b(x+s) + a
  //          = c(x^2 + 2sx + s^2) + b(x + s) + a
  //          = cx^2 + (2c + b)x + (cs^2 + bs + a)
  //          = cx^2 + f'(s) x + f(s)
  //
  // Or, for an more general formulation, see:
  //     http://math.stackexchange.com/questions/694565/polynomial-shift
  const float new_b = Derivative(x_shift);
  const float new_a = Evaluate(x_shift);
  c_[0] = new_a;
  c_[1] = new_b;
}

float QuadraticCurve::ReliableDiscriminant(const float epsilon) const {
  // When discriminant is (relative to coefficients) close to zero, we treat
  // it as zero. It's possible that the discriminant is barely below zero due
  // to floating point error.
  const float discriminant = Discriminant();
  return ClampNearZero(discriminant, epsilon);
}

size_t QuadraticCurve::Roots(float roots[2]) const {
  // Leave a little headroom for arithmetic.
  static const int kMaxExponentForRootCoeff = kMaxInvertableExponent - 1;

  // Scale in the x-axis so that c2 is in the range of the larger of c1 or c0.
  // This eliminates numerical precision problems in cases where, for example,
  // we have a tiny second derivative and a large constant.
  //
  // The x-axis scale is applied non-uniformly across the polynomial.
  //    f(x_scale * x) = x_scale^2 * c2 * x^2  +  x_scale * c1 * x  +  c0
  // We use this to bring x_scale^2 * c2 in approximately equal to
  // either x_scale * c1 or c0.
  const QuadraticCurve abs = AbsCoeff();
  const bool scale_with_linear = abs.c_[1] >= abs.c_[0];
  const float comparison_coeff = std::max(abs.c_[1], abs.c_[0]);
  const float x_scale_quotient = abs.c_[2] / comparison_coeff;
  const float x_scale_reciprocal_unclamped =
      !kInvertablePowerOf2Range.Contains(x_scale_quotient)
          ? 1.0f
          : scale_with_linear ? ReciprocalExponent(x_scale_quotient)
                              : SqrtReciprocalExponent(x_scale_quotient);

  // Since we normalize through powers of 2, the scale can be large without
  // losing precision. But we still have to worry about scaling to infinity.
  // Note that in x-scale, only the linear (c1) and quadratic (c2) coefficients
  // are
  // scaled, and the quadratic coefficient is scaled to match an existing
  // coefficient,
  // so we only need to check the linear coefficient.
  const float x_scale_reciprocal_max =
      MaxPowerOf2Scale(abs.c_[1], kMaxInvertableExponent);
  const float x_scale_reciprocal =
      std::min(x_scale_reciprocal_unclamped, x_scale_reciprocal_max);

  // Create the quatratic scaled in x.
  const QuadraticCurve x_scaled = ScaleInXByReciprocal(x_scale_reciprocal);
  const QuadraticCurve x_scaled_abs = x_scaled.AbsCoeff();

#ifdef MOTIVE_CURVE_SANITY_CHECKS
  // Sanity checks to ensure our math is correct.
  if (kInvertablePowerOf2Range.Contains(x_scale_quotient)) {
    const float x_scaled_quotient =
        x_scaled_abs.c_[2] /
        (scale_with_linear ? x_scaled_abs.c_[1] : x_scaled_abs.c_[0]);
    assert(0.5f <= x_scaled_quotient && x_scaled_quotient <= 2.0f);
    (void)x_scaled_quotient;
  }
#endif  // MOTIVE_CURVE_SANITY_CHECKS

  // Calculate the y-axis scale so that c2 is near 1.
  // We need this because the quadratic equation divides by c2.
  //
  // The y-scale is applied evenly to all coefficients, and doesn't affect the
  // roots.
  //   y_scale * f(x) = y_scale * c2 * x^2  +  y_scale * c1 * x  +  y_scale * c0
  //
  // Check need to clamp our y-scale so that the linear (c1) and constant (c0)
  // coefficients
  // don't go to infinity or denormalize. Note that the y-scale is calculated to
  // bring the
  // quadratic (c2) coefficient near 1, so we don't have to check the quadratic
  // coefficient.
  const float y_scale_unclamped =
      ReciprocalExponent(kInvertablePowerOf2Range.Clamp(x_scaled_abs.c_[2]));
  const float y_scale_max =
      std::min(MaxPowerOf2Scale(x_scaled_abs.c_[0], kMaxExponentForRootCoeff),
               MaxPowerOf2Scale(x_scaled_abs.c_[1], kMaxExponentForRootCoeff));
  const float y_scale = std::min(y_scale_max, y_scale_unclamped);

  // Create a scaled version of our quadratic.
  const QuadraticCurve x_and_y_scaled = x_scaled.ScaleInY(y_scale);

#ifdef MOTIVE_CURVE_SANITY_CHECKS
  // Sanity check to ensure our math is correct.
  const QuadraticCurve x_and_y_scaled_abs = x_and_y_scaled.AbsCoeff();
  assert((Range(0.5f, 2.0f).Contains(x_and_y_scaled_abs.c_[2]) ||
          !kInvertablePowerOf2Range.Contains(x_scaled_abs.c_[2]) ||
          y_scale != y_scale_unclamped) &&
         x_and_y_scaled_abs.c_[1] <= std::numeric_limits<float>::max() &&
         x_and_y_scaled_abs.c_[0] <= std::numeric_limits<float>::max());
  (void)x_and_y_scaled_abs;
#endif  // MOTIVE_CURVE_SANITY_CHECKS

  // Calculate the roots and then undo the x_scaling.
  const size_t num_roots = x_and_y_scaled.RootsWithoutNormalizing(roots);
  for (size_t i = 0; i < num_roots; ++i) {
    roots[i] *= x_scale_reciprocal;
  }
  return num_roots;
}

// See the Quadratic Formula for details:
// http://en.wikipedia.org/wiki/Quadratic_formula
// Roots returned in sorted order, smallest to largest.
size_t QuadraticCurve::RootsWithoutNormalizing(float roots[2]) const {
  // x^2 coefficient of zero means that curve is linear or constant.
  const float epsilon = EpsilonOfCoefficients();
  if (std::fabs(c_[2]) < epsilon) {
    // If constant, even if zero, return no roots. This is arbitrary.
    if (std::fabs(c_[1]) < epsilon) return 0;

    // Linear 0 = c1x + c0 ==> x = -c0 / c1.
    roots[0] = -c_[0] / c_[1];
    return 1;
  }

  // A negative discriminant means no real roots.
  const float discriminant = ReliableDiscriminant(epsilon);
  if (discriminant < 0.0f) return 0;

  // A zero discriminant means there is only one root.
  const float divisor = (1.0f / c_[2]) * 0.5f;
  if (discriminant == 0.0f) {
    roots[0] = -c_[1] * divisor;
    return 1;
  }

  // Positive discriminant means two roots. We use the quadratic formula.
  const float sqrt_discriminant = std::sqrt(discriminant);
  const float root_minus = (-c_[1] - sqrt_discriminant) * divisor;
  const float root_plus = (-c_[1] + sqrt_discriminant) * divisor;
  assert(root_minus != root_plus);
  roots[0] = std::min(root_minus, root_plus);
  roots[1] = std::max(root_minus, root_plus);
  return 2;
}

size_t QuadraticCurve::RootsInRange(const Range& valid_x,
                                    float roots[2]) const {
  const size_t num_roots = Roots(roots);

  // We allow the roots to be slightly outside the bounds, since this may
  // happen due to floating point error.
  const float epsilon_x = valid_x.Length() * kEpsilonScale;

  // Loop through each root and only return it if it is within the range
  // [start_x - epsilon_x, end_x + epsilon_x]. Clamp to [start_x, end_x].
  return Range::ValuesInRange(valid_x, epsilon_x, num_roots, roots);
}

size_t QuadraticCurve::RangesMatchingSign(const Range& x_limits, float sign,
                                          Range matching[2]) const {
  // Gather the roots of the validity spline. These are transitions between
  // valid and invalid regions.
  float roots[2];
  const size_t num_roots = RootsInRange(x_limits, roots);

  // We want ranges where the spline's sign equals valid_sign's.
  const bool valid_at_start = sign * Evaluate(x_limits.start()) >= 0.0f;
  const bool valid_at_end = sign * Evaluate(x_limits.end()) >= 0.0f;

  // If no roots, the curve never crosses zero, so the start and end validity
  // must be the same.
  // If two roots, the curve crosses zero twice, so the start and end validity
  // must be the same.
  assert(num_roots == 1 || valid_at_start == valid_at_end);

  // Starts invalid, and never crosses zero so never becomes valid.
  if (num_roots == 0 && !valid_at_start) return 0;

  // Starts valid, crosses zero to invalid, crosses zero again back to valid,
  // then ends valid.
  if (num_roots == 2 && valid_at_start) {
    matching[0] = Range(x_limits.start(), roots[0]);
    matching[1] = Range(roots[1], x_limits.end());
    return 2;
  }

  // If num_roots == 0: must be valid at both start and end so entire range.
  // If num_roots == 1: crosses zero once, or just touches zero.
  // If num_roots == 2: must start and end invalid, so valid range is between
  // roots.
  const float start = valid_at_start ? x_limits.start() : roots[0];
  const float end =
      valid_at_end ? x_limits.end() : num_roots == 2 ? roots[1] : roots[0];
  matching[0] = Range(start, end);
  return 1;
}

bool QuadraticCurve::operator==(const QuadraticCurve& rhs) const {
  for (int i = 0; i < kNumCoeff; ++i) {
    if (c_[i] != rhs.c_[i]) return false;
  }
  return true;
}

std::string QuadraticCurve::Text() const {
  std::ostringstream text;
  text << c_[2] << "x^2 + " << c_[1] << "x + " << c_[0];
  return text.str();
}

void CubicCurve::Init(const CubicInit& init) {
  //  f(x) = dx^3 + cx^2 + bx + a
  //
  // Solve for a and b by substituting with x = 0.
  //  y0 = f(0) = a
  //  s0 = f'(0) = b
  //
  // Solve for c and d by substituting with x = init.width_x = w. Gives two
  // linear equations with unknowns 'c' and 'd'.
  //  y1 = f(x1) = dw^3 + cw^2 + bw + a
  //  s1 = f'(x1) = 3dw^2 + 2cw + b
  //    ==> 3*y1 - w*s1 = (3dw^3 + 3cw^2 + 3bw + 3a) - (3dw^3 + 2cw^2 + bw)
  //        3*y1 - w*s1 = cw^2 - 2bw + 3a
  //               cw^2 = 3*y1 - w*s1 + 2bw - 3a
  //               cw^2 = 3*y1 - w*s1 + 2*s0*w - 3*y0
  //               cw^2 = 3(y1 - y0) - w*(s1 + 2*s0)
  //                  c = (3/w^2)*(y1 - y0) - (1/w)*(s1 + 2*s0)
  //    ==> 2*y1 - w*s1 = (2dw^3 + 2cw^2 + 2bw + 2a) - (3dw^3 + 2cw^2 + bw)
  //        2*y1 - w*s1 = -dw^3 + bw + 2a
  //               dw^3 = -2*y1 + w*s1 + bw + 2a
  //               dw^3 = -2*y1 + w*s1 + s0*w + 2*y0
  //               dw^3 = 2(y0 - y1) + w*(s1 + s0)
  //                  d = (2/w^3)*(y0 - y1) + (1/w^2)*(s1 + s0)
  const float one_over_w = init.width_x > 0.f ? (1.0f / init.width_x) : 1.f;
  const float one_over_w_sq = one_over_w * one_over_w;
  const float one_over_w_cubed = one_over_w_sq * one_over_w;
  c_[0] = init.start_y;
  c_[1] = init.width_x > 0.f ? init.start_derivative : 0.f;
  c_[2] = 3.0f * one_over_w_sq * (init.end_y - init.start_y) -
          one_over_w * (init.end_derivative + 2.0f * init.start_derivative);
  c_[3] = 2.0f * one_over_w_cubed * (init.start_y - init.end_y) +
          one_over_w_sq * (init.end_derivative + init.start_derivative);
}

void CubicCurve::ShiftLeft(const float x_shift) {
  // Early out optimization.
  if (x_shift == 0.0f) return;

  // s = x_shift
  // f(x) = dx^3 + cx^2 + bx + a
  // f(x + s) = d(x+s)^3 + c(x+s)^2 + b(x+s) + a
  //          = d(x^3 + 3sx^2 + 3s^2x + s^3) + c(x^2 + 2sx + s^2) + b(x + s) + a
  //          = dx^3 + (3sd + c)x^2 + (3ds^2 + 2c + b)x + (ds^3 + cs^2 + bs + a)
  //          = dx^3 + (f''(s)/2) x^2 + f'(s) x + f(s)
  //
  // Or, for an more general formulation, see:
  //     http://math.stackexchange.com/questions/694565/polynomial-shift
  const float new_c = SecondDerivative(x_shift) * 0.5f;
  const float new_b = Derivative(x_shift);
  const float new_a = Evaluate(x_shift);
  c_[0] = new_a;
  c_[1] = new_b;
  c_[2] = new_c;
}

bool CubicCurve::UniformCurvature(const Range& x_limits) const {
  // Curvature is given by the second derivative. The second derivative is
  // linear. So, the curvature is uniformly positive or negative iff
  //     Sign(f''(x_limits.start)) == Sign(f''(x_limits.end))
  const float epsilon = Epsilon();
  const float start_second_derivative =
      ClampNearZero(SecondDerivative(x_limits.start()), epsilon);
  const float end_second_derivative =
      ClampNearZero(SecondDerivative(x_limits.end()), epsilon);
  return start_second_derivative * end_second_derivative >= 0.0f;
}

bool CubicCurve::operator==(const CubicCurve& rhs) const {
  for (int i = 0; i < kNumCoeff; ++i) {
    if (c_[i] != rhs.c_[i]) return false;
  }
  return true;
}

std::string CubicCurve::Text() const {
  std::ostringstream text;
  text << c_[3] << "x^3 + " << c_[2] << "x^2 + " << c_[1] << "x + " << c_[0];
  return text.str();
}

// TODO: Move these to mathfu and templatize.
static inline int Round(float f) { return static_cast<int>(f + 0.5f); }

static inline vec2i Round(const vec2& v) {
  return vec2i(Round(v.x), Round(v.y));
}

static inline vec2 Min(const vec2& a, const vec2& b) {
  return vec2(std::min(a.x, b.x), std::min(a.y, b.y));
}

static inline vec2 Max(const vec2& a, const vec2& b) {
  return vec2(std::max(a.x, b.x), std::max(a.y, b.y));
}

static inline bool CompareBigYSmallX(const vec2i& a, const vec2i& b) {
  return a.y == b.y ? a.x < b.x : a.y > b.y;
}

std::string Graph2DPoints(const vec2* points, const int num_points,
                          const vec2i& size) {
#if defined(FPL_CURVE_GRAPH_FUNCTIONS)
  if (num_points == 0) return std::string();

  // Calculate x extents.
  vec2 min = points[0];
  vec2 max = points[0];
  for (const vec2* q = points + 1; q < &points[num_points]; ++q) {
    min = Min(min, *q);
    max = Max(max, *q);
  }
  const vec2 p_size = max - min;
  const vec2 gaps = vec2(size) - vec2(1.0f, 1.0f);
  const vec2 inc = p_size / gaps;
  const int zero_row = Round((0.0f - min.y) * size.y / p_size.y);

  // Convert to graph space on the screen.
  std::vector<vec2i> p;
  p.reserve(num_points);
  for (int i = 0; i < num_points; ++i) {
    p.push_back(Round((points[i] - min) / inc));
  }

  // Sort by Y, biggest to smallest, then by X smallest to largest.
  // This is the write order: top to bottom, left to right.
  std::sort(p.begin(), p.end(), CompareBigYSmallX);

  // Avoid reallocating the string by setting to a reasonable max size.
  std::string r;
  r.reserve(size.y * size.x + 100);

  // Iterate through each "pixel" of the graph.
  r += "y = " + std::to_string(static_cast<long double>(max.y)) + "\n";
  const vec2i* q = &p[0];
  for (int row = size.y; row >= 0; --row) {
    for (int col = 0; col <= size.x; ++col) {
      if (q->x == col && q->y == row) {
        r += '*';
        for (q++; q->x == col && q->y == row; q++) {}
        if (q > &p.back()) break;
      } else if (col == 0) {
        r += '|';
      } else if (row == zero_row) {
        r += '-';
      } else if (q->y < row) {
        break;
      } else {
        r += ' ';
      }
    }
    r += '\n';
    if (q > &p.back()) break;
  }
  r += "y = " + std::to_string(static_cast<long double>(min.y)) + "\n";
  return r;
#else
  (void)points;
  (void)num_points;
  (void)size;
  return std::string();
#endif  // defined(FPL_CURVE_GRAPH_FUNCTIONS)
}

}  // namespace motive
