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
#include "gtest/gtest.h"

using motive::CalculateQuadraticEaseInEaseOut;
using motive::CubicCurve;
using motive::CubicInit;
using motive::QuadraticCurve;
using motive::QuadraticEaseInEaseOut;
using motive::Range;

// The scale to be used to estimate error in our tests.
static const float kEpsilonScale = 0.0001f;

// The number of points we want to test on our curve.
static const float kNumTestPoints = 100.0f;

static void PrintCurveAsAsciiGraph(const QuadraticEaseInEaseOut& q) {
  printf("%s", GraphCurveOnXRange(q, motive::kCurveValue, Range(0, q.total_x()))
                   .c_str());
}

class CurveUtilTests : public ::testing::Test {
 protected:
  virtual void SetUp() {}
  virtual void TearDown() {}
};

static void TestEaseInEaseOut(float start_value, float start_derivative,
                              float start_second_derivative_abs,
                              float end_value, float end_derivative,
                              float end_second_derivative_abs,
                              float typical_total_x) {
  QuadraticEaseInEaseOut start_q = motive::CalculateQuadraticEaseInEaseOut(
      start_value, start_derivative, start_second_derivative_abs, end_value,
      end_derivative, end_second_derivative_abs, typical_total_x);
  PrintCurveAsAsciiGraph(start_q);

  // TODO(laijess): Epsilon values still need to be improved.
  // Calculate variables to use for measuring error.
  const float x_epsilon = start_q.total_x() * kEpsilonScale;
  const float value_epsilon =
      std::max(std::abs(start_value), std::abs(end_value)) * kEpsilonScale;
  const float avg_derivative =
      start_q.total_x() <= x_epsilon
          ? 0.0f
          : std::abs(end_value - start_value) / start_q.total_x();
  const float derivative_epsilon =
      std::max(std::max(avg_derivative, std::abs(start_derivative)),
               std::abs(end_derivative)) *
      kEpsilonScale;
  EXPECT_NEAR(start_q.in_curve().Evaluate(0.0f), start_value, value_epsilon);
  EXPECT_NEAR(start_q.in_curve().Derivative(0.0f), start_derivative,
              derivative_epsilon);

  // Equal in_curve and out_curve means that the desired end derivative is not
  // attainable. Therefore, we are prioritizing achieving the end value
  // and using just the in_curve.
  if (start_q.out_curve() == start_q.in_curve()) {
    EXPECT_NEAR(start_q.in_curve().Evaluate(start_q.total_x()), end_value,
                value_epsilon);
  } else {
    EXPECT_NEAR(start_q.out_curve().Evaluate(start_q.total_x()), end_value,
                value_epsilon);
    EXPECT_NEAR(start_q.out_curve().Derivative(start_q.total_x()),
                end_derivative, derivative_epsilon);
  }

  float delta_x = start_q.total_x() / (kNumTestPoints + 1);

  // Reevaluate curves, starting at different points on the start graph.
  float cur_value = start_q.Evaluate(delta_x);
  float cur_derivative = start_q.Derivative(delta_x);
  float cur_x = delta_x;
  for (;;) {
    QuadraticEaseInEaseOut q = motive::CalculateQuadraticEaseInEaseOut(
        cur_value, cur_derivative, start_second_derivative_abs, end_value,
        end_derivative, end_second_derivative_abs, typical_total_x);

    // Reevaluated curve should end and transition at the same time.
    EXPECT_NEAR(q.total_x() + cur_x, start_q.total_x(), x_epsilon);
    if (q.intersection_x() > x_epsilon) {
      EXPECT_NEAR(q.intersection_x() + cur_x, start_q.intersection_x(),
                  x_epsilon);
    } else {
      EXPECT_LE(q.intersection_x(), x_epsilon);
    }

    // Values in the curves should match.
    for (float x = 0.0f; x <= q.total_x(); x += delta_x) {
      if (delta_x == 0.0f) break;
      const float start_x = cur_x + x;
      EXPECT_NEAR(q.Evaluate(x), start_q.Evaluate(start_x), value_epsilon);
      EXPECT_NEAR(q.Derivative(x), start_q.Derivative(start_x),
                  derivative_epsilon);
    }
    if (q.out_curve() == q.in_curve()) {
      EXPECT_NEAR(q.in_curve().Evaluate(q.total_x()), end_value, value_epsilon);
    } else {
      EXPECT_NEAR(q.out_curve().Evaluate(q.total_x()), end_value,
                  value_epsilon);
      EXPECT_NEAR(q.out_curve().Derivative(q.total_x()), end_derivative,
                  derivative_epsilon);
    }

    // Advance the curve by delta_x.
    cur_value = q.Evaluate(delta_x);
    cur_derivative = q.Derivative(delta_x);
    cur_x += delta_x;
    if (cur_x >= start_q.total_x() || q.total_x() == 0.0f ||
        cur_x / delta_x >= kNumTestPoints)
      break;
  }
}

// Super-simple ease in ease out, start and end derivatives 0.
TEST_F(CurveUtilTests, EaseInEaseOutZeroToOne) {
  TestEaseInEaseOut(0.0f, 0.0f, 0.1f, 1.0f, 0.0f, 0.1f, 6.4f);
}

// Fast ease-in and slow ease-out.
TEST_F(CurveUtilTests, FastInSlowOut) {
  TestEaseInEaseOut(0.0f, 0.0f, 0.2f, 1.0f, 0.0f, 0.05f, 7.1f);
}

// Slow ease-in and fast ease-out.
TEST_F(CurveUtilTests, SlowInFastOut) {
  TestEaseInEaseOut(0.0f, 0.0f, 0.01f, 1.0f, 0.0f, 0.1f, 14.9f);
}

// Really small second derivatives.
TEST_F(CurveUtilTests, LongTime) {
  TestEaseInEaseOut(0.0f, 0.0f, 0.0001f, 1.0f, 0.0f, 0.0003f, 163.3f);
}

// Larger second derivatives.
TEST_F(CurveUtilTests, ShortTime) {
  TestEaseInEaseOut(0.0f, 0.0f, 0.6f, 1.0f, 0.0f, 0.8f, 2.5f);
}

// Simple fast ease in ease out.
TEST_F(CurveUtilTests, FastInFastOut) {
  TestEaseInEaseOut(0.0f, 0.0f, 0.8f, 1.0f, 0.0f, 0.8f, 2.3f);
}

// Has a non-zero end derivative.
TEST_F(CurveUtilTests, EndNonzeroFirst) {
  TestEaseInEaseOut(0.0f, 0.0f, 0.1f, 1.0f, 0.2f, 0.1f, 5.0f);
}

// Has a non-zero start derivative.
TEST_F(CurveUtilTests, StartNonzeroFirst) {
  TestEaseInEaseOut(0.0f, 0.2f, 0.1f, 1.0f, 0.0f, 0.1f, 5.0f);
}

// Similar start and end with large start derivative.
TEST_F(CurveUtilTests, CloseStartEndLargeStartDerivative) {
  TestEaseInEaseOut(0.3f, 1.3f, 0.1f, .32f, 0.0f, 0.1f, 13.0f);
}

// Fast ease-in and slow ease-out when points are very close.
TEST_F(CurveUtilTests, FastInSlowOutCloseStartEnd) {
  TestEaseInEaseOut(0.1f, 0.0f, 0.2f, 0.2f, 0.0f, 0.05f, 2.3f);
}

// Slow ease-in and fast ease-out when points are very close.
TEST_F(CurveUtilTests, SlowInFastOutCloseStartEnd) {
  TestEaseInEaseOut(0.1f, 0.0f, 0.01f, 0.12f, 0.0f, 0.1f, 2.1f);
}

// Super-simple ease in ease out, start and end derivatives 0.
TEST_F(CurveUtilTests, EaseInEaseOutOneToZero) {
  TestEaseInEaseOut(1.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.1f, 6.4f);
}

// Fast ease-in and slow ease-out.
TEST_F(CurveUtilTests, FastInSlowOutOneToZero) {
  TestEaseInEaseOut(1.0f, 0.0f, 0.2f, 0.0f, 0.0f, 0.05f, 7.1f);
}

// Fast test with large start derivative.
TEST_F(CurveUtilTests, ShortTimeLargeStartDerivative) {
  TestEaseInEaseOut(0.0f, 0.5f, 0.2f, 1.0f, 0.0f, 0.8f, 2.1f);
}

// Long test, start and end close.
TEST_F(CurveUtilTests, LongTimeCloseStartEnd) {
  TestEaseInEaseOut(0.0f, 0.0f, 0.0001f, 0.15f, 0.0f, 0.0003f, 63.3f);
}

// Negative derivatives.
TEST_F(CurveUtilTests, NegativeStartDerivative) {
  TestEaseInEaseOut(0.0f, -0.1f, 0.1f, 1.0f, 0.1f, 0.1f, 6.7f);
}

// // Negative derivatives.
TEST_F(CurveUtilTests, ReallyNegativeStartDerivative) {
  TestEaseInEaseOut(0.0f, -0.9f, 0.1f, 1.0f, 0.1f, 0.1f, 22.3f);
}

// Large start derivative.
TEST_F(CurveUtilTests, LargeStartDerivative) {
  TestEaseInEaseOut(0.0f, 0.9f, 0.1f, 1.0f, 0.0f, 0.1f, 9.0f);
}

// Slow test with large end derivative.
TEST_F(CurveUtilTests, LongTimeLargeEndDerivative) {
  TestEaseInEaseOut(0.0f, 0.0f, 0.0001f, 1.0f, 0.9f, 0.0003f, 6001.2f);
}

// Similar start and end with large end derivative.
TEST_F(CurveUtilTests, CloseStartEndLargeEndDerivative) {
  TestEaseInEaseOut(0.9f, 0.0f, 0.1f, 1.0f, 0.9f, 0.1f, 9.0f);
}

// Slow in with big start derivative.
TEST_F(CurveUtilTests, SlowInLargeStartDerivative) {
  TestEaseInEaseOut(0.0f, 0.9f, 0.0001f, 1.0f, 0.0f, 0.1f, 12.4f);
}

// Similar start and end with large end derivative and nonzero start.
TEST_F(CurveUtilTests, CloseStartEndLargeEndNonzeroStartDerivative) {
  TestEaseInEaseOut(0.9f, 0.3f, 0.1f, 1.0f, 0.9f, 0.1f, 6.0f);
}

// Large end derivative.
TEST_F(CurveUtilTests, LargeEndDerivative) {
  TestEaseInEaseOut(0.0f, 0.0f, 0.1f, 1.0f, 0.9f, 0.1f, 9.0f);
}
//
//// Same start and end with time between.
TEST_F(CurveUtilTests, SameStartEndTime) {
  TestEaseInEaseOut(1.0f, -0.1f, 0.1f, 1.0f, -0.1f, 0.1f, 4.0f);
}

// NOTE: The tests below this comment are still failing.

// NOTE: This test fails by a very small margin.
// Long test, similar start and end with large start derivative.
// TEST_F(CurveUtilTests, LongTimeCloseStartEndLargeStartDerivative) {
//  TestEaseInEaseOut(0.3f, 0.9f, 0.001f, 0.32f, 0.0f, 0.001f);
//}

// NOTE: This test fails by a very small margin.
// Slow test with big start derivative.
// TEST_F(CurveUtilTests, LongTimeLargeStartDerivative) {
//  TestEaseInEaseOut(0.0f, 0.9f, 0.0001f, 1.0f, 0.0f, 0.0003f);
//}

// NOTE: This test fails because its intersection_x is wrong.
// NOTE: This test is also made to fail when intersection_x < 0.0 is
// considered an extreme case.
// NOTE: In this test, we flip curvature when the curve
// is trying to go above end_value to get a negative
// end derivative at the end_value.
// Negative derivatives.
// TEST_F(CurveUtilTests, NegativeEndDerivative) {
//   TestEaseInEaseOut(0.0f, 0.1f, 0.1f, 1.0f, -0.1f, 0.1f);
// }

// Negative derivatives.
// TEST_F(CurveUtilTests, ReallyNegativeEndDerivative) {
//   TestEaseInEaseOut(0.0f, 0.1f, 0.1f, 1.0f, -0.9f, 0.1f);
// }

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
