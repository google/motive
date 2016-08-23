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

#include "gtest/gtest.h"
#include "motive/math/bulk_spline_evaluator.h"
#include "motive/common.h"

using motive::QuadraticCurve;
using motive::QuadraticInitWithStartDerivative;
using motive::QuadraticInitWithOrigin;
using motive::QuadraticInitWithPoint;
using motive::CubicCurve;
using motive::CubicInit;
using motive::Range;
using mathfu::vec2;
using mathfu::vec2i;

static const float kMaxFloat = std::numeric_limits<float>::max();
static const float kMinFloat = std::numeric_limits<float>::min();

class CurveTests : public ::testing::Test {
protected:
  virtual void SetUp() {}
  virtual void TearDown() {}
};

TEST_F(CurveTests, QuadraticInitWithStartDerivative) {
  const QuadraticInitWithStartDerivative init(1.0f, 0.3f, 2.2f);
  const QuadraticCurve c(init);
  EXPECT_EQ(c.Evaluate(0.0f), init.start_y);
  EXPECT_EQ(c.Evaluate(1.0f), init.end_y);
  EXPECT_EQ(c.Derivative(0.0f), init.start_derivative);
}

TEST_F(CurveTests, QuadraticInitWithOrigin) {
  const QuadraticInitWithOrigin init(-10.0f, 3.2f, 0.2f);
  const QuadraticCurve c(init);
  EXPECT_EQ(c.Evaluate(0.0f), init.y);
  EXPECT_EQ(c.Derivative(0.0f), init.derivative);
  EXPECT_EQ(c.SecondDerivative(), init.second_derivative);
}

TEST_F(CurveTests, QuadraticInitWithPoint) {
  const QuadraticInitWithPoint init(3.0f, 1.7f, 0.33f, 0.01f);
  const QuadraticCurve c(init);
  EXPECT_EQ(c.Evaluate(init.x), init.y_at_x);
  EXPECT_EQ(c.Derivative(init.x), init.derivative_at_x);
  EXPECT_EQ(c.SecondDerivative(), init.second_derivative);
}

TEST_F(CurveTests, Plus) {
  const QuadraticCurve c1(1.0f, -2.0f, 3.0f);
  const QuadraticCurve c2(2.1f, -0.3f, 0.0001f);
  QuadraticCurve c = c1;
  c += c2;
  for (int i = 0; i < QuadraticCurve::kNumCoeff; ++i) {
    EXPECT_EQ(c.Coeff(i), c1.Coeff(i) + c2.Coeff(i));
  }
}

TEST_F(CurveTests, Minus) {
  const QuadraticCurve c1(1.0f, -2.0f, 3.0f);
  const QuadraticCurve c2(2.1f, -0.3f, 0.0001f);
  QuadraticCurve c = c1;
  c -= c2;
  for (int i = 0; i < QuadraticCurve::kNumCoeff; ++i) {
    EXPECT_EQ(c.Coeff(i), c1.Coeff(i) - c2.Coeff(i));
  }
}

TEST_F(CurveTests, PlusExternal) {
  const QuadraticCurve c1(1.0f, -2.0f, 3.0f);
  const QuadraticCurve c2(2.1f, -0.3f, 0.0001f);
  const QuadraticCurve c = c1 + c2;
  for (int i = 0; i < QuadraticCurve::kNumCoeff; ++i) {
    EXPECT_EQ(c.Coeff(i), c1.Coeff(i) + c2.Coeff(i));
  }
}

TEST_F(CurveTests, MinusExternal) {
  const QuadraticCurve c1(1.0f, -2.0f, 3.0f);
  const QuadraticCurve c2(2.1f, -0.3f, 0.0001f);
  const QuadraticCurve c = c1 - c2;
  for (int i = 0; i < QuadraticCurve::kNumCoeff; ++i) {
    EXPECT_EQ(c.Coeff(i), c1.Coeff(i) - c2.Coeff(i));
  }
}

static void CheckQuadraticRoots(const QuadraticCurve& s,
                                size_t num_expected_roots) {
  // Ensure we have the correct number of roots.
  QuadraticCurve::RootsArray roots;
  s.Roots(&roots);
  EXPECT_EQ(num_expected_roots, roots.len);

  // Ensure roots are in ascending order.
  EXPECT_TRUE(roots.len < 2 || roots.arr[0] < roots.arr[1]);

  // Ensure roots all evaluate to zero.
  for (size_t i = 0; i < roots.len; ++i) {
    float should_be_zero = s.Evaluate(roots.arr[i]);
    float epsilon = s.EpsilonInRange(roots.arr[i]);

    // If the quadratic has crazy coefficients and evalues to infinity,
    // scale it down in y. The roots should be the same.
    // Note that we don't want to do this in general because it's a less
    // accurate test.
    if (std::fabs(should_be_zero) > std::numeric_limits<float>::max()) {
      const motive::QuadraticCurve s_shrunk(s, 1.0f / s.MaxCoeff());
      should_be_zero = s_shrunk.Evaluate(roots.arr[i]);
      epsilon = s_shrunk.EpsilonInRange(roots.arr[i]);
    }

    EXPECT_NEAR(should_be_zero, 0.0f, epsilon);
  }
}

static void CheckCriticalPoint(const QuadraticCurve& s) {
  // Derivative should be zero at critical point.
  const float critical_point_x = s.CriticalPoint();
  const float critical_point_derivative = s.Derivative(critical_point_x);
  const float epsilon = s.EpsilonInRange(critical_point_x);
  EXPECT_LT(std::fabs(critical_point_derivative), epsilon);
}

// Test for some coefficients as max float, one solution.
TEST_F(CurveTests, QuadraticRoot_OneMaxOneSolution) {
  CheckQuadraticRoots(QuadraticCurve(kMaxFloat, 0.0f, 0.0f), 1);
}

// Test for all coefficients as max float, two solutions.
TEST_F(CurveTests, QuadraticRoot_AllMaxTwoSolutions) {
  CheckQuadraticRoots(QuadraticCurve(kMaxFloat, kMaxFloat, -kMaxFloat), 2);
}

// Test for some coefficients as max float, two solutions.
TEST_F(CurveTests, QuadraticRoot_TwoMaxTwoSolutions) {
  CheckQuadraticRoots(QuadraticCurve(kMaxFloat, kMaxFloat, -1.0f), 2);
}

// Test for all coefficients as max float, no solutions.
TEST_F(CurveTests, QuadraticRoot_AllMaxNoSolutions) {
  CheckQuadraticRoots(QuadraticCurve(kMaxFloat, kMaxFloat, kMaxFloat), 0);
}

// Test for all coefficients as min float, no solutions.
TEST_F(CurveTests, QuadraticRoot_AllMinNoSolutions) {
  CheckQuadraticRoots(QuadraticCurve(kMinFloat, kMinFloat, kMinFloat), 0);
}

// Test for all coefficients as min float, two solutions.
TEST_F(CurveTests, QuadraticRoot_AllMinTwoSolutions) {
  CheckQuadraticRoots(QuadraticCurve(-kMinFloat, kMinFloat, kMinFloat), 2);
}

// Test for one coefficient as min float, one solution.
TEST_F(CurveTests, QuadraticRoot_OneMinOneSolution) {
  CheckQuadraticRoots(QuadraticCurve(-kMinFloat, 0.0f, 0.0f), 1);
}

// Test for one coefficient as min float, one solution.
TEST_F(CurveTests, QuadraticRoot_MinMaxMixOneSolution) {
  CheckQuadraticRoots(QuadraticCurve(-kMinFloat, kMaxFloat, 1.0f), 1);
}

// Test for one coefficient as min float, two solutions.
TEST_F(CurveTests, QuadraticRoot_MaxMinMixOneSolution) {
  CheckQuadraticRoots(QuadraticCurve(kMaxFloat, -kMinFloat, 0.0f), 1);
}

// Test for zeros everywhere but the constant component.
TEST_F(CurveTests, QuadraticRoot_Constant) {
  CheckQuadraticRoots(QuadraticCurve(0.0f, 0.0f, 1.0f), 0);
}

// Test for zeros everywhere but the linear component.
TEST_F(CurveTests, QuadraticRoot_Linear) {
  CheckQuadraticRoots(QuadraticCurve(0.0f, 1.0f, 0.0f), 1);
}

// Test for zeros everywhere but the quadratic component.
TEST_F(CurveTests, QuadraticRoot_Quadratic) {
  CheckQuadraticRoots(QuadraticCurve(1.0f, 0.0f, 0.0f), 1);
}

TEST_F(CurveTests, QuadraticRoot_UpwardsAbove) {
  // Curves upwards, critical point above zero ==> no real roots.
  CheckQuadraticRoots(QuadraticCurve(60.0f, -32.0f, 6.0f), 0);
}

TEST_F(CurveTests, QuadraticRoot_UpwardsAt) {
  // Curves upwards, critical point at zero ==> one real root.
  CheckQuadraticRoots(QuadraticCurve(60.0f, -32.0f, 4.26666689f), 1);
}

TEST_F(CurveTests, QuadraticRoot_UpwardsBelow) {
  // Curves upwards, critical point below zero ==> two real roots.
  CheckQuadraticRoots(QuadraticCurve(60.0f, -32.0f, 4.0f), 2);
}

TEST_F(CurveTests, QuadraticRoot_DownwardsAbove) {
  // Curves downwards, critical point above zero ==> two real roots.
  CheckQuadraticRoots(QuadraticCurve(-0.00006f, -0.000028f, 0.0001f), 2);
}

TEST_F(CurveTests, QuadraticRoot_DownwardsAt) {
  // Curves downwards, critical point at zero ==> one real root at critical
  // point.
  CheckQuadraticRoots(QuadraticCurve(-0.00006f, -0.000028f, -0.00000326666691f),
                      1);
}

TEST_F(CurveTests, QuadraticRoot_DownwardsBelow) {
  // Curves downwards, critical point below zero ==> no real roots.
  CheckQuadraticRoots(QuadraticCurve(-0.00006f, -0.000028f, -0.000006f), 0);
}

TEST_F(CurveTests, QuadraticRoot_AllTinyCoefficients) {
  // Curves upwards, critical point below zero ==> two real roots.
  CheckQuadraticRoots(
      QuadraticCurve(0.000000006f, -0.0000000032f, 0.0000000004f), 2);
}

TEST_F(CurveTests, QuadraticRoot_SmallSquareCoefficient) {
  CheckQuadraticRoots(QuadraticCurve(-0.00000003f, 0.0f, 0.0008f), 2);
}

TEST_F(CurveTests, QuadraticRoot_TinySquareCoefficient) {
  CheckQuadraticRoots(QuadraticCurve(0.000000001f, 1.0f, -0.00000001f), 2);
}

TEST_F(CurveTests, QuadraticCriticalPoint) {
  // Curves upwards, critical point above zero ==> no real roots.
  CheckCriticalPoint(QuadraticCurve(60.0f, -32.0f, 6.0f));
}

TEST_F(CurveTests, QuadraticRangesMatchingSign_SmallValues) {
  const Range limits(0.0f, 1.0f);
  const QuadraticCurve small(1.006107e-11f, -3.01832101e-11f, 1.006107e-11f);

  QuadraticCurve::RangeArray matching;
  small.RangesMatchingSign(limits, 1.0f, &matching);
  EXPECT_EQ(matching.len, 1u);
}

TEST_F(CurveTests, CubicWithWidth) {
  const CubicInit init(1.0f, -8.0f, 0.3f, -4.0f, 1.0f);
  const CubicCurve c(init);
  const float epsilon = c.Epsilon();
  EXPECT_LT(std::fabs(c.Evaluate(init.width_x) - init.end_y), epsilon);
}

typedef void ShiftFn(float shift, CubicCurve* c);
static void ShiftLeft(float shift, CubicCurve* c) { c->ShiftLeft(shift); }
static void ShiftRight(float shift, CubicCurve* c) { c->ShiftRight(shift); }

static void TestShift(const CubicInit& init, float shift, ShiftFn* fn,
                      float direction) {
  const CubicCurve c(init);
  CubicCurve shifted(c);
  fn(shift, &shifted);

  const float epsilon = shifted.Epsilon();
  const float offset = direction * shift;
  EXPECT_NEAR(c.Evaluate(0.0f), shifted.Evaluate(offset), epsilon);
  EXPECT_NEAR(c.Evaluate(-offset), shifted.Evaluate(0.0f), epsilon);
  EXPECT_NEAR(c.Derivative(0.0f), shifted.Derivative(offset), epsilon);
  EXPECT_NEAR(c.Derivative(-offset), shifted.Derivative(0.0f), epsilon);
  EXPECT_NEAR(c.SecondDerivative(0.0f), shifted.SecondDerivative(offset),
              epsilon);
  EXPECT_NEAR(c.SecondDerivative(-offset), shifted.SecondDerivative(0.0f),
              epsilon);
}

static void TestShiftLeft(const CubicInit& init, float shift) {
  TestShift(init, shift, ShiftLeft, -1.0f);
}

static void TestShiftRight(const CubicInit& init, float shift) {
  TestShift(init, shift, ShiftRight, 1.0f);
}

TEST_F(CurveTests, CubicShiftLeft) {
  const CubicInit init(1.0f, -8.0f, 0.3f, -4.0f, 1.0f);
  TestShiftLeft(init, 0.0f);
  TestShiftLeft(init, 1.0f);
  TestShiftLeft(init, -0.1f);
  TestShiftLeft(init, 0.00001f);
  TestShiftLeft(init, 10.0f);
}

TEST_F(CurveTests, CubicShiftRight) {
  const CubicInit init(1.0f, -8.0f, 0.3f, -4.0f, 1.0f);
  TestShiftRight(init, 0.0f);
  TestShiftRight(init, 1.0f);
  TestShiftRight(init, -0.1f);
  TestShiftRight(init, 0.00001f);
  TestShiftRight(init, 10.0f);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

