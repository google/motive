/*
* Copyright (c) 2014 Google, Inc.
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "gtest/gtest.h"
#include "motive/math/bulk_spline_evaluator.h"
#include "motive/common.h"

using fpl::QuadraticCurve;
using fpl::CubicCurve;
using fpl::CubicInit;
using fpl::Range;
using mathfu::vec2;
using mathfu::vec2i;


class CurveTests : public ::testing::Test {
protected:
  virtual void SetUp() {}
  virtual void TearDown() {}
};

static void CheckQuadraticRoots(const QuadraticCurve& s,
                                size_t num_expected_roots) {
  // Ensure we have the correct number of roots.
  QuadraticCurve::RootsArray roots;
  s.Roots(&roots);
  EXPECT_EQ(num_expected_roots, roots.len);

  // Ensure roots are in ascending order.
  EXPECT_TRUE(roots.len < 2 || roots.arr[0] < roots.arr[1]);

  // Ensure roots all evaluate to zero.
  const float epsilon = s.Epsilon();
  for (size_t i = 0; i < roots.len; ++i) {
    const float should_be_zero = s.Evaluate(roots.arr[i]);
    EXPECT_LT(fabs(should_be_zero), epsilon);
  }
}

static void CheckCriticalPoint(const QuadraticCurve& s) {
  // Derivative should be zero at critical point.
  const float epsilon = s.Epsilon();
  const float critical_point_x = s.CriticalPoint();
  const float critical_point_derivative = s.Derivative(critical_point_x);
  EXPECT_LT(fabs(critical_point_derivative), epsilon);
}

TEST_F(CurveTests, QuadraticRoot_UpwardsAbove) {
  // Curves upwards, critical point above zero ==> no real roots.
  CheckQuadraticRoots(QuadraticCurve(60.0f, -32.0f, 6.0f), 0);
}

TEST_F(CurveTests, QuadraticRoot_UpwardsAt) {
  // Curves upwards, critical point at zero ==> one real roots.
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
  // Curves downwards, critical point above zero ==> two real roots.
  CheckQuadraticRoots(QuadraticCurve(-0.00006f, -0.000028f,
                                     -0.0000032666619999999896f), 1);
}

TEST_F(CurveTests, QuadraticRoot_DownwardsBelow) {
  // Curves downwards, critical point above zero ==> two real roots.
  CheckQuadraticRoots(QuadraticCurve(-0.00006f, -0.000028f, -0.000006f), 0);
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
  EXPECT_LT(fabs(c.Evaluate(init.width_x) - init.end_y), epsilon);
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

