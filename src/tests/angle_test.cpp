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

#define FPL_ANGLE_UNIT_TESTS

#include <string>
#include "motive/math/angle.h"
#include "motive/common.h"
#include "gtest/gtest.h"

using fpl::Angle;
using fpl::AngleToVectorSystem;
using fpl::kPi;
using fpl::kHalfPi;
using fpl::kQuarterPi;
using fpl::kMinUniqueAngle;
using fpl::kMaxUniqueAngle;
using mathfu::vec3;
using mathfu::mat3;

union FloatInt {
  float f;
  int i;
};

static const float kAnglePrecision = 0.0000005f;
static const float kUnitVectorPrecision = 0.0000005f;

// When diff is +-1, returns the smallest float value that is different than f.
// Note: You can construct examples where this function fails. But it works in
// almost all cases, and is good for the values we use in these tests.
static float MinutelyDifferentFloat(float f, int diff) {
  FloatInt u;
  u.f = f;
  u.i += diff;
  return u.f;
}

class AngleTests : public ::testing::Test {
protected:
  virtual void SetUp()
  {
    above_pi_ = MinutelyDifferentFloat(kPi, 1);
    below_pi_ = MinutelyDifferentFloat(kPi, -1);
    above_negative_pi_ = MinutelyDifferentFloat(-kPi, -1);
    below_negative_pi_ = MinutelyDifferentFloat(-kPi, 1);
    half_pi_ = Angle(kHalfPi);
  }
  virtual void TearDown() {}

protected:
  float above_pi_;
  float below_pi_;
  float above_negative_pi_;
  float below_negative_pi_;
  Angle half_pi_;
};

// Ensure the constants are in (or not in) the valid angle range.
TEST_F(AngleTests, RangeExtremes) {
  EXPECT_TRUE(Angle::IsAngleInRange(kPi));
  EXPECT_FALSE(Angle::IsAngleInRange(-kPi));
  EXPECT_TRUE(Angle::IsAngleInRange(kMinUniqueAngle));
  EXPECT_TRUE(Angle::IsAngleInRange(kMaxUniqueAngle));
}

// Ensure constant values are what we expect them to be.
TEST_F(AngleTests, RangeConstants) {
  EXPECT_FLOAT_EQ(above_negative_pi_, kMinUniqueAngle);
  EXPECT_FLOAT_EQ(static_cast<float>(M_PI), kMaxUniqueAngle);
}

// Ensure the smallest value above pi is false.
TEST_F(AngleTests, AbovePi) {
  EXPECT_FALSE(Angle::IsAngleInRange(above_pi_));
}

// Ensure the smallest value above -pi is true.
TEST_F(AngleTests, AboveNegativePi) {
  EXPECT_TRUE(Angle::IsAngleInRange(above_negative_pi_));
}

// -pi should be represented as pi.
TEST_F(AngleTests, ModFromNegativePi) {
  EXPECT_FLOAT_EQ(Angle::FromWithinThreePi(-kPi).ToRadians(), kPi);
}

// pi should be represented as pi.
TEST_F(AngleTests, ModFromPositivePi) {
  EXPECT_FLOAT_EQ(Angle::FromWithinThreePi(kPi).ToRadians(), kPi);
}

// Slightly below -pi should mod to near pi.
TEST_F(AngleTests, ModBelowNegativePi) {
  const Angle a = Angle::FromWithinThreePi(below_negative_pi_);
  EXPECT_TRUE(a.IsValid());
}

// Slightly above pi should mod to near -pi (but above).
TEST_F(AngleTests, ModAbovePi) {
  const Angle a = Angle::FromWithinThreePi(above_pi_);
  EXPECT_TRUE(a.IsValid());
}

// Addition should use modular arithmetic.
TEST_F(AngleTests, Addition) {
  const Angle sum = half_pi_ + half_pi_ + half_pi_ + half_pi_;
  EXPECT_NEAR(sum.ToRadians(), 0.0f, kAnglePrecision);
}

// Subtraction should use modular arithmetic.
TEST_F(AngleTests, Subtraction) {
  const Angle diff = half_pi_ - half_pi_ - half_pi_ - half_pi_;
  EXPECT_NEAR(diff.ToRadians(), kPi, kAnglePrecision);
}

// Addition should use modular arithmetic.
TEST_F(AngleTests, Multiplication) {
  const Angle product = half_pi_ * 3.f;
  EXPECT_NEAR(product.ToRadians(), -half_pi_.ToRadians(), kAnglePrecision);
}

// Subtraction should use modular arithmetic.
TEST_F(AngleTests, Division) {
  const Angle quotient = Angle::FromWithinThreePi(kPi) / 2.0f;
  EXPECT_NEAR(quotient.ToRadians(), kHalfPi, kAnglePrecision);
}

// Unary negate should change the sign.
TEST_F(AngleTests, Negate) {
  const Angle a = Angle(kHalfPi);
  EXPECT_FLOAT_EQ(-a.ToRadians(), -kHalfPi);
}

// Unary negate should send pi to pi, because -pi is not in range.
TEST_F(AngleTests, NegatePi) {
  const Angle a = Angle(kPi);
  const Angle negative_a = -a;
  EXPECT_FLOAT_EQ(negative_a.ToRadians(), kPi);
}

// Ensure wrapping produces angles in the range (-pi, pi].
TEST_F(AngleTests, WrapAngleTest) {
  const float a1 = Angle::WrapAngle(
      static_cast<float>(-M_PI - M_2_PI - M_2_PI));
  EXPECT_TRUE(Angle::IsAngleInRange(a1));

  const float a2 = Angle::WrapAngle(static_cast<float>(-M_PI - M_2_PI));
  EXPECT_TRUE(Angle::IsAngleInRange(a2));

  const float a3 = Angle::WrapAngle(static_cast<float>(-M_PI));
  EXPECT_TRUE(Angle::IsAngleInRange(a3));

  const float a4 = Angle::WrapAngle(0.f);
  EXPECT_TRUE(Angle::IsAngleInRange(a4));

  const float a5 = Angle::WrapAngle(static_cast<float>(M_PI + M_2_PI));
  EXPECT_TRUE(Angle::IsAngleInRange(a5));

  const float a6 = Angle::WrapAngle(
      static_cast<float>(M_PI + M_2_PI + M_2_PI));
  EXPECT_TRUE(Angle::IsAngleInRange(a6));
}

// Clamping a value that's inside the range should not change the value.
TEST_F(AngleTests, ClampInside) {
  const Angle a(kHalfPi + 0.1f);
  const Angle center(kHalfPi);
  const Angle max_diff(0.2f);
  EXPECT_FLOAT_EQ(a.Clamp(center, max_diff).ToRadians(), a.ToRadians());
}

// Clamping a value that's above the range should clamp to the top boundary.
TEST_F(AngleTests, ClampAbove) {
  const Angle a(kHalfPi + 0.2f);
  const Angle center(kHalfPi);
  const Angle max_diff(0.1f);
  EXPECT_FLOAT_EQ(a.Clamp(center, max_diff).ToRadians(),
                  (center + max_diff).ToRadians());
}

// Clamping a value that's below the range should clamp to the bottom boundary.
TEST_F(AngleTests, ClampBelow) {
  const Angle a(-kHalfPi - 0.2f);
  const Angle center(-kHalfPi);
  const Angle max_diff(0.1f);
  EXPECT_FLOAT_EQ(a.Clamp(center, max_diff).ToRadians(),
                  (center - max_diff).ToRadians());
}

// Clamping to a range that strattles pi should wrap to the boundary that's
// closest under modular arithmetic.
TEST_F(AngleTests, ClampModularAtPositiveCenterPositiveAngle) {
  const Angle a(kPi - 0.2f);
  const Angle center(kPi);
  const Angle max_diff(0.1f);
  // This tests a positive number clamped to the range.
  EXPECT_FLOAT_EQ(a.Clamp(center, max_diff).ToRadians(),
                  (center - max_diff).ToRadians());
}

// Clamping to a range that strattles pi should wrap to the boundary that's
// closest under modular arithmetic.
TEST_F(AngleTests, ClampModularAtPositiveCenterNegativeAngle) {
  const Angle a(-kPi + 1.1f);
  const Angle center(kPi);
  const Angle max_diff(0.1f);
  // This tests a negative number clamped to the range.
  EXPECT_FLOAT_EQ(a.Clamp(center, max_diff).ToRadians(),
                  (center + max_diff).ToRadians());
}

// Clamping to a range that strattles pi should wrap to the boundary that's
// closest under modular arithmetic.
TEST_F(AngleTests, ClampModularAtNegativeCenterPositiveAngle) {
  const Angle a(kPi - 0.2f);
  const Angle center(kMinUniqueAngle);
  const Angle max_diff(0.1f);
  // This tests a positive number clamped to a range centered about a negative
  // number.
  EXPECT_FLOAT_EQ(a.Clamp(center, max_diff).ToRadians(),
                  (center - max_diff).ToRadians());
}

// Clamping to a range that strattles pi should wrap to the boundary that's
// closest under modular arithmetic.
TEST_F(AngleTests, ClampModularAtNegativeCenterNegativeAngle) {
  const Angle a(-kPi + 1.1f);
  const Angle center(kMinUniqueAngle);
  const Angle max_diff(0.1f);
  // This tests a negative number clamped to the range.
  EXPECT_FLOAT_EQ(a.Clamp(center, max_diff).ToRadians(),
                  (center + max_diff).ToRadians());
}

// Clamping with zero diff should return the center.
TEST_F(AngleTests, ClampWithZeroDiff) {
  const Angle a(-kPi + 1.1f);
  const Angle center(kPi - 2.1f);
  const Angle max_diff(0.0f);
  // This tests a negative number clamped to the range.
  EXPECT_FLOAT_EQ(a.Clamp(center, max_diff).ToRadians(), center.ToRadians());
}

void TestToVectorSystem(const AngleToVectorSystem system, const int zero_axis,
                        const int ninety_axis, const int ignored_axis) {
  // Angle zero should translate to the zero axis.
  const vec3 zero = Angle(0.0f).ToVectorSystem(system);
  EXPECT_NEAR(zero[zero_axis], 1.0f, kUnitVectorPrecision);
  EXPECT_NEAR(zero[ninety_axis], 0.0f, kUnitVectorPrecision);
  EXPECT_NEAR(zero[ignored_axis], 0.0f, kUnitVectorPrecision);

  // Angle 180 should translate to the negative zero axis.
  const vec3 minus_zero = Angle(kPi).ToVectorSystem(system);
  EXPECT_NEAR(minus_zero[zero_axis], -1.0f, kUnitVectorPrecision);
  EXPECT_NEAR(minus_zero[ninety_axis], 0.0f, kUnitVectorPrecision);
  EXPECT_NEAR(minus_zero[ignored_axis], 0.0f, kUnitVectorPrecision);

  // Angle 90 should translate to the 90 axis.
  const vec3 ninety = Angle(kHalfPi).ToVectorSystem(system);
  EXPECT_NEAR(ninety[zero_axis], 0.0f, kUnitVectorPrecision);
  EXPECT_NEAR(ninety[ninety_axis], 1.0f, kUnitVectorPrecision);
  EXPECT_NEAR(ninety[ignored_axis], 0.0f, kUnitVectorPrecision);

  // Angle -90 should translate to the -90 axis.
  const vec3 minus_ninety = Angle(-kHalfPi).ToVectorSystem(system);
  EXPECT_NEAR(minus_ninety[zero_axis], 0.0f, kUnitVectorPrecision);
  EXPECT_NEAR(minus_ninety[ninety_axis], -1.0f, kUnitVectorPrecision);
  EXPECT_NEAR(minus_ninety[ignored_axis], 0.0f, kUnitVectorPrecision);

  // Angle 45 should translate to a unit vector between the 0 and 90 axes.
  const vec3 forty_five = Angle(kQuarterPi).ToVectorSystem(system);
  const float one_over_root_two = 1.0f / sqrt(2.0f);
  EXPECT_NEAR(forty_five[zero_axis], one_over_root_two, kUnitVectorPrecision);
  EXPECT_NEAR(forty_five[ninety_axis], one_over_root_two, kUnitVectorPrecision);
  EXPECT_NEAR(forty_five[ignored_axis], 0.0f, kUnitVectorPrecision);
}

// Test conversions from angles to vectors.
TEST_F(AngleTests, ToVectorSystem) {
  TestToVectorSystem(fpl::kAngleToVectorXY, 0, 1, 2);
  TestToVectorSystem(fpl::kAngleToVectorXZ, 0, 2, 1);
  TestToVectorSystem(fpl::kAngleToVectorYZ, 1, 2, 0);
  TestToVectorSystem(fpl::kAngleToVectorYX, 1, 0, 2);
  TestToVectorSystem(fpl::kAngleToVectorZX, 2, 0, 1);
  TestToVectorSystem(fpl::kAngleToVectorZY, 2, 1, 0);
}

static const float kIgnoredAxisValues[] = { 0.0f, 1.0f, -10.0f, 300000.0f };
void TestFromVectorSystem(const AngleToVectorSystem system, const int zero_axis,
                          const int ninety_axis, const int ignored_axis) {
  for (size_t i = 0; i < MOTIVE_ARRAY_SIZE(kIgnoredAxisValues); ++i) {
    // No matter what the ignored axis is, the returned value should be the
    // same.
    vec3 init_vector = mathfu::kZeros3f;
    init_vector[ignored_axis] = kIgnoredAxisValues[i];

    // Zero axis should convert to zero angle.
    vec3 zero = init_vector;
    zero[zero_axis] = 1.0f;
    const Angle zero_angle = Angle::FromVectorSystem(zero, system);
    EXPECT_NEAR(zero_angle.ToRadians(), 0.0f, kAnglePrecision);

    // Ninety axis should convert to ninety angle.
    vec3 ninety = init_vector;
    ninety[ninety_axis] = 1.0f;
    const Angle ninety_angle = Angle::FromVectorSystem(ninety, system);
    EXPECT_NEAR(ninety_angle.ToRadians(), kHalfPi, kAnglePrecision);

    // Negative zero axis should convert to 180 angle.
    vec3 neg_zero = init_vector;
    neg_zero[zero_axis] = -1.0f;
    const Angle neg_zero_angle = Angle::FromVectorSystem(neg_zero, system);
    EXPECT_NEAR(neg_zero_angle.ToRadians(), kPi, kAnglePrecision);

    // Negative ninety axis should convert to -90 angle.
    vec3 neg_ninety = init_vector;
    neg_ninety[ninety_axis] = -1.0f;
    const Angle neg_ninety_angle = Angle::FromVectorSystem(neg_ninety, system);
    EXPECT_NEAR(neg_ninety_angle.ToRadians(), -kHalfPi, kAnglePrecision);

    // Half way between zero and ninety axes should return 45 degree angle.
    vec3 forty_five = init_vector;
    const float one_over_root_two = 1.0f / sqrt(2.0f);
    forty_five[zero_axis] = one_over_root_two;
    forty_five[ninety_axis] = one_over_root_two;
    const Angle forty_five_angle = Angle::FromVectorSystem(forty_five, system);
    EXPECT_NEAR(forty_five_angle.ToRadians(), kQuarterPi, kAnglePrecision);
  }
}

// Test conversions from angles to vectors.
TEST_F(AngleTests, FromVectorSystem) {
  TestFromVectorSystem(fpl::kAngleToVectorXY, 0, 1, 2);
  TestFromVectorSystem(fpl::kAngleToVectorXZ, 0, 2, 1);
  TestFromVectorSystem(fpl::kAngleToVectorYZ, 1, 2, 0);
  TestFromVectorSystem(fpl::kAngleToVectorYX, 1, 0, 2);
  TestFromVectorSystem(fpl::kAngleToVectorZX, 2, 0, 1);
  TestFromVectorSystem(fpl::kAngleToVectorZY, 2, 1, 0);
}

void TestToRotationMatrix(const AngleToVectorSystem system, const int zero_axis,
                          const int ninety_axis, const int ignored_axis) {
  vec3 v;
  v[zero_axis] = 1.0f;
  v[ninety_axis] = 0.0f;
  v[ignored_axis] = 3.0f;

  // Angle zero should translate to the zero axis.
  const mat3 zero_mat = Angle(0.0f).ToRotationMatrix(system);
  const vec3 zero = zero_mat * v;
  EXPECT_NEAR(zero[zero_axis], 1.0f, kUnitVectorPrecision);
  EXPECT_NEAR(zero[ninety_axis], 0.0f, kUnitVectorPrecision);
  EXPECT_NEAR(zero[ignored_axis], 3.0f, kUnitVectorPrecision);

  // Angle 180 should translate to the negative zero axis.
  const mat3 minus_zero_mat = Angle(kPi).ToRotationMatrix(system);
  const vec3 minus_zero = minus_zero_mat * v;
  EXPECT_NEAR(minus_zero[zero_axis], -1.0f, kUnitVectorPrecision);
  EXPECT_NEAR(minus_zero[ninety_axis], 0.0f, kUnitVectorPrecision);
  EXPECT_NEAR(minus_zero[ignored_axis], 3.0f, kUnitVectorPrecision);

  // Angle 90 should translate to the 90 axis.
  const mat3 ninety_mat = Angle(kHalfPi).ToRotationMatrix(system);
  const vec3 ninety = ninety_mat * v;
  EXPECT_NEAR(ninety[zero_axis], 0.0f, kUnitVectorPrecision);
  EXPECT_NEAR(ninety[ninety_axis], 1.0f, kUnitVectorPrecision);
  EXPECT_NEAR(ninety[ignored_axis], 3.0f, kUnitVectorPrecision);

  // Angle -90 should translate to the -90 axis.
  const mat3 minus_ninety_mat = Angle(-kHalfPi).ToRotationMatrix(system);
  const vec3 minus_ninety = minus_ninety_mat * v;
  EXPECT_NEAR(minus_ninety[zero_axis], 0.0f, kUnitVectorPrecision);
  EXPECT_NEAR(minus_ninety[ninety_axis], -1.0f, kUnitVectorPrecision);
  EXPECT_NEAR(minus_ninety[ignored_axis], 3.0f, kUnitVectorPrecision);

  // Angle 45 should translate to a unit vector between the 0 and 90 axes.
  const mat3 forty_five_mat = Angle(kQuarterPi).ToRotationMatrix(system);
  const vec3 forty_five = forty_five_mat * v;
  const float one_over_root_two = 1.0f / sqrt(2.0f);
  EXPECT_NEAR(forty_five[zero_axis], one_over_root_two, kUnitVectorPrecision);
  EXPECT_NEAR(forty_five[ninety_axis], one_over_root_two, kUnitVectorPrecision);
  EXPECT_NEAR(forty_five[ignored_axis], 3.0f, kUnitVectorPrecision);
}

// Test conversions from angles to vectors.
TEST_F(AngleTests, ToRotationMatrix) {
  TestToRotationMatrix(fpl::kAngleToVectorXY, 0, 1, 2);
  TestToRotationMatrix(fpl::kAngleToVectorXZ, 0, 2, 1);
  TestToRotationMatrix(fpl::kAngleToVectorYZ, 1, 2, 0);
  TestToRotationMatrix(fpl::kAngleToVectorYX, 1, 0, 2);
  TestToRotationMatrix(fpl::kAngleToVectorZX, 2, 0, 1);
  TestToRotationMatrix(fpl::kAngleToVectorZY, 2, 1, 0);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
