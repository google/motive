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
#include "motive/math/angle.h"

using motive::Range;
using motive::kPi;

static const float kInf = std::numeric_limits<float>::infinity();
static const float kAngleEpsilon = 0.0001f;
static const float kZeroOneEpsilon = 0.00003f;

class RangeTests : public ::testing::Test {
protected:
  virtual void SetUp() {}
  virtual void TearDown() {}
};


TEST_F(RangeTests, Valid) {
  // When start <= end, the range is considered valid.
  const Range valid(0.0f, 1.0f);
  EXPECT_TRUE(valid.Valid());

  // When end > start, the range is considered invalid.
  const Range invalid(1.0f, -1.0f);
  EXPECT_TRUE(valid.Valid());

  // By default, the range should be initialized to something invalid.
  const Range invalid_default;
  EXPECT_FALSE(invalid_default.Valid());
}

// Infinities should be able to be used in ranges.
TEST_F(RangeTests, Valid_Infinity) {
  const Range full(-kInf, kInf);
  EXPECT_TRUE(full.Valid());

  const Range neg_half(-kInf, 0.0f);
  EXPECT_TRUE(neg_half.Valid());

  const Range pos_half(-10.0, kInf);
  EXPECT_TRUE(pos_half.Valid());
}

// Ensure the middle of the range is the algebraic middle.
TEST_F(RangeTests, Middle) {
  // Test positive range.
  const Range zero_one(0.0f, 1.0f);
  EXPECT_EQ(0.5f, zero_one.Middle());

  // Test range that spans zero.
  const Range minus_one_one(-1.0f, 1.0f);
  EXPECT_EQ(0.0f, minus_one_one.Middle());
}

// Ensure the length is the width of the interval.
TEST_F(RangeTests, Length) {
  // Test positive range.
  const Range zero_one(0.0f, 1.0f);
  EXPECT_EQ(1.0f, zero_one.Length());

  // Test range that spans zero.
  const Range minus_one_one(-1.0f, 1.0f);
  EXPECT_EQ(0.0f, minus_one_one.Middle());

  // Test range with infinity.
  const Range one_inf(1.0f, kInf);
  EXPECT_EQ(kInf, one_inf.Length());
}

// Clamping values inside the range should result in the same value.
TEST_F(RangeTests, Clamp_Inside) {
  const Range zero_one(0.0f, 1.0f);
  EXPECT_EQ(0.5f, zero_one.Clamp(0.5f));
  EXPECT_EQ(0.9999999f, zero_one.Clamp(0.9999999f));
}

// Clamping values inside the range should result in the same value.
TEST_F(RangeTests, Clamp_Border) {
  const Range zero_one(0.0f, 1.0f);
  EXPECT_EQ(0.0f, zero_one.Clamp(0.0f));
  EXPECT_EQ(1.0f, zero_one.Clamp(1.0f));
}

// Clamping values inside the range should result in the same value.
TEST_F(RangeTests, Clamp_Outside) {
  const Range zero_one(0.0f, 1.0f);
  EXPECT_EQ(0.0f, zero_one.Clamp(-1.0f));
  EXPECT_EQ(1.0f, zero_one.Clamp(1.0000001f));
}

// Passing infinity into a range should clamp fine.
TEST_F(RangeTests, Clamp_Infinity) {
  const Range zero_one(0.0f, 1.0f);
  EXPECT_EQ(1.0f, zero_one.Clamp(kInf));
  EXPECT_EQ(0.0f, zero_one.Clamp(-kInf));
}

// Clamping values to the full range should always return the original value.
TEST_F(RangeTests, Clamp_ToInfinity) {
  const Range full(-kInf, kInf);
  EXPECT_EQ(kInf, full.Clamp(kInf));
  EXPECT_EQ(1.0f, full.Clamp(1.0f));
  EXPECT_EQ(-kInf, full.Clamp(-kInf));
}

// Clamping values above the start threshold.
TEST_F(RangeTests, Clamp_AfterStart) {
  const Range r(-1.0f, 1.0f);
  EXPECT_EQ(-1.0f, r.ClampAfterStart(-2.0f));
  EXPECT_EQ(-1.0f, r.ClampAfterStart(-1.0f));
  EXPECT_EQ(-0.9f, r.ClampAfterStart(-0.9f));
  EXPECT_EQ(2.0f, r.ClampAfterStart(2.0f));
}

// Clamping values below the end threshold.
TEST_F(RangeTests, Clamp_BeforeEnd) {
  const Range r(-1.0f, 1.0f);
  EXPECT_EQ(1.0f, r.ClampBeforeEnd(2.0f));
  EXPECT_EQ(1.0f, r.ClampBeforeEnd(1.0f));
  EXPECT_EQ(0.9f, r.ClampBeforeEnd(0.9f));
  EXPECT_EQ(-2.0f, r.ClampBeforeEnd(-2.0f));
}

// Distance from the range should be zero for elements inside the range.
TEST_F(RangeTests, DistanceFrom_Inside) {
  const Range zero_one(0.0f, 1.0f);
  EXPECT_EQ(0.0f, zero_one.DistanceFrom(0.0000001f));
  EXPECT_EQ(0.0f, zero_one.DistanceFrom(0.5f));
  EXPECT_EQ(0.0f, zero_one.DistanceFrom(0.9f));
}

// Distance from the range should be zero for elements on the border.
TEST_F(RangeTests, DistanceFrom_Border) {
  const Range zero_one(0.0f, 1.0f);
  EXPECT_EQ(0.0f, zero_one.DistanceFrom(0.0f));
  EXPECT_EQ(0.0f, zero_one.DistanceFrom(1.0f));
}

// Distance from the range should be match for elements outside the range.
TEST_F(RangeTests, DistanceFrom_Outside) {
  const Range zero_one(0.0f, 1.0f);
  EXPECT_EQ(1.0f, zero_one.DistanceFrom(-1.0f));
  EXPECT_NEAR(0.2f, zero_one.DistanceFrom(1.2f), 0.000001f);
}

// Distance from the range should always be infinity for infinite values.
TEST_F(RangeTests, DistanceFrom_Infinity) {
  const Range zero_one(0.0f, 1.0f);
  EXPECT_EQ(kInf, zero_one.DistanceFrom(kInf));
  EXPECT_EQ(kInf, zero_one.DistanceFrom(-kInf));
}

// Distance from the full range should always be zero.
TEST_F(RangeTests, DistanceFrom_InfiniteRange) {
  const Range full(-kInf, kInf);
  EXPECT_EQ(0.0f, full.DistanceFrom(0.0f));
  EXPECT_EQ(0.0f, full.DistanceFrom(1.0f));
// Note: Doesn't work when passing in kInf because kInf - kInf = NaN.
//  EXPECT_EQ(0.0f, full.DistanceFrom(kInf));
//  EXPECT_EQ(0.0f, full.DistanceFrom(-kInf));
}

// 1.  |-a---|    |-b---|  ==>  return invalid
TEST_F(RangeTests, Intersect_DisjointBelow) {
  const Range a(0.0f, 1.0f);
  const Range b(2.0f, 3.0f);
  const Range intersection = Range::Intersect(a, b);
  EXPECT_FALSE(intersection.Valid());
  EXPECT_TRUE(intersection.Invert().Valid());
  EXPECT_EQ(intersection.Invert().Length(), 1.0f);
}

// 2.  |-b---|    |-a---|  ==>  return invalid
TEST_F(RangeTests, Intersect_DisjointAbove) {
  const Range a(2.0f, 3.0f);
  const Range b(0.0f, 1.0f);
  const Range intersection = Range::Intersect(a, b);
  EXPECT_FALSE(intersection.Valid());
  EXPECT_TRUE(intersection.Invert().Valid());
  EXPECT_EQ(intersection.Invert().Length(), 1.0f);
}

// 3.  |-a---------|       ==>  return b
//        |-b---|
TEST_F(RangeTests, Intersect_ContainsSecond) {
  const Range a(-10.0f, 10.0f);
  const Range b(2.0f, 3.0f);
  const Range intersection = Range::Intersect(a, b);
  EXPECT_TRUE(intersection.Valid());
  EXPECT_EQ(intersection, b);
}

// 4.  |-b---------|       ==>  return a
//        |-a---|
TEST_F(RangeTests, Intersect_ContainsFirst) {
  const Range a(2.0f, 3.0f);
  const Range b(-10.0f, 10.0f);
  const Range intersection = Range::Intersect(a, b);
  EXPECT_TRUE(intersection.Valid());
  EXPECT_EQ(intersection, a);
}


// 5.  |-a---|             ==>  return (b.start, a.end)
//        |-b---|
TEST_F(RangeTests, Intersect_OverlapFirst) {
  const Range a(0.0f, 2.0f);
  const Range b(1.0f, 3.0f);
  const Range intersection = Range::Intersect(a, b);
  EXPECT_EQ(intersection, Range(1.0f, 2.0f));
}

// 6.  |-b---|             ==>  return (a.start, b.end)
//        |-a---|
TEST_F(RangeTests, Intersect_OverlapSecond) {
  const Range a(1.0f, 3.0f);
  const Range b(0.0f, 2.0f);
  const Range intersection = Range::Intersect(a, b);
  EXPECT_EQ(intersection, Range(1.0f, 2.0f));
}

TEST_F(RangeTests, Normalize_Inside) {
  const Range a(-kPi, kPi);
  EXPECT_EQ(0.0f, a.Normalize(0.0f));
  EXPECT_EQ(1.0f, a.Normalize(1.0f));
  EXPECT_EQ(-1.0f, a.Normalize(-1.0f));
  EXPECT_EQ(2.1f, a.Normalize(2.1f));
}

TEST_F(RangeTests, Normalize_LowerBoundary) {
  const Range a(-kPi, kPi);
  const Range zero_one(0.0f, 1.0f);
  EXPECT_EQ(kPi, a.Normalize(-kPi));
  EXPECT_EQ(1.0f, zero_one.Normalize(0.0f));
}

TEST_F(RangeTests, Normalize_UpperBoundary) {
  const Range a(-kPi, kPi);
  const Range zero_one(0.0f, 1.0f);
  EXPECT_EQ(kPi, a.Normalize(kPi));
  EXPECT_EQ(1.0f, zero_one.Normalize(1.0f));
}

TEST_F(RangeTests, Normalize_Below) {
  const Range a(-kPi, kPi);
  const Range zero_one(0.0f, 1.0f);
  EXPECT_EQ(0.0f, a.Normalize(-2.0f * kPi));
  EXPECT_NEAR(kPi - 1.0f, a.Normalize(-kPi - 1.0f), kAngleEpsilon);
  EXPECT_NEAR(0.1f, zero_one.Normalize(-0.9f), kZeroOneEpsilon);
  EXPECT_NEAR(0.5f, zero_one.Normalize(-0.5f), kZeroOneEpsilon);
}

TEST_F(RangeTests, Normalize_Above) {
  const Range a(-kPi, kPi);
  const Range zero_one(0.0f, 1.0f);
  EXPECT_EQ(0.0f, a.Normalize(2.0f * kPi));
  EXPECT_NEAR(-kPi + 1.0f, a.Normalize(kPi + 1.0f), kAngleEpsilon);
  EXPECT_NEAR(0.9f, zero_one.Normalize(1.9f), kZeroOneEpsilon);
  EXPECT_NEAR(0.5f, zero_one.Normalize(1.5f), kZeroOneEpsilon);
}

typedef float NormalizeFn(const Range& r, float x);

static float NormalizeWild(const Range& r, float x) {
  return r.NormalizeWildValue(x);
}

static float NormalizeClose(const Range& r, float x) {
  return r.NormalizeCloseValue(x);
}

void TestNormalize_Inside(NormalizeFn* fn) {
  const Range a(-kPi, kPi);
  const Range zero_one(0.0f, 1.0f);
  EXPECT_EQ(0.0f, fn(a, 0.0f));
  EXPECT_NEAR(kPi - 0.1f, fn(a, kPi - 0.1f), kAngleEpsilon);
  EXPECT_EQ(1.0f, fn(zero_one, 1.0f));
  EXPECT_EQ(0.5f, fn(zero_one, 0.5f));
}

void TestNormalize_Border(NormalizeFn* fn) {
  const Range a(-kPi, kPi);
  const Range zero_one(0.0f, 1.0f);
  EXPECT_EQ(kPi, fn(a, -kPi));
  EXPECT_EQ(kPi, fn(a, kPi));
  EXPECT_EQ(1.0f, fn(zero_one, 0.0f));
  EXPECT_EQ(1.0f, fn(zero_one, 1.0f));
}

void TestNormalize_JustOutside(NormalizeFn* fn) {
  const Range a(-kPi, kPi);
  const Range zero_one(0.0f, 1.0f);
  EXPECT_NEAR(0.9999f * kPi, fn(a, -1.0001f * kPi), kAngleEpsilon);
  EXPECT_NEAR(-kPi + 0.00001f, fn(a, kPi + 0.00001f), kAngleEpsilon);
  EXPECT_NEAR(0.99f, fn(zero_one, -1.01f), kZeroOneEpsilon);
  EXPECT_NEAR(0.0000004f, fn(zero_one, 1.0000004f), kZeroOneEpsilon);
}

void TestNormalize_FartherOutside(NormalizeFn* fn) {
  const Range a(-kPi, kPi);
  const Range zero_one(0.0f, 1.0f);
  EXPECT_NEAR(-0.9f * kPi, fn(a, -4.9f * kPi), kAngleEpsilon);
  EXPECT_NEAR(1.0f, fn(a, 4.0f * kPi + 1.0f), kAngleEpsilon);
  EXPECT_NEAR(0.7f, fn(zero_one, -2.3f), kZeroOneEpsilon);
  EXPECT_NEAR(0.5f, fn(zero_one, 3.5f), kZeroOneEpsilon);
}

void TestNormalize_Distant(NormalizeFn* fn) {
  const Range a(-kPi, kPi);
  const Range zero_one(0.0f, 1.0f);
  EXPECT_NEAR(0.0f, fn(a, -10.0f * kPi), kAngleEpsilon);
  EXPECT_NEAR(1.0f, fn(a, 100.0f * kPi + 1.0f), kAngleEpsilon);
  EXPECT_NEAR(0.7f, fn(zero_one, -19.3f), kZeroOneEpsilon);
  EXPECT_NEAR(0.5f, fn(zero_one, 10.5f), kZeroOneEpsilon);
}

TEST_F(RangeTests, NormalizeWild_Inside) {
  TestNormalize_Inside(NormalizeWild);
}
TEST_F(RangeTests, NormalizeWild_Border) {
  TestNormalize_Border(NormalizeWild);
}
TEST_F(RangeTests, NormalizeWild_JustOutside) {
  TestNormalize_JustOutside(NormalizeWild);
}
TEST_F(RangeTests, NormalizeWild_FartherOutside) {
  TestNormalize_FartherOutside(NormalizeWild);
}
TEST_F(RangeTests, NormalizeWild_Distant) {
  TestNormalize_Distant(NormalizeWild);
}

TEST_F(RangeTests, NormalizeClose_Inside) {
  TestNormalize_Inside(NormalizeClose);
}
TEST_F(RangeTests, NormalizeClose_Border) {
  TestNormalize_Border(NormalizeClose);
}
TEST_F(RangeTests, NormalizeClose_JustOutside) {
  TestNormalize_JustOutside(NormalizeClose);
}
TEST_F(RangeTests, NormalizeClose_FartherOutside) {
  TestNormalize_FartherOutside(NormalizeClose);
}
TEST_F(RangeTests, NormalizeClose_Distant) {
  TestNormalize_Distant(NormalizeClose);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

