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

#include "motive/math/float.h"
#include "gtest/gtest.h"

class FloatingPointTests : public ::testing::Test {
 protected:
  virtual void SetUp() {}
  virtual void TearDown() {}
};

static const float kMinFloat = std::numeric_limits<float>::min();
static const float kMaxFloat = std::numeric_limits<float>::max();
static const float kInfinity = std::numeric_limits<float>::infinity();

TEST_F(FloatingPointTests, ExponentAsIntSpecial) {
  EXPECT_EQ(motive::kInfinityExponent, motive::ExponentAsInt(kInfinity));
  EXPECT_EQ(motive::kInfinityExponent, motive::ExponentAsInt(-kInfinity));
  EXPECT_EQ(motive::kZeroExponent, motive::ExponentAsInt(0.0f));
  EXPECT_EQ(motive::kZeroExponent, motive::ExponentAsInt(-0.0f));
  EXPECT_EQ(motive::kMaxFloatExponent, motive::ExponentAsInt(kMaxFloat));
  EXPECT_EQ(motive::kMaxFloatExponent, motive::ExponentAsInt(-kMaxFloat));
  EXPECT_EQ(motive::kMinFloatExponent, motive::ExponentAsInt(kMinFloat));
  EXPECT_EQ(motive::kMinFloatExponent, motive::ExponentAsInt(-kMinFloat));
}

TEST_F(FloatingPointTests, ExponentAsIntExact) {
  int exponent = motive::kMinFloatExponent;
  for (float f = kMinFloat; f <= kMaxFloat; f *= 2.0f) {
    EXPECT_EQ(exponent, motive::ExponentAsInt(f));
    exponent++;
  }
}

TEST_F(FloatingPointTests, ExponentAsIntOffset) {
  int exponent = motive::kMinFloatExponent;
  for (float f = kMinFloat * 1.1f; f <= kMaxFloat; f *= 2.0f) {
    EXPECT_EQ(exponent, motive::ExponentAsInt(f));
    exponent++;
  }
}

TEST_F(FloatingPointTests, ExponentAsIntNegative) {
  int i = motive::kMinFloatExponent;
  for (float f = -kMinFloat; f >= -kMaxFloat; f *= 2.0f) {
    EXPECT_EQ(i, motive::ExponentAsInt(f));
    i++;
  }
}

TEST_F(FloatingPointTests, ExponentFromIntSpecial) {
  EXPECT_EQ(kInfinity, motive::ExponentFromInt(motive::kInfinityExponent));
  EXPECT_EQ(0.0f, motive::ExponentFromInt(motive::kZeroExponent));
  EXPECT_EQ(std::pow(2.0f, 127.0f),
            motive::ExponentFromInt(motive::kMaxFloatExponent));
  EXPECT_EQ(std::pow(2.0f, -126.0f),
            motive::ExponentFromInt(motive::kMinFloatExponent));
}

TEST_F(FloatingPointTests, ExponentFromInt) {
  float f = kMinFloat;
  for (int i = motive::kMinInvertableExponent;
       i <= motive::kMaxInvertableExponent; ++i) {
    EXPECT_EQ(f, motive::ExponentFromInt(i));
    f *= 2.0f;
  }
}

TEST_F(FloatingPointTests, ExponentBackAndForthToInt) {
  for (int i = motive::kZeroExponent; i <= motive::kInfinityExponent; ++i) {
    EXPECT_EQ(i, motive::ExponentAsInt(motive::ExponentFromInt(i)));
  }
}

TEST_F(FloatingPointTests, ReciprocalExponentExtremes) {
  EXPECT_EQ(motive::kMinInvertablePowerOf2,
            motive::ReciprocalExponent(motive::kMaxInvertablePowerOf2));
  EXPECT_EQ(motive::kMaxInvertablePowerOf2,
            motive::ReciprocalExponent(motive::kMinInvertablePowerOf2));
}

TEST_F(FloatingPointTests, ReciprocalExponentExact) {
  for (float f = motive::kMinInvertablePowerOf2;
       f <= motive::kMaxInvertablePowerOf2; f *= 2.0f) {
    EXPECT_EQ(1.0f / f, motive::ReciprocalExponent(f));
  }
}

TEST_F(FloatingPointTests, ReciprocalExponentOffset) {
  for (float f = motive::kMinInvertablePowerOf2 * 1.3f;
       f <= motive::kMaxInvertablePowerOf2; f *= 2.0f) {
    EXPECT_EQ(1.0f / motive::ExponentFromInt(motive::ExponentAsInt(f)),
              motive::ReciprocalExponent(f));
  }
}

TEST_F(FloatingPointTests, SqrtReciprocalExponentExact) {
  for (float f = motive::kMinInvertablePowerOf2;
       f <= motive::kMaxInvertablePowerOf2; f *= 2.0f) {
    EXPECT_EQ(1.0f / motive::ExponentFromInt(motive::ExponentAsInt(f) / 2),
              motive::SqrtReciprocalExponent(f));
  }
}

TEST_F(FloatingPointTests, SqrtReciprocalExponentOffset) {
  for (float f = motive::kMinInvertablePowerOf2 * 1.7f;
       f <= motive::kMaxInvertablePowerOf2; f *= 2.0f) {
    EXPECT_EQ(1.0f / motive::ExponentFromInt(motive::ExponentAsInt(f) / 2),
              motive::SqrtReciprocalExponent(f));
  }
}

TEST_F(FloatingPointTests, MaxPowerOf2ScaleExact) {
  EXPECT_EQ(4.0f, motive::MaxPowerOf2Scale(1.0f, 2));
  EXPECT_EQ(2.0f, motive::MaxPowerOf2Scale(2.0f, 2));
  EXPECT_EQ(1.0f, motive::MaxPowerOf2Scale(4.0f, 2));
}

TEST_F(FloatingPointTests, MaxPowerOf2ScaleOffset) {
  EXPECT_EQ(4.0f, motive::MaxPowerOf2Scale(1.1f, 2));
  EXPECT_EQ(2.0f, motive::MaxPowerOf2Scale(2.4f, 2));
  EXPECT_EQ(1.0f, motive::MaxPowerOf2Scale(4.9f, 2));
}

TEST_F(FloatingPointTests, MaxPowerOf2ScaleLessThan1) {
  for (float f = motive::kMinInvertablePowerOf2; f <= 1.0f; f *= 2.0f) {
    EXPECT_EQ(motive::ExponentFromInt(motive::kMaxFloatExponent),
              motive::MaxPowerOf2Scale(f, motive::kMaxFloatExponent));
  }
}

TEST_F(FloatingPointTests, MaxPowerOf2ScaleMoreThan1) {
  float max = motive::ExponentFromInt(motive::kMaxFloatExponent);
  for (float f = 1.0f; f <= motive::kMaxInvertablePowerOf2; f *= 2.0f) {
    EXPECT_EQ(max, motive::MaxPowerOf2Scale(f, motive::kMaxFloatExponent));
    max /= 2.0f;
  }
}

TEST_F(FloatingPointTests, ClampNearZero) {
  EXPECT_EQ(0.0f, motive::ClampNearZero(0.0f, 0.0f));
  EXPECT_EQ(kInfinity, motive::ClampNearZero(kInfinity, 0.0f));
  EXPECT_EQ(0.0f, motive::ClampNearZero(1.0f, 1.0f));
  EXPECT_EQ(2.0f, motive::ClampNearZero(2.0f, 1.0f));
  EXPECT_EQ(0.0f, motive::ClampNearZero(0.00001f, 0.0001f));
  EXPECT_EQ(0.00001f, motive::ClampNearZero(0.00001f, 0.000001f));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
