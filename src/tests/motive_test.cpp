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
#include "flatbuffers/flatbuffers.h"
#include "motive/engine.h"
#include "motive/init.h"
#include "mathfu/constants.h"
#include "motive/math/angle.h"
#include "motive/common.h"

#define DEBUG_PRINT_MATRICES 0

using fpl::kPi;
using fpl::kHalfPi;
using fpl::Range;
using motive::MotiveEngine;
using motive::Motivator1f;
using motive::MotivatorMatrix4f;
using motive::MotiveTime;
using motive::MotivatorInit;
using motive::MotiveTarget1f;
using motive::OvershootInit;
using motive::SmoothInit;
using motive::MatrixInit;
using motive::MatrixOperationInit;
using motive::Settled1f;
using motive::MatrixOperationType;
using motive::kRotateAboutX;
using motive::kRotateAboutY;
using motive::kRotateAboutZ;
using motive::kTranslateX;
using motive::kTranslateY;
using motive::kTranslateZ;
using motive::kScaleX;
using motive::kScaleY;
using motive::kScaleZ;
using motive::kScaleUniformly;
using mathfu::mat4;
using mathfu::vec3;

typedef mathfu::Matrix<float, 3> mat3;

static const MotiveTime kTimePerFrame = 10;
static const MotiveTime kMaxTime = 10000;
static const float kMatrixEpsilon = 0.00001f;
static const float kAngleEpsilon = 0.01f;

class MotiveTests : public ::testing::Test {
protected:
  virtual void SetUp()
  {
    const Range angle_range(-3.14159265359f, 3.14159265359f);
    motive::OvershootInit::Register();
    motive::SmoothInit::Register();
    motive::MatrixInit::Register();

    // Create an OvershootInit with reasonable values.
    overshoot_angle_init_.set_modular(true);
    overshoot_angle_init_.set_range(angle_range);
    overshoot_angle_init_.set_max_velocity(0.021f);
    overshoot_angle_init_.set_max_delta(3.141f);
    overshoot_angle_init_.at_target().max_difference = 0.087f;
    overshoot_angle_init_.at_target().max_velocity = 0.00059f;
    overshoot_angle_init_.set_accel_per_difference(0.00032f);
    overshoot_angle_init_.set_wrong_direction_multiplier(4.0f);
    overshoot_angle_init_.set_max_delta_time(10);

    // Create an OvershootInit that represents a percent from 0 ~ 100.
    // It does not wrap around.
    overshoot_percent_init_.set_modular(false);
    overshoot_percent_init_.set_range(Range(0.0f, 100.0f));
    overshoot_percent_init_.set_max_velocity(10.0f);
    overshoot_percent_init_.set_max_delta(50.0f);
    overshoot_percent_init_.at_target().max_difference = 0.087f;
    overshoot_percent_init_.at_target().max_velocity = 0.00059f;
    overshoot_percent_init_.set_accel_per_difference(0.00032f);
    overshoot_percent_init_.set_wrong_direction_multiplier(4.0f);
    overshoot_percent_init_.set_max_delta_time(10);

    smooth_angle_init_.set_modular(true);
    smooth_angle_init_.set_range(angle_range);

    smooth_scalar_init_.set_modular(false);
    smooth_scalar_init_.set_range(Range(-100.0f, 100.0f));
  }
  virtual void TearDown() {}

protected:
 void InitMotivator(const MotivatorInit& init, float start_value,
                    float start_velocity, float target_value,
                    Motivator1f* motivator) {
   motivator->InitializeWithTarget(
       init, &engine_, motive::CurrentToTarget1f(start_value, start_velocity,
                                                 target_value, 0.0f, 1));
  }

  void InitOvershootMotivator(Motivator1f* motivator) {
    InitMotivator(overshoot_percent_init_, overshoot_percent_init_.Max(),
                  overshoot_percent_init_.max_velocity(),
                  overshoot_percent_init_.Max(), motivator);
  }

  void InitOvershootMotivatorArray(Motivator1f* motivators, int len) {
    for (int i = 0; i < len; ++i) {
      InitOvershootMotivator(&motivators[i]);
    }
  }

  MotiveTime TimeToSettle(const Motivator1f& motivator,
                          const Settled1f& settled) {
    MotiveTime time = 0;
    while (time < kMaxTime && !settled.Settled(motivator)) {
      engine_.AdvanceFrame(kTimePerFrame);
      time += kTimePerFrame;
    }
    return time;
  }

  MotiveEngine engine_;
  OvershootInit overshoot_angle_init_;
  OvershootInit overshoot_percent_init_;
  SmoothInit smooth_angle_init_;
  SmoothInit smooth_scalar_init_;
};

// Ensure we wrap around from pi to -pi.
TEST_F(MotiveTests, ModularMovement) {
  Motivator1f motivator;
  InitMotivator(overshoot_angle_init_, kPi, 0.001f, -kPi + 1.0f, &motivator);
  engine_.AdvanceFrame(1);

  // We expect the position to go up from +pi since it has positive velocity.
  // Since +pi is the max of the range, we expect the value to wrap down to -pi.
  EXPECT_LE(motivator.Value(), 0.0f);
}

// Ensure the simulation settles on the target in a reasonable amount of time.
TEST_F(MotiveTests, EventuallySettles) {
  Motivator1f motivator;
  InitMotivator(overshoot_angle_init_, 0.0f,
                overshoot_angle_init_.max_velocity(), -kPi + 1.0f, &motivator);
  const MotiveTime time_to_settle =
      TimeToSettle(motivator, overshoot_angle_init_.at_target());

  // The simulation should complete in about half a second (time is in ms).
  // Checke that it doesn't finish too quickly nor too slowly.
  EXPECT_GT(time_to_settle, 0);
  EXPECT_LT(time_to_settle, 700);
}

// Ensure the simulation settles when the target is the max bound in a modular
// type. It will oscillate between the max and min bound a lot.
TEST_F(MotiveTests, SettlesOnMax) {
  Motivator1f motivator;
  InitMotivator(overshoot_angle_init_, kPi,
                overshoot_angle_init_.max_velocity(), kPi, &motivator);
  const MotiveTime time_to_settle =
      TimeToSettle(motivator, overshoot_angle_init_.at_target());

  // The simulation should complete in about half a second (time is in ms).
  // Checke that it doesn't finish too quickly nor too slowly.
  EXPECT_GT(time_to_settle, 0);
  EXPECT_LT(time_to_settle, 500);
}

// Ensure the simulation does not exceed the max bound, on constrants that
// do not wrap around.
TEST_F(MotiveTests, StaysWithinBound) {
  Motivator1f motivator;
  InitOvershootMotivator(&motivator);
  engine_.AdvanceFrame(1);

  // Even though we're at the bound and trying to travel beyond the bound,
  // the simulation should clamp our position to the bound.
  EXPECT_EQ(motivator.Value(), overshoot_percent_init_.Max());
}

// Open up a hole in the data and then call Defragment() to close it.
TEST_F(MotiveTests, Defragment) {
  Motivator1f motivators[4];
  const int len = static_cast<int>(MOTIVE_ARRAY_SIZE(motivators));
  for (int hole = 0; hole < len; ++hole) {
    InitOvershootMotivatorArray(motivators, len);

    // Invalidate motivator at index 'hole'.
    motivators[hole].Invalidate();
    EXPECT_FALSE(motivators[hole].Valid());

    // Defragment() is called at the start of AdvanceFrame.
    engine_.AdvanceFrame(1);
    EXPECT_FALSE(motivators[hole].Valid());

    // Compare the remaining motivators against each other.
    const int compare = hole == 0 ? 1 : 0;
    EXPECT_TRUE(motivators[compare].Valid());
    for (int i = 0; i < len; ++i) {
      if (i == hole || i == compare)
        continue;

      // All the motivators should be valid and have the same values.
      EXPECT_TRUE(motivators[i].Valid());
      EXPECT_EQ(motivators[i].Value(), motivators[compare].Value());
      EXPECT_EQ(motivators[i].Velocity(), motivators[compare].Velocity());
      EXPECT_EQ(motivators[i].TargetValue(), motivators[compare].TargetValue());
    }
  }
}

// Copy a valid motivator. Ensure original motivator gets invalidated.
TEST_F(MotiveTests, CopyConstructor) {
  Motivator1f orig_motivator;
  InitOvershootMotivator(&orig_motivator);
  EXPECT_TRUE(orig_motivator.Valid());
  const float value = orig_motivator.Value();

  Motivator1f new_motivator(orig_motivator);
  EXPECT_FALSE(orig_motivator.Valid());
  EXPECT_TRUE(new_motivator.Valid());
  EXPECT_EQ(new_motivator.Value(), value);
}

// Copy an invalid motivator.
TEST_F(MotiveTests, CopyConstructorInvalid) {
  Motivator1f invalid_motivator;
  EXPECT_FALSE(invalid_motivator.Valid());

  Motivator1f copy_of_invalid(invalid_motivator);
  EXPECT_FALSE(copy_of_invalid.Valid());
}

TEST_F(MotiveTests, AssignmentOperator) {
  Motivator1f orig_motivator;
  InitOvershootMotivator(&orig_motivator);
  EXPECT_TRUE(orig_motivator.Valid());
  const float value = orig_motivator.Value();

  Motivator1f new_motivator;
  new_motivator = orig_motivator;
  EXPECT_FALSE(orig_motivator.Valid());
  EXPECT_TRUE(new_motivator.Valid());
  EXPECT_EQ(new_motivator.Value(), value);
}

TEST_F(MotiveTests, VectorResize) {
  static const int kStartSize = 4;
  std::vector<Motivator1f> motivators(kStartSize);

  // Create the motivators and ensure that they're valid.
  for (int i = 0; i < kStartSize; ++i) {
    InitOvershootMotivator(&motivators[i]);
    EXPECT_TRUE(motivators[i].Valid());
  }

  // Expand the size of 'motivators'. This should force the array to be
  // reallocated and all motivators in the array to be moved.
  const Motivator1f* orig_address = &motivators[0];
  motivators.resize(kStartSize + 1);
  const Motivator1f* new_address = &motivators[0];
  EXPECT_NE(orig_address, new_address);

  // All the move motivators should still be valid.
  for (int i = 0; i < kStartSize; ++i) {
    InitOvershootMotivator(&motivators[i]);
    EXPECT_TRUE(motivators[i].Valid());
  }
}

TEST_F(MotiveTests, SmoothModular) {
  static const float kMargin = 0.1f;
  static const MotiveTime kTime = 10;
  static const float kStart = kPi - kMargin;
  static const float kEnd = -kPi + kMargin;
  Motivator1f angle(smooth_angle_init_, &engine_,
                    motive::CurrentToTarget1f(kStart, 0.0f, kEnd, 0.0f, kTime));

  // The difference should be the short way around, across kPi.
  EXPECT_NEAR(angle.Value(), kStart, kAngleEpsilon);
  EXPECT_NEAR(angle.Difference(), 2.0f * kMargin, kAngleEpsilon);

  // Ensure that we're always near kPi, never near 0. We want to go the
  // short way around.
  for (MotiveTime t = 0; t < kTime; ++t) {
    EXPECT_TRUE(kStart - kAngleEpsilon <= angle.Value() ||
                angle.Value() <= kEnd + kAngleEpsilon);
    engine_.AdvanceFrame(1);
  }
  EXPECT_NEAR(angle.Value(), kEnd, kAngleEpsilon);
}

// Print matrices with columns vertically.
static void PrintMatrix(const char* name, const mat4& m) {
  (void)name; (void)m;
  #if DEBUG_PRINT_MATRICES
  printf("%s\n(%f %f %f %f)\n(%f %f %f %f)\n(%f %f %f %f)\n(%f %f %f %f)\n",
         name, m[0], m[4], m[8], m[12], m[1], m[5], m[9], m[13],
         m[2], m[6], m[10], m[14], m[3], m[7], m[11], m[15]);
  #endif // DEBUG_PRINT_MATRICES
}

// Create a matrix that performs the transformation specified in 'op_init'.
static mat4 CreateMatrixFromOp(const MatrixOperationInit& op_init) {
  const float v = op_init.initial_value;

  switch (op_init.type) {
    case kRotateAboutX: return mat4::FromRotationMatrix(mat3::RotationX(v));
    case kRotateAboutY: return mat4::FromRotationMatrix(mat3::RotationY(v));
    case kRotateAboutZ: return mat4::FromRotationMatrix(mat3::RotationZ(v));
    case kTranslateX: return mat4::FromTranslationVector(vec3(v, 0.0f, 0.0f));
    case kTranslateY: return mat4::FromTranslationVector(vec3(0.0f, v, 0.0f));
    case kTranslateZ: return mat4::FromTranslationVector(vec3(0.0f, 0.0f, v));
    case kScaleX: return mat4::FromScaleVector(vec3(v, 1.0f, 1.0f));
    case kScaleY: return mat4::FromScaleVector(vec3(1.0f, v, 1.0f));
    case kScaleZ: return mat4::FromScaleVector(vec3(1.0f, 1.0f, v));
    case kScaleUniformly: return mat4::FromScaleVector(vec3(v));
    default:
      assert(false);
      return mat4::Identity();
  }
}

// Return the product of the matrices for each operation in 'matrix_init'.
static mat4 CreateMatrixFromOps(const MatrixInit& matrix_init) {
  const MatrixInit::OpVector& ops = matrix_init.ops();

  mat4 m = mat4::Identity();
  for (size_t i = 0; i < ops.size(); ++i) {
    m *= CreateMatrixFromOp(ops[i]);
  }
  return m;
}

static void ExpectMatricesEqual(const mat4& a, const mat4&b, float epsilon) {
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      EXPECT_NEAR(a(i, j), b(i, j), epsilon);
    }
  }
}

static void TestMatrixMotivator(const MatrixInit& matrix_init,
                                MotiveEngine* engine) {
  MotivatorMatrix4f matrix_motivator(matrix_init, engine);
  engine->AdvanceFrame(kTimePerFrame);
  const mat4 check_matrix = CreateMatrixFromOps(matrix_init);
  const mat4 motive_matrix = matrix_motivator.Value();
  ExpectMatricesEqual(motive_matrix, check_matrix, kMatrixEpsilon);

  // Output matrices for debugging.
  PrintMatrix("motivator", motive_matrix);
  PrintMatrix("check", check_matrix);
}

// Test the matrix operation kTranslateX.
TEST_F(MotiveTests, MatrixTranslateX) {
  MatrixInit matrix_init;
  matrix_init.AddOp(motive::kTranslateX, smooth_scalar_init_, 2.0f);
  TestMatrixMotivator(matrix_init, &engine_);
}

// Don't use an motivator to drive the animation. Use a constant value.
TEST_F(MotiveTests, MatrixTranslateXConstValue) {
  MatrixInit matrix_init;
  matrix_init.AddOp(motive::kTranslateX, 2.0f);
  TestMatrixMotivator(matrix_init, &engine_);
}

// Test the matrix operation kRotateAboutX.
TEST_F(MotiveTests, MatrixRotateAboutX) {
  MatrixInit matrix_init;
  matrix_init.AddOp(motive::kRotateAboutX, smooth_angle_init_, kHalfPi);
  TestMatrixMotivator(matrix_init, &engine_);
}

// Test the matrix operation kRotateAboutY.
TEST_F(MotiveTests, MatrixRotateAboutY) {
  MatrixInit matrix_init;
  matrix_init.AddOp(motive::kRotateAboutY, smooth_angle_init_, kHalfPi / 3.0f);
  TestMatrixMotivator(matrix_init, &engine_);
}

// Test the matrix operation kRotateAboutZ.
TEST_F(MotiveTests, MatrixRotateAboutZ) {
  MatrixInit matrix_init;
  matrix_init.AddOp(motive::kRotateAboutZ, smooth_angle_init_, -kHalfPi / 1.2f);
  TestMatrixMotivator(matrix_init, &engine_);
}

// Test the matrix operation kScaleX.
TEST_F(MotiveTests, MatrixScaleX) {
  MatrixInit matrix_init;
  matrix_init.AddOp(motive::kScaleX, smooth_scalar_init_, -3.0f);
  TestMatrixMotivator(matrix_init, &engine_);
}

// Test the series of matrix operations for translating XYZ.
TEST_F(MotiveTests, MatrixTranslateXYZ) {
  MatrixInit matrix_init;
  matrix_init.AddOp(motive::kTranslateX, smooth_scalar_init_, 2.0f);
  matrix_init.AddOp(motive::kTranslateY, smooth_scalar_init_, -3.0f);
  matrix_init.AddOp(motive::kTranslateZ, smooth_scalar_init_, 0.5f);
  TestMatrixMotivator(matrix_init, &engine_);
}

// Test the series of matrix operations for rotating about X, Y, and Z,
// in turn.
TEST_F(MotiveTests, MatrixRotateAboutXYZ) {
  MatrixInit matrix_init;
  matrix_init.AddOp(motive::kRotateAboutX, smooth_angle_init_, -kHalfPi / 2.0f);
  matrix_init.AddOp(motive::kRotateAboutY, smooth_angle_init_, kHalfPi / 3.0f);
  matrix_init.AddOp(motive::kRotateAboutZ, smooth_angle_init_, kHalfPi / 5.0f);
  TestMatrixMotivator(matrix_init, &engine_);
}

// Test the series of matrix operations for scaling XYZ non-uniformly.
TEST_F(MotiveTests, MatrixScaleXYZ) {
  MatrixInit matrix_init;
  matrix_init.AddOp(motive::kScaleX, smooth_scalar_init_, -3.0f);
  matrix_init.AddOp(motive::kScaleY, smooth_scalar_init_, 2.2f);
  matrix_init.AddOp(motive::kScaleZ, smooth_scalar_init_, 1.01f);
  TestMatrixMotivator(matrix_init, &engine_);
}

// Test the matrix operation kScaleUniformly.
TEST_F(MotiveTests, MatrixScaleUniformly) {
  MatrixInit matrix_init;
  matrix_init.AddOp(motive::kScaleUniformly, smooth_scalar_init_, 10.1f);
  TestMatrixMotivator(matrix_init, &engine_);
}

// Test the series of matrix operations for translating and rotating.
TEST_F(MotiveTests, MatrixTranslateRotateTranslateBack) {
  MatrixInit matrix_init;
  matrix_init.AddOp(motive::kTranslateY, smooth_scalar_init_, 1.0f);
  matrix_init.AddOp(motive::kRotateAboutX, smooth_angle_init_, kHalfPi);
  matrix_init.AddOp(motive::kTranslateY, smooth_scalar_init_, -1.0f);
  TestMatrixMotivator(matrix_init, &engine_);
}

// Test the series of matrix operations for translating, rotating, and scaling.
TEST_F(MotiveTests, MatrixTranslateRotateScale) {
  MatrixInit matrix_init;
  matrix_init.AddOp(motive::kTranslateY, smooth_scalar_init_, 1.0f);
  matrix_init.AddOp(motive::kRotateAboutX, smooth_angle_init_, kHalfPi);
  matrix_init.AddOp(motive::kScaleZ, smooth_scalar_init_, -1.4f);
  TestMatrixMotivator(matrix_init, &engine_);
}

// Test a complex the series of matrix operations.
TEST_F(MotiveTests, MatrixTranslateRotateScaleGoneWild) {
  MatrixInit matrix_init;
  matrix_init.AddOp(motive::kTranslateY, smooth_scalar_init_, 1.0f);
  matrix_init.AddOp(motive::kTranslateX, smooth_scalar_init_, -1.6f);
  matrix_init.AddOp(motive::kRotateAboutX, smooth_angle_init_, kHalfPi * 0.1f);
  matrix_init.AddOp(motive::kRotateAboutY, smooth_angle_init_, kHalfPi * 0.33f);
  matrix_init.AddOp(motive::kScaleZ, smooth_scalar_init_, -1.4f);
  matrix_init.AddOp(motive::kRotateAboutY, smooth_angle_init_,
                    -kHalfPi * 0.33f);
  matrix_init.AddOp(motive::kTranslateX, smooth_scalar_init_, -1.2f);
  matrix_init.AddOp(motive::kTranslateY, smooth_scalar_init_, -1.5f);
  matrix_init.AddOp(motive::kTranslateZ, smooth_scalar_init_, -2.2f);
  matrix_init.AddOp(motive::kRotateAboutZ, smooth_angle_init_, -kHalfPi * 0.5f);
  matrix_init.AddOp(motive::kScaleX, smooth_scalar_init_, 2.0f);
  matrix_init.AddOp(motive::kScaleY, smooth_scalar_init_, 4.1f);
  TestMatrixMotivator(matrix_init, &engine_);
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
