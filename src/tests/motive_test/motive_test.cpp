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
#include "flatbuffers/flatbuffers.h"
#include "motive/engine.h"
#include "motive/init.h"
#include "mathfu/constants.h"
#include "motive/math/angle.h"
#include "motive/common.h"

#define DEBUG_PRINT_MATRICES 0

using motive::Angle;
using motive::kPi;
using motive::kHalfPi;
using motive::Range;
using motive::CompactSpline;
using motive::SplinePlayback;
using motive::MatrixMotivator4f;
using motive::MotiveDimension;
using motive::MotiveEngine;
using motive::Motivator1f;
using motive::Motivator2f;
using motive::Motivator3f;
using motive::Motivator4f;
using motive::MotiveTime;
using motive::MotivatorInit;
using motive::MotiveTarget1f;
using motive::MotiveTarget2f;
using motive::MotiveTarget3f;
using motive::MotiveTarget4f;
using motive::OvershootInit;
using motive::SmoothInit;
using motive::MatrixInit;
using motive::MatrixOperationInit;
using motive::Settled1f;
using motive::MatrixOperationType;
using motive::MatrixOpArray;
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
using mathfu::vec2;
using mathfu::vec3;
using mathfu::vec4;

typedef mathfu::Matrix<float, 3> mat3;

static const MotiveTime kTimePerFrame = 10;
static const MotiveTime kMaxTime = 10000;
static const float kMatrixEpsilon = 0.001f;
static const float kAngleEpsilon = 0.01f;

#define TEST_ALL_VECTOR_MOTIVATORS_F(MOTIVE_TEST_NAME) \
  TEST_F(MotiveTests, MOTIVE_TEST_NAME##Test) {        \
    MOTIVE_TEST_NAME<Motivator3f>(*this);              \
    MOTIVE_TEST_NAME<Motivator4f>(*this);              \
  }

// For tests. Exact comparison.
template <class T, int d>
bool VectorEqual(const mathfu::Vector<T, d>& lhs,
                 const mathfu::Vector<T, d>& rhs) {
  for (int i = 0; i < d; ++i) {
    if (lhs[i] != rhs[i]) return false;
  }
  return true;
}
bool VectorEqual(float lhs, float rhs) { return lhs == rhs; }

// For tests. All elements of vector are exactly the same.
template <class T, int d>
bool VectorUniform(const mathfu::Vector<T, d>& v) {
  for (int i = 1; i < d; ++i) {
    if (v[i] != v[0]) return false;
  }
  return true;
}
bool VectorUniform(float /*v*/) { return true; }

// For tests. All elements of vector are normalized
template <class T, int d>
mathfu::Vector<T, d> VectorNormalize(const mathfu::Vector<T, d>& v) {
  mathfu::Vector<T, d> out;
  for (int i = 0; i < d; ++i) {
    out[i] = Angle::WrapAngle(v[i]);
  }
  return out;
}
float VectorNormalize(float v) { return Angle::WrapAngle(v); }

// For tests. All elements of vector are <=.
template <class T, int d>
bool operator<=(const mathfu::Vector<T, d>& lhs,
                const mathfu::Vector<T, d>& rhs) {
  for (int i = 0; i < d; ++i) {
    if (lhs[i] > rhs[i]) return false;
  }
  return true;
}

// For tests. All elements of vector are >=.
template <class T, int d>
bool operator>=(const mathfu::Vector<T, d>& lhs,
                const mathfu::Vector<T, d>& rhs) {
  for (int i = 0; i < d; ++i) {
    if (lhs[i] < rhs[i]) return false;
  }
  return true;
}

// For tests. All elements of vector are within `precision` of each other.
template <class T>
bool VectorNear(const T& lhs, const T& rhs, const T& precision) {
  const T diff = lhs - rhs;
  return -precision <= diff && diff <= precision;
}

class MotiveTests : public ::testing::Test {
 public:
  MotiveEngine& engine() { return engine_; }
  const OvershootInit& overshoot_angle_init() const {
    return overshoot_angle_init_;
  }
  const OvershootInit& overshoot_percent_init() const {
    return overshoot_percent_init_;
  }
  const SmoothInit& smooth_angle_init() const { return smooth_angle_init_; }
  const SmoothInit& smooth_scalar_init() const { return smooth_scalar_init_; }
  const CompactSpline& simple_spline() const { return simple_spline_; }
  const CompactSpline* simple_splines(MotiveDimension dimension) const {
    assert(static_cast<size_t>(dimension) <=
           MOTIVE_ARRAY_SIZE(simple_splines_));
    return simple_splines_;
  }

  template <class MotivatorT>
  void InitMotivator(const MotivatorInit& init, float start_value,
                     float start_velocity, float target_value,
                     MotivatorT* motivator) {
    typedef typename MotivatorT::TargetBuilder Tar;
    typedef typename MotivatorT::ExT Vec;
    motivator->InitializeWithTarget(
        init, &engine_,
        Tar::CurrentToTarget(Vec(start_value), Vec(start_velocity),
                             Vec(target_value), Vec(0.0f), 1));
  }

  template <class MotivatorT>
  void InitOvershootMotivator(MotivatorT* motivator) {
    InitMotivator(overshoot_percent_init_, overshoot_percent_init_.Max(),
                  overshoot_percent_init_.max_velocity(),
                  overshoot_percent_init_.Max(), motivator);
  }

  template <class MotivatorT>
  void InitOvershootMotivatorArray(MotivatorT* motivators, int len) {
    for (int i = 0; i < len; ++i) {
      InitOvershootMotivator(&motivators[i]);
    }
  }

  template <class MotivatorT>
  MotiveTime TimeToSettle(const MotivatorT& motivator,
                          const Settled1f& settled) {
    MotiveTime time = 0;
    while (time < kMaxTime && !settled.Settled(motivator)) {
      engine_.AdvanceFrame(kTimePerFrame);
      time += kTimePerFrame;
    }
    return time;
  }

 protected:
  virtual void SetUp() {
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

    // Create a simple spline from time 0~kEndTime. The y-values don't really
    // matter.
    const float kSimpleSplineEndTime = 1000;
    simple_spline_.Init(Range(-1.0f, 1.0f), 1.0f, 3);
    simple_spline_.AddNode(0.0f, -1.0f, 0.001f);
    simple_spline_.AddNode(0.5f * kSimpleSplineEndTime, 0.5f, 0.0f);
    simple_spline_.AddNode(kSimpleSplineEndTime, 1.0f, 0.001f);

    // Duplicate simple_spline_ for multi-dimensional testing.
    for (size_t i = 0; i < MOTIVE_ARRAY_SIZE(simple_splines_); ++i) {
      simple_splines_[i] = simple_spline_;
    }
  }
  virtual void TearDown() {}

  MotiveEngine engine_;
  OvershootInit overshoot_angle_init_;
  OvershootInit overshoot_percent_init_;
  SmoothInit smooth_angle_init_;
  SmoothInit smooth_scalar_init_;
  CompactSpline simple_spline_;
  CompactSpline simple_splines_[4];
};

// Ensure we wrap around from pi to -pi.
template <class MotivatorT>
void ModularMovement(MotiveTests& t) {
  typedef typename MotivatorT::ExT Vec;
  const OvershootInit& overshoot = t.overshoot_angle_init();
  MotivatorT motivator;
  t.InitMotivator(overshoot, kPi, 0.001f, -kPi + 1.0f, &motivator);
  t.engine().AdvanceFrame(1);

  // We expect the position to go up from +pi since it has positive velocity.
  // Since +pi is the max of the range, we expect the value to wrap down to -pi.
  EXPECT_TRUE(motivator.Value() <= Vec(0.0f));
}
TEST_ALL_VECTOR_MOTIVATORS_F(ModularMovement)

// Ensure the simulation settles on the target in a reasonable amount of time.
template <class MotivatorT>
void EventuallySettles(MotiveTests& t) {
  const OvershootInit& overshoot = t.overshoot_angle_init();
  MotivatorT motivator;
  t.InitMotivator(overshoot, 0.0f, overshoot.max_velocity(), -kPi + 1.0f,
                  &motivator);
  const MotiveTime time_to_settle =
      t.TimeToSettle(motivator, overshoot.at_target());

  // The simulation should complete in about half a second (time is in ms).
  // Checke that it doesn't finish too quickly nor too slowly.
  EXPECT_GT(time_to_settle, 0);
  EXPECT_LT(time_to_settle, 700);
}
TEST_ALL_VECTOR_MOTIVATORS_F(EventuallySettles)

// Ensure the simulation settles when the target is the max bound in a modular
// type. It will oscillate between the max and min bound a lot.
template <class MotivatorT>
void SettlesOnMax(MotiveTests& t) {
  MotivatorT motivator;
  const OvershootInit& overshoot = t.overshoot_angle_init();
  t.InitMotivator(overshoot, kPi, overshoot.max_velocity(), kPi, &motivator);
  const MotiveTime time_to_settle =
      t.TimeToSettle(motivator, overshoot.at_target());

  // The simulation should complete in about half a second (time is in ms).
  // Checke that it doesn't finish too quickly nor too slowly.
  EXPECT_GT(time_to_settle, 0);
  EXPECT_LT(time_to_settle, 500);
}
TEST_ALL_VECTOR_MOTIVATORS_F(SettlesOnMax)

// Ensure the simulation does not exceed the max bound, on constrants that
// do not wrap around.
template <class MotivatorT>
void StaysWithinBound(MotiveTests& t) {
  typedef typename MotivatorT::ExT Vec;
  MotivatorT motivator;
  t.InitOvershootMotivator(&motivator);
  t.engine().AdvanceFrame(1);

  // Even though we're at the bound and trying to travel beyond the bound,
  // the simulation should clamp our position to the bound.
  EXPECT_TRUE(
      VectorEqual(motivator.Value(), Vec(t.overshoot_percent_init().Max())));
}
TEST_ALL_VECTOR_MOTIVATORS_F(StaysWithinBound)

// Open up a hole in the data and then call Defragment() to close it.
template <class MotivatorT>
void Defragment(MotiveTests& t) {
  MotivatorT motivators[4];
  const int len = static_cast<int>(MOTIVE_ARRAY_SIZE(motivators));
  for (int hole = 0; hole < len; ++hole) {
    t.InitOvershootMotivatorArray(motivators, len);

    // Invalidate motivator at index 'hole'.
    motivators[hole].Invalidate();
    EXPECT_FALSE(motivators[hole].Valid());

    // Defragment() is called at the start of AdvanceFrame.
    t.engine().AdvanceFrame(1);
    EXPECT_FALSE(motivators[hole].Valid());

    // Compare the remaining motivators against each other.
    const int compare = hole == 0 ? 1 : 0;
    EXPECT_TRUE(motivators[compare].Valid());
    for (int i = 0; i < len; ++i) {
      if (i == hole || i == compare) continue;

      // All the motivators should be valid and have the same values.
      EXPECT_TRUE(motivators[i].Valid());
      EXPECT_TRUE(
          VectorEqual(motivators[i].Value(), motivators[compare].Value()));
      EXPECT_TRUE(VectorEqual(motivators[i].Velocity(),
                              motivators[compare].Velocity()));
      EXPECT_TRUE(VectorEqual(motivators[i].TargetValue(),
                              motivators[compare].TargetValue()));
    }
  }
}
TEST_ALL_VECTOR_MOTIVATORS_F(Defragment)

// Copy a valid motivator. Ensure original motivator gets invalidated.
template <class MotivatorT>
void CopyConstructor(MotiveTests& t) {
  MotivatorT orig_motivator;
  t.InitOvershootMotivator(&orig_motivator);
  EXPECT_TRUE(orig_motivator.Valid());
  const typename MotivatorT::ExT value = orig_motivator.Value();

  MotivatorT new_motivator(orig_motivator);
  EXPECT_FALSE(orig_motivator.Valid());
  EXPECT_TRUE(new_motivator.Valid());
  EXPECT_TRUE(VectorEqual(new_motivator.Value(), value));
}
TEST_ALL_VECTOR_MOTIVATORS_F(CopyConstructor)

// Copy an invalid motivator.
template <class MotivatorT>
void CopyConstructorInvalid(MotiveTests& /*t*/) {
  MotivatorT invalid_motivator;
  EXPECT_FALSE(invalid_motivator.Valid());

  MotivatorT copy_of_invalid(invalid_motivator);
  EXPECT_FALSE(copy_of_invalid.Valid());
}
TEST_ALL_VECTOR_MOTIVATORS_F(CopyConstructorInvalid)

// Test operator=() of an invalid motivator to another invalid motivator.
template <class MotivatorT>
void AssignmentOperatorInvalidToInvalid(MotiveTests& /*t*/) {
  MotivatorT orig_motivator;
  MotivatorT new_motivator;
  EXPECT_FALSE(orig_motivator.Valid());
  EXPECT_FALSE(orig_motivator.Valid());

  new_motivator = orig_motivator;
  EXPECT_FALSE(orig_motivator.Valid());
  EXPECT_FALSE(new_motivator.Valid());
}
TEST_ALL_VECTOR_MOTIVATORS_F(AssignmentOperatorInvalidToInvalid)

// Test operator=() of a valid motivator to an invalid motivator.
template <class MotivatorT>
void AssignmentOperatorValidToInvalid(MotiveTests& t) {
  MotivatorT orig_motivator;
  t.InitOvershootMotivator(&orig_motivator);
  EXPECT_TRUE(orig_motivator.Valid());
  const typename MotivatorT::ExT value = orig_motivator.Value();

  MotivatorT new_motivator;
  new_motivator = orig_motivator;
  EXPECT_FALSE(orig_motivator.Valid());
  EXPECT_TRUE(new_motivator.Valid());
  EXPECT_TRUE(VectorEqual(new_motivator.Value(), value));
}
TEST_ALL_VECTOR_MOTIVATORS_F(AssignmentOperatorValidToInvalid)

// Test operator=() of an invalid motivator to a valid motivator.
template <class MotivatorT>
void AssignmentOperatorInvalidToValid(MotiveTests& t) {
  MotivatorT orig_motivator;
  EXPECT_FALSE(orig_motivator.Valid());

  MotivatorT new_motivator;
  t.InitOvershootMotivator(&new_motivator);

  new_motivator = orig_motivator;

  EXPECT_FALSE(orig_motivator.Valid());
  EXPECT_FALSE(new_motivator.Valid());
}
TEST_ALL_VECTOR_MOTIVATORS_F(AssignmentOperatorInvalidToValid)

// Test operator=() of a valid motivator to another valid motivator.
template <class MotivatorT>
void AssignmentOperatorValidToValid(MotiveTests& t) {
  typedef typename MotivatorT::TargetBuilder Tar;
  typedef typename MotivatorT::ExT Vec;

  MotivatorT orig_motivator;
  t.InitOvershootMotivator(&orig_motivator);
  EXPECT_TRUE(orig_motivator.Valid());
  const typename MotivatorT::ExT orig_value = orig_motivator.Value();

  MotivatorT new_motivator;
  new_motivator.InitializeWithTarget(t.overshoot_angle_init(), &t.engine(),
                                     Tar::Current(Vec(orig_value + 1)));
  EXPECT_TRUE(new_motivator.Valid());
  const typename MotivatorT::ExT new_value = new_motivator.Value();

  // Give orig and new different values.
  EXPECT_FALSE(VectorEqual(new_value, orig_value));

  new_motivator = orig_motivator;

  // After the assignment, new should have the orig value.
  EXPECT_FALSE(orig_motivator.Valid());
  EXPECT_TRUE(new_motivator.Valid());
  EXPECT_TRUE(VectorEqual(new_motivator.Value(), orig_value));
}
TEST_ALL_VECTOR_MOTIVATORS_F(AssignmentOperatorValidToValid)

template <class MotivatorT>
void VectorResize(MotiveTests& t) {
  static const int kStartSize = 4;
  std::vector<MotivatorT> motivators(kStartSize);

  // Create the motivators and ensure that they're valid.
  for (int i = 0; i < kStartSize; ++i) {
    t.InitOvershootMotivator(&motivators[i]);
    EXPECT_TRUE(motivators[i].Valid());
  }

  // Expand the size of 'motivators'. This should force the array to be
  // reallocated and all motivators in the array to be moved.
  const MotivatorT* orig_address = &motivators[0];
  motivators.resize(kStartSize + 1);
  const MotivatorT* new_address = &motivators[0];
  EXPECT_NE(orig_address, new_address);

  // All the move motivators should still be valid.
  for (int i = 0; i < kStartSize; ++i) {
    t.InitOvershootMotivator(&motivators[i]);
    EXPECT_TRUE(motivators[i].Valid());
  }
}
TEST_ALL_VECTOR_MOTIVATORS_F(VectorResize)

template <class MotivatorT>
void SmoothModular(MotiveTests& t) {
  typedef typename MotivatorT::TargetBuilder Tar;
  typedef typename MotivatorT::ExT Vec;

  static const float kMargin = 0.1f;
  static const MotiveTime kTime = 10;
  static const float kStart = kPi - kMargin;
  static const float kEnd = -kPi + kMargin;
  MotivatorT angle(t.smooth_angle_init(), &t.engine(),
                   Tar::CurrentToTarget(Vec(kStart), Vec(0.0f), Vec(kEnd),
                                        Vec(0.0f), kTime));

  // The difference should be the short way around, across kPi.
  EXPECT_TRUE(VectorNear(angle.Value(), Vec(kStart), Vec(kAngleEpsilon)));
  EXPECT_TRUE(
      VectorNear(angle.Difference(), Vec(2.0f * kMargin), Vec(kAngleEpsilon)));

  // Ensure that we're always near kPi, never near 0. We want to go the
  // short way around.
  for (MotiveTime time = 0; time < kTime; ++time) {
    EXPECT_TRUE(Vec(kStart - kAngleEpsilon) <= angle.Value() ||
                angle.Value() <= Vec(kEnd + kAngleEpsilon));
    t.engine().AdvanceFrame(1);
  }
  const Vec normalized = VectorNormalize(angle.Value());
  EXPECT_TRUE(VectorNear(normalized, Vec(kEnd), Vec(kAngleEpsilon)));
}
TEST_ALL_VECTOR_MOTIVATORS_F(SmoothModular)

// Print matrices with columns vertically.
static void PrintMatrix(const char* name, const mat4& m) {
  (void)name;
  (void)m;
#if DEBUG_PRINT_MATRICES
  printf("%s\n(%f %f %f %f)\n(%f %f %f %f)\n(%f %f %f %f)\n(%f %f %f %f)\n",
         name, m[0], m[4], m[8], m[12], m[1], m[5], m[9], m[13], m[2], m[6],
         m[10], m[14], m[3], m[7], m[11], m[15]);
#endif  // DEBUG_PRINT_MATRICES
}

// Create a matrix that performs the transformation specified in 'op_init'.
static mat4 CreateMatrixFromOp(const MatrixOperationInit& op_init) {
  const float v = op_init.initial_value;

  switch (op_init.type) {
    case kRotateAboutX:
      return mat4::FromRotationMatrix(mat3::RotationX(v));
    case kRotateAboutY:
      return mat4::FromRotationMatrix(mat3::RotationY(v));
    case kRotateAboutZ:
      return mat4::FromRotationMatrix(mat3::RotationZ(v));
    case kTranslateX:
      return mat4::FromTranslationVector(vec3(v, 0.0f, 0.0f));
    case kTranslateY:
      return mat4::FromTranslationVector(vec3(0.0f, v, 0.0f));
    case kTranslateZ:
      return mat4::FromTranslationVector(vec3(0.0f, 0.0f, v));
    case kScaleX:
      return mat4::FromScaleVector(vec3(v, 1.0f, 1.0f));
    case kScaleY:
      return mat4::FromScaleVector(vec3(1.0f, v, 1.0f));
    case kScaleZ:
      return mat4::FromScaleVector(vec3(1.0f, 1.0f, v));
    case kScaleUniformly:
      return mat4::FromScaleVector(vec3(v));
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

static void ExpectMatricesEqual(const mat4& a, const mat4& b, float epsilon) {
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      EXPECT_NEAR(a(i, j), b(i, j), epsilon);
    }
  }
}

static void TestMatrixMotivator(const MatrixInit& matrix_init,
                                MotiveEngine* engine) {
  MatrixMotivator4f matrix_motivator(matrix_init, engine);
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
  MatrixOpArray ops(1);
  ops.AddOp(motive::kTranslateX, smooth_scalar_init_, 2.0f);
  TestMatrixMotivator(MatrixInit(ops), &engine_);
}

// Don't use an motivator to drive the animation. Use a constant value.
TEST_F(MotiveTests, MatrixTranslateXConstValue) {
  MatrixOpArray ops(1);
  ops.AddOp(motive::kTranslateX, 2.0f);
  TestMatrixMotivator(MatrixInit(ops), &engine_);
}

// Test the matrix operation kRotateAboutX.
TEST_F(MotiveTests, MatrixRotateAboutX) {
  MatrixOpArray ops(1);
  ops.AddOp(motive::kRotateAboutX, smooth_angle_init_, kHalfPi);
  TestMatrixMotivator(MatrixInit(ops), &engine_);
}

// Test the matrix operation kRotateAboutY.
TEST_F(MotiveTests, MatrixRotateAboutY) {
  MatrixOpArray ops(1);
  ops.AddOp(motive::kRotateAboutY, smooth_angle_init_, kHalfPi / 3.0f);
  TestMatrixMotivator(MatrixInit(ops), &engine_);
}

// Test the matrix operation kRotateAboutZ.
TEST_F(MotiveTests, MatrixRotateAboutZ) {
  MatrixOpArray ops(1);
  ops.AddOp(motive::kRotateAboutZ, smooth_angle_init_, -kHalfPi / 1.2f);
  TestMatrixMotivator(MatrixInit(ops), &engine_);
}

// Test the matrix operation kScaleX.
TEST_F(MotiveTests, MatrixScaleX) {
  MatrixOpArray ops(1);
  ops.AddOp(motive::kScaleX, smooth_scalar_init_, -3.0f);
  TestMatrixMotivator(MatrixInit(ops), &engine_);
}

// Test the series of matrix operations for translating XYZ.
TEST_F(MotiveTests, MatrixTranslateXYZ) {
  MatrixOpArray ops(3);
  ops.AddOp(motive::kTranslateX, smooth_scalar_init_, 2.0f);
  ops.AddOp(motive::kTranslateY, smooth_scalar_init_, -3.0f);
  ops.AddOp(motive::kTranslateZ, smooth_scalar_init_, 0.5f);
  TestMatrixMotivator(MatrixInit(ops), &engine_);
}

// Test the series of matrix operations for rotating about X, Y, and Z,
// in turn.
TEST_F(MotiveTests, MatrixRotateAboutXYZ) {
  MatrixOpArray ops(3);
  ops.AddOp(motive::kRotateAboutX, smooth_angle_init_, -kHalfPi / 2.0f);
  ops.AddOp(motive::kRotateAboutY, smooth_angle_init_, kHalfPi / 3.0f);
  ops.AddOp(motive::kRotateAboutZ, smooth_angle_init_, kHalfPi / 5.0f);
  TestMatrixMotivator(MatrixInit(ops), &engine_);
}

// Test the series of matrix operations for scaling XYZ non-uniformly.
TEST_F(MotiveTests, MatrixScaleXYZ) {
  MatrixOpArray ops(3);
  ops.AddOp(motive::kScaleX, smooth_scalar_init_, -3.0f);
  ops.AddOp(motive::kScaleY, smooth_scalar_init_, 2.2f);
  ops.AddOp(motive::kScaleZ, smooth_scalar_init_, 1.01f);
  TestMatrixMotivator(MatrixInit(ops), &engine_);
}

// Test the matrix operation kScaleUniformly.
TEST_F(MotiveTests, MatrixScaleUniformly) {
  MatrixOpArray ops(1);
  ops.AddOp(motive::kScaleUniformly, smooth_scalar_init_, 10.1f);
  TestMatrixMotivator(MatrixInit(ops), &engine_);
}

// Test the series of matrix operations for translating and rotating.
TEST_F(MotiveTests, MatrixTranslateRotateTranslateBack) {
  MatrixOpArray ops(3);
  ops.AddOp(motive::kTranslateY, smooth_scalar_init_, 1.0f);
  ops.AddOp(motive::kRotateAboutX, smooth_angle_init_, kHalfPi);
  ops.AddOp(motive::kTranslateY, smooth_scalar_init_, -1.0f);
  TestMatrixMotivator(MatrixInit(ops), &engine_);
}

// Test the series of matrix operations for translating, rotating, and scaling.
TEST_F(MotiveTests, MatrixTranslateRotateScale) {
  MatrixOpArray ops(3);
  ops.AddOp(motive::kTranslateY, smooth_scalar_init_, 1.0f);
  ops.AddOp(motive::kRotateAboutX, smooth_angle_init_, kHalfPi);
  ops.AddOp(motive::kScaleZ, smooth_scalar_init_, -1.4f);
  TestMatrixMotivator(MatrixInit(ops), &engine_);
}

// Test a complex the series of matrix operations.
TEST_F(MotiveTests, MatrixTranslateRotateScaleGoneWild) {
  MatrixOpArray ops(16);
  ops.AddOp(motive::kTranslateY, smooth_scalar_init_, 1.0f);
  ops.AddOp(motive::kTranslateX, smooth_scalar_init_, -1.6f);
  ops.AddOp(motive::kRotateAboutX, smooth_angle_init_, kHalfPi * 0.1f);
  ops.AddOp(motive::kRotateAboutY, smooth_angle_init_, kHalfPi * 0.33f);
  ops.AddOp(motive::kScaleZ, smooth_scalar_init_, -1.4f);
  ops.AddOp(motive::kRotateAboutY, smooth_angle_init_, -kHalfPi * 0.33f);
  ops.AddOp(motive::kTranslateX, smooth_scalar_init_, -1.2f);
  ops.AddOp(motive::kTranslateY, smooth_scalar_init_, -1.5f);
  ops.AddOp(motive::kTranslateZ, smooth_scalar_init_, -2.2f);
  ops.AddOp(motive::kRotateAboutZ, smooth_angle_init_, -kHalfPi * 0.5f);
  ops.AddOp(motive::kScaleX, smooth_scalar_init_, 2.0f);
  ops.AddOp(motive::kScaleY, smooth_scalar_init_, 4.1f);
  TestMatrixMotivator(MatrixInit(ops), &engine_);
}

// Test the MotivatorVector::SplineTime() function.
template <class MotivatorT>
void SplineTime(MotiveTests& t) {
  static const MotiveTime kStartTime = 250;
  static const MotiveTime kDeltaTime = 500;

  const motive::CompactSpline* splines =
      t.simple_splines(MotivatorT::kDimensions);
  const MotiveTime end_time = static_cast<MotiveTime>(splines[0].EndX());

  // Two updates of kDeltaTime should wrap past end_time.
  assert(kStartTime + 2 * kDeltaTime > end_time);

  // Create a motivator that plays `spline` from kStartTime, and repeats
  // at the start when it reaches the end.
  MotivatorT angle(t.smooth_angle_init(), &t.engine());
  angle.SetSplines(splines, SplinePlayback(kStartTime, true));

  // When we start, the spline time should be the same as the start time.
  EXPECT_EQ(angle.SplineTime(), kStartTime);

  // Since the playback rate is 1, we expect the spline time to advance with
  // the delta time.
  t.engine().AdvanceFrame(kDeltaTime);
  EXPECT_EQ(angle.SplineTime(), kStartTime + kDeltaTime);

  // Since repeat=true in SetSpline(), we expect to wrap around once we
  // advance past end_time.
  t.engine().AdvanceFrame(kDeltaTime);
  EXPECT_EQ(angle.SplineTime(), (kStartTime + 2 * kDeltaTime) % end_time);
}
TEST_ALL_VECTOR_MOTIVATORS_F(SplineTime)

// Test the MotivatorVector::SetSplineTime() function.
template <class MotivatorT>
void SetSplineTime(MotiveTests& t) {
  typedef typename MotivatorT::ExT Vec;
  static const MotiveTime kStartTime = 250;
  static const MotiveTime kDeltaTime = 500;

  const motive::CompactSpline* splines =
      t.simple_splines(MotivatorT::kDimensions);
  const MotiveTime end_time = static_cast<MotiveTime>(splines[0].EndX());

  // Create a motivator that plays `spline` from kStartTime.
  MotivatorT angle(t.smooth_angle_init(), &t.engine());
  angle.SetSplines(splines, SplinePlayback(kStartTime));

  // Set to time 0.
  const float start_value = splines[0].StartY();
  angle.SetSplineTime(0);
  EXPECT_EQ(angle.SplineTime(), 0);
  EXPECT_TRUE(VectorNear(angle.Value(), Vec(start_value), Vec(kAngleEpsilon)));

  // Advance to double-check that the Motivator was properly re-initialized.
  t.engine().AdvanceFrame(kDeltaTime);
  EXPECT_EQ(angle.SplineTime(), kDeltaTime);

  // Set to the end time.
  const float end_value = splines[0].EndY();
  angle.SetSplineTime(end_time);
  EXPECT_EQ(angle.SplineTime(), end_time);
  EXPECT_TRUE(VectorNear(angle.Value(), Vec(end_value), Vec(kAngleEpsilon)));

  // Set to a spline node time.
  const MotiveTime mid_time = static_cast<MotiveTime>(splines[0].NodeX(1));
  const float mid_value = splines[0].NodeY(1);
  angle.SetSplineTime(mid_time);
  EXPECT_EQ(angle.SplineTime(), mid_time);
  EXPECT_TRUE(VectorNear(angle.Value(), Vec(mid_value), Vec(kAngleEpsilon)));

  // Set back to the start time.
  angle.SetSplineTime(kStartTime);
  EXPECT_EQ(angle.SplineTime(), kStartTime);
}
TEST_ALL_VECTOR_MOTIVATORS_F(SetSplineTime)

// Test the MotivatorVector::PlaybackRate() function.
template <class MotivatorT>
void PlaybackRate(MotiveTests& t) {
  static const MotiveTime kDeltaTime = 10;
  static const float kPlaybackRates[] = {0.0f, 0.5f, 1.0f, 2.0f};
  static const float kMidPlaybackRateOffset = 0.1f;

  const motive::CompactSpline* splines =
      t.simple_splines(MotivatorT::kDimensions);
  const MotiveTime end_time = static_cast<MotiveTime>(splines[0].EndX());

  // Create a motivator that plays `spline` from kStartTime, and repeats
  // at the start when it reaches the end.
  MotivatorT angle(t.smooth_angle_init(), &t.engine());

  for (size_t i = 0; i < MOTIVE_ARRAY_SIZE(kPlaybackRates); ++i) {
    const float playback_rate = kPlaybackRates[i];
    angle.SetSplines(splines, SplinePlayback(0.0f, true, playback_rate));

    // Since the playback rate is 1, we expect the spline time to advance with
    // the delta time.
    t.engine().AdvanceFrame(kDeltaTime);
    EXPECT_EQ(angle.SplineTime(),
              static_cast<MotiveTime>(kDeltaTime * playback_rate) % end_time);
    EXPECT_TRUE(VectorUniform(angle.Value()));

    // Change the playback rate mid-playback.
    const MotiveTime mid_spline_time = angle.SplineTime();
    const float mid_playback_rate = playback_rate + kMidPlaybackRateOffset;
    angle.SetSplinePlaybackRate(mid_playback_rate);
    t.engine().AdvanceFrame(kDeltaTime);
    EXPECT_EQ(angle.SplineTime(),
              (mid_spline_time +
               static_cast<MotiveTime>(kDeltaTime * mid_playback_rate)) %
                  end_time);
    EXPECT_TRUE(VectorUniform(angle.Value()));
  }
}
TEST_ALL_VECTOR_MOTIVATORS_F(PlaybackRate)

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
