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

#include "flatbuffers/flatbuffers.h"
#include "gtest/gtest.h"
#include "mathfu/constants.h"
#include "motive/common.h"
#include "motive/const_init.h"
#include "motive/ease_in_ease_out_init.h"
#include "motive/engine.h"
#include "motive/math/angle.h"
#include "motive/math/curve_util.h"
#include "motive/matrix_init.h"
#include "motive/matrix_motivator.h"
#include "motive/matrix_op.h"
#include "motive/overshoot_init.h"
#include "motive/spline_init.h"
#include "motive/sqt_init.h"

#define DEBUG_PRINT_MATRICES 0

using mathfu::mat4;
using mathfu::vec2;
using mathfu::vec2i;
using mathfu::vec3;
using mathfu::vec4;
using motive::Angle;
using motive::CompactSpline;
using motive::EaseInEaseOutInit;
using motive::EaseInEaseOutInit1f;
using motive::kAngleRange;
using motive::kHalfPi;
using motive::kInvalidRange;
using motive::kPi;
using motive::kRotateAboutX;
using motive::kRotateAboutY;
using motive::kRotateAboutZ;
using motive::kScaleUniformly;
using motive::kScaleX;
using motive::kScaleY;
using motive::kScaleZ;
using motive::kTranslateX;
using motive::kTranslateY;
using motive::kTranslateZ;
using motive::MathFuVectorConverter;
using motive::MatrixInit;
using motive::MatrixMotivator4f;
using motive::MatrixOperationInit;
using motive::MatrixOperationType;
using motive::Motivator1f;
using motive::Motivator2f;
using motive::Motivator3f;
using motive::Motivator4f;
using motive::MotivatorInit;
using motive::MotiveCurveShape;
using motive::MotiveDimension;
using motive::MotiveEngine;
using motive::MotiveTarget1f;
using motive::MotiveTarget2f;
using motive::MotiveTarget3f;
using motive::MotiveTarget4f;
using motive::MotiveTime;
using motive::OvershootInit;
using motive::Range;
using motive::Settled1f;
using motive::SimpleInitTemplate;
using motive::SplineInit;
using motive::SplinePlayback;
using motive::SqtInit;

typedef mathfu::Matrix<float, 3> mat3;

static const MotiveTime kTimePerFrame = 10;
static const MotiveTime kMaxTime = 10000;
static const float kMatrixEpsilon = 0.001f;
static const float kAngleEpsilon = 0.01f;
static const int kNumCheckPoints = motive::kDefaultGraphWidth;
static const vec2i kGraphSize(kNumCheckPoints, motive::kDefaultGraphHeight);
static const float kEpsilonScale = 0.001f;

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
  const SplineInit& spline_angle_init() const { return spline_angle_init_; }
  const SplineInit& smooth_scalar_init() const { return spline_scalar_init; }
  const EaseInEaseOutInit& ease_init_() const { return ease_init__; }
  const CompactSpline& simple_spline() const { return simple_spline_; }
  const CompactSpline* simple_splines(MotiveDimension dimension) const {
    assert(static_cast<size_t>(dimension) <=
           MOTIVE_ARRAY_SIZE(simple_splines_));
    (void)dimension;
    return simple_splines_;
  }

  template <class MotivatorT>
  void InitMotivator(const MotivatorInit& init, float start_value,
                     float start_velocity, float target_value,
                     MotiveTime target_time, MotivatorT* motivator) {
    typedef typename MotivatorT::TargetBuilder Tar;
    typedef typename MotivatorT::Vec Vec;
    motivator->InitializeWithTarget(
        init, &engine_,
        Tar::CurrentToTarget(Vec(start_value), Vec(start_velocity),
                             Vec(target_value), Vec(0.0f), target_time));
  }

  template <class MotivatorT>
  void InitEaseInEaseOutMotivator(const MotivatorInit& init, float target_value,
                                  float target_velocity,
                                  const MotiveCurveShape& shape,
                                  MotivatorT* motivator) {
    typedef typename MotivatorT::Vec Vec;
    motivator->InitializeWithTargetShape(
        init, &engine_, motivator->Dimensions(), shape, Vec(target_value),
        Vec(target_velocity));
  }

  template <class MotivatorT>
  void InitOvershootMotivator(MotivatorT* motivator) {
    InitMotivator(overshoot_percent_init_,
                  overshoot_percent_init_.range().start(),
                  overshoot_percent_init_.max_velocity(),
                  overshoot_percent_init_.range().end(), 1, motivator);
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
    motive::OvershootInit::Register();
    motive::SplineInit::Register();
    motive::EaseInEaseOutInit::Register();
    motive::MatrixInit::Register();
    motive::SqtInit::Register();

    // Create an OvershootInit with reasonable values.
    overshoot_angle_init_.set_modular(true);
    overshoot_angle_init_.set_range(kAngleRange);
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

    spline_angle_init_.set_range(kAngleRange);
    spline_scalar_init.set_range(kInvalidRange);

    // Create a simple spline from time 0~kEndTime. The y-values don't really
    // matter.
    const float kSimpleSplineEndTime = 1000;
    simple_spline_.Init(Range(-1.0f, 1.0f), 1.0f);
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
  SplineInit spline_angle_init_;
  SplineInit spline_scalar_init;
  EaseInEaseOutInit ease_init__;
  CompactSpline simple_spline_;
  CompactSpline simple_splines_[4];
};

template <class MotivatorT>
static void TestEaseInEaseOutInternal(float start_value, float start_velocity,
                                      float target_value, float target_velocity,
                                      const MotiveCurveShape& shape,
                                      MotiveTime delta_time,
                                      bool test_with_set_target_every_frame,
                                      MotiveTests* t) {
  typedef typename MotivatorT::TargetBuilder Tar;
  typedef typename MotivatorT::Vec Vec;
  typedef SimpleInitTemplate<EaseInEaseOutInit, MathFuVectorConverter,
                             MotivatorT::kDimensions> Init;

  const float value_epsilon = std::fabs(target_value) * kEpsilonScale;
  const float velocity_epsilon = std::fabs(target_velocity) * kEpsilonScale;

  // Set up a motivator with the desired start values.
  MotivatorT motivator;

  // Note, the parenthesis around the first Vec are required to keep
  // the compiler from misinterpreting the line below. See
  // "the most vexing parse" of C++ for more detail.
  // https://en.wikipedia.org/wiki/Most_vexing_parse
  const Init ease_in_ease_out_init((Vec(start_value)), Vec(start_velocity));
  t->InitEaseInEaseOutMotivator(ease_in_ease_out_init, target_value,
                                target_velocity, shape, &motivator);

  EXPECT_TRUE(
      VectorNear(Vec(start_value), motivator.Value(), Vec(value_epsilon)));
  EXPECT_TRUE(VectorNear(Vec(start_velocity), motivator.Velocity(),
                         Vec(velocity_epsilon)));
  EXPECT_TRUE(VectorNear(Vec(target_value), motivator.TargetValue(),
                         Vec(value_epsilon)));

  const float delta_time_float = static_cast<float>(delta_time);
  float current_velocity[MotivatorT::kDimensions];
  motivator.Velocities(current_velocity);
  float past_velocity = std::numeric_limits<float>::infinity();
  float past_value = std::numeric_limits<float>::infinity();

  // Create a vector of points for printing the graph.
  std::vector<vec2> points;
  points.push_back(vec2(0.0f, motivator.Values()[0]));

  // Advance frame more so we go past
  // the total_x. We want to test the behavior
  // of it past the original total x.
  while (current_velocity[0] != past_velocity &&
         motivator.Values()[0] != past_value) {
    past_velocity = current_velocity[0];
    past_value = motivator.Values()[0];
    t->engine().AdvanceFrame(delta_time);
    motivator.Velocities(current_velocity);

    // Setting the target to the same target should have no affect.
    if (test_with_set_target_every_frame && motivator.TargetTime() > 0.0f) {
      const MotiveTime target_time = motivator.TargetTime();
      motivator.SetTarget(Tar::CurrentToTarget(
          Vec(motivator.Values()[0]), Vec(current_velocity[0]),
          Vec(target_value), Vec(target_velocity), 1));
      EXPECT_EQ(target_time, motivator.TargetTime());
    }
    points.push_back(
        vec2(points.size() * delta_time_float, motivator.Values()[0]));
  }

  // Go another kPointsPastZeroVelocity ticks past reaching 0 velocity.
  const int kPointsPastZeroVelocity = 30;
  for (int j = 0; j < kPointsPastZeroVelocity; ++j) {
    t->engine().AdvanceFrame(delta_time);
    EXPECT_GE(0, motivator.TargetTime());
    EXPECT_TRUE(VectorEqual(motivator.Velocity(), Vec(0.0f)));
    points.push_back(
        vec2(points.size() * delta_time_float, motivator.Values()[0]));
  }
  EXPECT_TRUE(
      VectorNear(Vec(target_value), motivator.Value(), Vec(value_epsilon)));
#ifdef MOTIVE_OUTPUT_DEBUG_CURVES_IN_TESTS
  printf("\n%s\n\n",
         motive::Graph2DPoints(&points[0], static_cast<int>(points.size()),
                               kGraphSize)
             .c_str());
#endif  // MOTIVE_OUTPUT_DEBUG_CURVES_IN_TESTS
}

template <class MotivatorT>
static void TestEaseInEaseOut(float start_value, float start_velocity,
                              float target_value, float target_velocity,
                              float typical_delta_value,
                              float typical_total_time, float bias,
                              MotiveTime delta_x, MotiveTests* t) {
  const MotiveCurveShape shape(typical_delta_value, typical_total_time, bias);
  TestEaseInEaseOutInternal<MotivatorT>(start_value, start_velocity,
                                        target_value, target_velocity, shape,
                                        delta_x, true, t);
  TestEaseInEaseOutInternal<MotivatorT>(start_value, start_velocity,
                                        target_value, target_velocity, shape,
                                        delta_x, false, t);
}

template <class MotivatorT>
static void TestMultiMotivators(float start_value, float start_velocity,
                                float target_value, float target_velocity,
                                float typical_delta_value,
                                float typical_total_time, float bias,
                                MotiveTime delta_x, MotiveTests* t) {
  typedef typename MotivatorT::Vec Vec;
  typedef SimpleInitTemplate<EaseInEaseOutInit, MathFuVectorConverter,
                             MotivatorT::kDimensions> Init;
  const int kNumTestMotivators = 100;
  const MotiveCurveShape shape(typical_delta_value, typical_total_time, bias);

  // Create test motivators to exist while
  // the tests are run.
  MotivatorT test_motivators[kNumTestMotivators];
  const Init ease_in_ease_out_init((Vec(start_value)), Vec(start_velocity));
  for (int i = 0; i < kNumTestMotivators; ++i) {
    test_motivators[i] = MotivatorT();
    t->InitEaseInEaseOutMotivator(ease_in_ease_out_init, target_value,
                                  target_velocity, shape, &test_motivators[i]);
  }

  TestEaseInEaseOut<MotivatorT>(start_value, start_velocity, target_value,
                                target_velocity, typical_delta_value,
                                typical_total_time, bias, delta_x, t);
}
// Simple test to create a 1 dimensional motivator, initialize it from 0~50,
// and advance it past the end.
TEST_F(MotiveTests, EaseInEaseOut1Dimension) {
  TestEaseInEaseOut<Motivator1f>(0.0f, 0.0f, 50.0f, 0.0f, 100.0f, 200.0f, 0.85f,
                                 1, this);
}

// 1D motivator with a right bias and larger than one delta_time.
TEST_F(MotiveTests, EaseInEaseOut1DimensionDeltaTime) {
  TestEaseInEaseOut<Motivator1f>(0.0f, 0.0f, 50.0f, 0.0f, 100.0f, 200.0f, 0.5f,
                                 10, this);
}

// 1D motivator with a right bias and huge delta_time.
TEST_F(MotiveTests, EaseInEaseOut1DimensionHugeDeltaTime) {
  TestEaseInEaseOut<Motivator1f>(0.0f, 0.0f, 50.0f, 0.0f, 100.0f, 200.0f, 0.5f,
                                 310, this);
}

// 1D motivator with a left bias.
TEST_F(MotiveTests, EaseInEaseOut1DimensionLeftBias) {
  TestEaseInEaseOut<Motivator1f>(0.0f, 0.0f, 50.0f, 0.0f, 87.4f, 189.0f, 0.15f,
                                 1, this);
}

// 1D motivator with a left bias and non-zero start and end velocities.
TEST_F(MotiveTests, EaseInEaseOut1DimensionLeftBiasNonZeroStartEnd) {
  TestEaseInEaseOut<Motivator1f>(0.0f, 0.2f, 75.0f, 0.67f, 87.4f, 1845.01f,
                                 0.19f, 1, this);
}

// 1D motivator with a right bias and non-zero start and end velocities.
TEST_F(MotiveTests, EaseInEaseOut1DimensionRightBiasNonZeroStartEnd) {
  TestEaseInEaseOut<Motivator1f>(0.0f, 0.1f, 23.0f, 0.8f, 11.2f, 34.0f, 0.9f, 1,
                                 this);
}

// 1D motivator with a right bias.
TEST_F(MotiveTests, EaseInEaseOut1DimensionRightBias) {
  TestEaseInEaseOut<Motivator1f>(0.0f, 0.0f, 103.4f, 0.0f, 150.2f, 93.0f, 0.75f,
                                 1, this);
}

// 1D motivator with a non-zero end velocity.
TEST_F(MotiveTests, EaseInEaseOut1DimensionNonZeroEndDerivative) {
  TestEaseInEaseOut<Motivator1f>(0.0f, 0.0f, 50.0f, 0.8f, 100.0f, 200.0f, 0.5f,
                                 1, this);
}

// 1D motivator with a negative end velocity.
TEST_F(MotiveTests, EaseInEaseOut1DimensionNonZeroFlipped) {
  TestEaseInEaseOut<Motivator1f>(0.0f, 0.0f, 50.0f, -0.8f, 100.0f, 200.0f, 0.5f,
                                 1, this);
}

// 1D motivator with a close start and end point with a left bias.
TEST_F(MotiveTests, CloseStartEnd1Dimension) {
  TestEaseInEaseOut<Motivator1f>(48.0f, 0.0f, 50.0f, 0.0f, 1.0f, 12.0f, 0.3f, 1,
                                 this);
}

// 2D motivator with a right bias.
TEST_F(MotiveTests, EaseInEaseOut2Dimension) {
  TestEaseInEaseOut<Motivator2f>(0.0f, 0.0f, 50.0f, 0.0f, 100.0f, 200.0f, 0.87f,
                                 1, this);
}

// 3D motivator with a right bias.
TEST_F(MotiveTests, EaseInEaseOut3Dimension) {
  TestEaseInEaseOut<Motivator3f>(0.0f, 0.0f, 65.3f, 0.0f, 134.0f, 89.3f, 0.60f,
                                 1, this);
}

// Test with extra active motivators.
TEST_F(MotiveTests, EaseInEaseOutMultiMotivators) {
  TestMultiMotivators<Motivator1f>(0.0f, 0.0f, 65.3f, 0.0f, 134.0f, 89.3f,
                                   0.60f, 1, this);
}

// Test with extra active, three dimensional motivators.
TEST_F(MotiveTests, EaseInEaseOut3DMultiMotivators) {
  TestMultiMotivators<Motivator3f>(0.0f, 0.0f, 65.3f, 0.0f, 134.0f, 89.3f,
                                   0.60f, 1, this);
}

// Ensure we wrap around from pi to -pi.
template <class MotivatorT>
void ModularMovement(MotiveTests& t) {
  typedef typename MotivatorT::Vec Vec;
  const OvershootInit& overshoot = t.overshoot_angle_init();
  MotivatorT motivator;
  t.InitMotivator(overshoot, kPi, 0.001f, -kPi + 1.0f, 1, &motivator);
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
  t.InitMotivator(overshoot, 0.0f, overshoot.max_velocity(), -kPi + 1.0f, 1,
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
  t.InitMotivator(overshoot, kPi, overshoot.max_velocity(), kPi, 1, &motivator);
  const MotiveTime time_to_settle =
      t.TimeToSettle(motivator, overshoot.at_target());

  // The simulation should complete in about half a second (time is in ms).
  // Checke that it doesn't finish too quickly nor too slowly.
  EXPECT_GT(time_to_settle, 0);
  EXPECT_LT(time_to_settle, 500);
}
TEST_ALL_VECTOR_MOTIVATORS_F(SettlesOnMax)

// Ensure the simulation does not exceed the max bound, on constraints that
// do not wrap around.
template <class MotivatorT>
void StaysWithinBound(MotiveTests& t) {
  typedef typename MotivatorT::Vec Vec;
  MotivatorT motivator;
  t.InitOvershootMotivator(&motivator);
  t.engine().AdvanceFrame(1000);

  // Even though we're at the bound and trying to travel beyond the bound,
  // the simulation should clamp our position to the bound.
  EXPECT_TRUE(VectorEqual(motivator.Value(),
                          Vec(t.overshoot_percent_init().range().end())));
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
      EXPECT_TRUE(motivators[i].Sane());
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

// Move a valid motivator. Ensure original motivator gets invalidated.
template <class MotivatorT>
void MoveConstructor(MotiveTests& t) {
  MotivatorT orig_motivator;
  t.InitOvershootMotivator(&orig_motivator);
  EXPECT_TRUE(orig_motivator.Valid());
  const typename MotivatorT::Vec value = orig_motivator.Value();

  MotivatorT new_motivator(std::move(orig_motivator));
  EXPECT_FALSE(orig_motivator.Valid());
  EXPECT_TRUE(new_motivator.Valid());
  EXPECT_TRUE(VectorEqual(new_motivator.Value(), value));
}
TEST_ALL_VECTOR_MOTIVATORS_F(MoveConstructor)

// Move an invalid motivator.
template <class MotivatorT>
void MoveConstructorInvalid(MotiveTests& /*t*/) {
  MotivatorT invalid_motivator;
  EXPECT_FALSE(invalid_motivator.Valid());

  MotivatorT copy_of_invalid(std::move(invalid_motivator));
  EXPECT_FALSE(copy_of_invalid.Valid());
}
TEST_ALL_VECTOR_MOTIVATORS_F(MoveConstructorInvalid)

// Test operator=() of an invalid motivator to another invalid motivator.
template <class MotivatorT>
void AssignmentOperatorInvalidToInvalid(MotiveTests& /*t*/) {
  MotivatorT orig_motivator;
  MotivatorT new_motivator;
  EXPECT_FALSE(orig_motivator.Valid());
  EXPECT_FALSE(new_motivator.Valid());

  new_motivator = std::move(orig_motivator);
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
  const typename MotivatorT::Vec value = orig_motivator.Value();

  MotivatorT new_motivator;
  new_motivator = std::move(orig_motivator);
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

  new_motivator = std::move(orig_motivator);

  EXPECT_FALSE(orig_motivator.Valid());
  EXPECT_FALSE(new_motivator.Valid());
}
TEST_ALL_VECTOR_MOTIVATORS_F(AssignmentOperatorInvalidToValid)

// Test operator=() of a valid motivator to another valid motivator.
template <class MotivatorT>
void AssignmentOperatorValidToValid(MotiveTests& t) {
  typedef typename MotivatorT::TargetBuilder Tar;
  typedef typename MotivatorT::Vec Vec;

  MotivatorT orig_motivator;
  t.InitOvershootMotivator(&orig_motivator);
  EXPECT_TRUE(orig_motivator.Valid());
  const typename MotivatorT::Vec orig_value = orig_motivator.Value();

  MotivatorT new_motivator;
  new_motivator.InitializeWithTarget(t.overshoot_angle_init(), &t.engine(),
                                     Tar::Current(Vec(orig_value + 1.0f)));
  EXPECT_TRUE(new_motivator.Valid());
  const typename MotivatorT::Vec new_value = new_motivator.Value();

  // Give orig and new different values.
  EXPECT_FALSE(VectorEqual(new_value, orig_value));

  new_motivator = std::move(orig_motivator);

  // After the assignment, new should have the orig value.
  EXPECT_FALSE(orig_motivator.Valid());
  EXPECT_TRUE(new_motivator.Valid());
  EXPECT_TRUE(VectorEqual(new_motivator.Value(), orig_value));
}
TEST_ALL_VECTOR_MOTIVATORS_F(AssignmentOperatorValidToValid)

// Initialize to an invalid motivator.
template <class MotivatorT>
void InitializeToInvalid(MotiveTests& t) {
  MotivatorT orig_motivator;
  EXPECT_FALSE(orig_motivator.Valid());

  MotivatorT new_motivator;
  new_motivator.CloneFrom(&orig_motivator);
  EXPECT_FALSE(orig_motivator.Valid());
  EXPECT_FALSE(new_motivator.Valid());
}
TEST_ALL_VECTOR_MOTIVATORS_F(InitializeToInvalid)

// Initialize to a valid spline motivator. Ensure the original motivator is
// still valid and that the two have matching values.
template <class MotivatorT>
void InitializeToValidSpline(MotiveTests& t) {
  typedef typename MotivatorT::Vec Vec;

  static const float kEnd = 1.f;
  static const MotiveTime kTime = 10;
  MotivatorT orig_motivator;
  t.InitMotivator(SplineInit(Range(-2.f, 2.f)), 0.f, 0.f, kEnd, kTime,
                  &orig_motivator);
  EXPECT_TRUE(orig_motivator.Valid());

  const typename MotivatorT::Vec value = orig_motivator.Value();
  const typename MotivatorT::Vec velocity = orig_motivator.Velocity();

  MotivatorT new_motivator;
  new_motivator.CloneFrom(&orig_motivator);
  EXPECT_TRUE(orig_motivator.Valid());
  EXPECT_TRUE(new_motivator.Valid());
  EXPECT_TRUE(VectorEqual(new_motivator.Value(), value));
  EXPECT_TRUE(VectorEqual(new_motivator.Velocity(), velocity));

  // Ensure that the two are always nearly identical.
  for (MotiveTime time = 0; time < kTime; ++time) {
    EXPECT_TRUE(VectorEqual(orig_motivator.Value(), new_motivator.Value()));
    EXPECT_TRUE(
        VectorEqual(orig_motivator.Velocity(), new_motivator.Velocity()));
    t.engine().AdvanceFrame(1);
  }
  EXPECT_TRUE(
      VectorNear(orig_motivator.Value(), Vec(kEnd), Vec(kEpsilonScale)));
  EXPECT_TRUE(VectorNear(new_motivator.Value(), Vec(kEnd), Vec(kEpsilonScale)));
}
TEST_ALL_VECTOR_MOTIVATORS_F(InitializeToValidSpline)

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
void SplineModular(MotiveTests& t) {
  typedef typename MotivatorT::TargetBuilder Tar;
  typedef typename MotivatorT::Vec Vec;

  static const float kMargin = 0.1f;
  static const MotiveTime kTime = 10;
  static const float kStart = kPi - kMargin;
  static const float kEnd = -kPi + kMargin;
  MotivatorT angle(t.spline_angle_init(), &t.engine(),
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
TEST_ALL_VECTOR_MOTIVATORS_F(SplineModular)

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
  const std::vector<MatrixOperationInit>& ops = matrix_init.ops();

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
  const mat4 check_matrix = CreateMatrixFromOps(matrix_init);

  const mat4 init_matrix = matrix_motivator.Value();
  ExpectMatricesEqual(init_matrix, check_matrix, kMatrixEpsilon);

  engine->AdvanceFrame(kTimePerFrame);
  const mat4 motive_matrix = matrix_motivator.Value();
  ExpectMatricesEqual(motive_matrix, check_matrix, kMatrixEpsilon);

  // Output matrices for debugging.
  PrintMatrix("motivator", motive_matrix);
  PrintMatrix("check", check_matrix);
}

// Test the matrix operation kTranslateX.
TEST_F(MotiveTests, MatrixTranslateX) {
  std::vector<MatrixOperationInit> ops;
  ops.emplace_back(0, motive::kTranslateX, spline_scalar_init, 2.0f);
  TestMatrixMotivator(MatrixInit(ops), &engine_);
}

// Don't use an motivator to drive the animation. Use a constant value.
TEST_F(MotiveTests, MatrixTranslateXConstValue) {
  std::vector<MatrixOperationInit> ops;
  ops.emplace_back(0, motive::kTranslateX, 2.0f);
  TestMatrixMotivator(MatrixInit(ops), &engine_);
}

// Test the matrix operation kRotateAboutX.
TEST_F(MotiveTests, MatrixRotateAboutX) {
  std::vector<MatrixOperationInit> ops;
  ops.emplace_back(0, motive::kRotateAboutX, spline_angle_init_, kHalfPi);
  TestMatrixMotivator(MatrixInit(ops), &engine_);
}

// Test the matrix operation kRotateAboutY.
TEST_F(MotiveTests, MatrixRotateAboutY) {
  std::vector<MatrixOperationInit> ops;
  ops.emplace_back(0, motive::kRotateAboutY, spline_angle_init_,
                   kHalfPi / 3.0f);
  TestMatrixMotivator(MatrixInit(ops), &engine_);
}

// Test the matrix operation kRotateAboutZ.
TEST_F(MotiveTests, MatrixRotateAboutZ) {
  std::vector<MatrixOperationInit> ops;
  ops.emplace_back(0, motive::kRotateAboutZ, spline_angle_init_,
                   -kHalfPi / 1.2f);
  TestMatrixMotivator(MatrixInit(ops), &engine_);
}

// Test the matrix operation kScaleX.
TEST_F(MotiveTests, MatrixScaleX) {
  std::vector<MatrixOperationInit> ops;
  ops.emplace_back(0, motive::kScaleX, spline_scalar_init, -3.0f);
  TestMatrixMotivator(MatrixInit(ops), &engine_);
}

// Test the series of matrix operations for translating XYZ.
TEST_F(MotiveTests, MatrixTranslateXYZ) {
  std::vector<MatrixOperationInit> ops;
  ops.emplace_back(0, motive::kTranslateX, spline_scalar_init, 2.0f);
  ops.emplace_back(0, motive::kTranslateY, spline_scalar_init, -3.0f);
  ops.emplace_back(0, motive::kTranslateZ, spline_scalar_init, 0.5f);
  TestMatrixMotivator(MatrixInit(ops), &engine_);
}

// Test the series of matrix operations for rotating about X, Y, and Z,
// in turn.
TEST_F(MotiveTests, MatrixRotateAboutXYZ) {
  std::vector<MatrixOperationInit> ops;
  ops.emplace_back(0, motive::kRotateAboutX, spline_angle_init_,
                   -kHalfPi / 2.0f);
  ops.emplace_back(0, motive::kRotateAboutY, spline_angle_init_,
                   kHalfPi / 3.0f);
  ops.emplace_back(0, motive::kRotateAboutZ, spline_angle_init_,
                   kHalfPi / 5.0f);
  TestMatrixMotivator(MatrixInit(ops), &engine_);
}

// Test the series of matrix operations for scaling XYZ non-uniformly.
TEST_F(MotiveTests, MatrixScaleXYZ) {
  std::vector<MatrixOperationInit> ops;
  ops.emplace_back(0, motive::kScaleX, spline_scalar_init, -3.0f);
  ops.emplace_back(0, motive::kScaleY, spline_scalar_init, 2.2f);
  ops.emplace_back(0, motive::kScaleZ, spline_scalar_init, 1.01f);
  TestMatrixMotivator(MatrixInit(ops), &engine_);
}

// Test the matrix operation kScaleUniformly.
TEST_F(MotiveTests, MatrixScaleUniformly) {
  std::vector<MatrixOperationInit> ops;
  ops.emplace_back(0, motive::kScaleUniformly, spline_scalar_init, 10.1f);
  TestMatrixMotivator(MatrixInit(ops), &engine_);
}

// Test the series of matrix operations for translating and rotating.
TEST_F(MotiveTests, MatrixTranslateRotateTranslateBack) {
  std::vector<MatrixOperationInit> ops;
  ops.emplace_back(0, motive::kTranslateY, spline_scalar_init, 1.0f);
  ops.emplace_back(0, motive::kRotateAboutX, spline_angle_init_, kHalfPi);
  ops.emplace_back(0, motive::kTranslateY, spline_scalar_init, -1.0f);
  TestMatrixMotivator(MatrixInit(ops), &engine_);
}

// Test the series of matrix operations for translating, rotating, and scaling.
TEST_F(MotiveTests, MatrixTranslateRotateScale) {
  std::vector<MatrixOperationInit> ops;
  ops.emplace_back(0, motive::kTranslateY, spline_scalar_init, 1.0f);
  ops.emplace_back(0, motive::kRotateAboutX, spline_angle_init_, kHalfPi);
  ops.emplace_back(0, motive::kScaleZ, spline_scalar_init, -1.4f);
  TestMatrixMotivator(MatrixInit(ops), &engine_);
}

// Test a complex the series of matrix operations.
TEST_F(MotiveTests, MatrixTranslateRotateScaleGoneWild) {
  std::vector<MatrixOperationInit> ops;
  ops.emplace_back(0, motive::kTranslateY, spline_scalar_init, 1.0f);
  ops.emplace_back(0, motive::kTranslateX, spline_scalar_init, -1.6f);
  ops.emplace_back(0, motive::kRotateAboutX, spline_angle_init_,
                   kHalfPi * 0.1f);
  ops.emplace_back(0, motive::kRotateAboutY, spline_angle_init_,
                   kHalfPi * 0.33f);
  ops.emplace_back(0, motive::kScaleZ, spline_scalar_init, -1.4f);
  ops.emplace_back(0, motive::kRotateAboutY, spline_angle_init_,
                   -kHalfPi * 0.33f);
  ops.emplace_back(0, motive::kTranslateX, spline_scalar_init, -1.2f);
  ops.emplace_back(0, motive::kTranslateY, spline_scalar_init, -1.5f);
  ops.emplace_back(0, motive::kTranslateZ, spline_scalar_init, -2.2f);
  ops.emplace_back(0, motive::kRotateAboutZ, spline_angle_init_,
                   -kHalfPi * 0.5f);
  ops.emplace_back(0, motive::kScaleX, spline_scalar_init, 2.0f);
  ops.emplace_back(0, motive::kScaleY, spline_scalar_init, 4.1f);
  TestMatrixMotivator(MatrixInit(ops), &engine_);
}

// Return the matrix equivalent to the Sqt operations from 'sqt_init'.
static mat4 CreateMatrixForSqt(const SqtInit& sqt_init) {
  const std::vector<MatrixOperationInit>& ops = sqt_init.ops();

  mathfu::vec3 translation = motive::DefaultOpsTranslation();
  mathfu::quat rotation = motive::DefaultOpsQuaternion();
  mathfu::vec3 scale = motive::DefaultOpsScale();

  for (size_t i = 0; i < ops.size(); ++i) {
    const MatrixOperationInit& op = ops[i];
    if (motive::TranslateOp(op.type)) {
      translation[op.type - motive::kTranslateX] = op.initial_value;
    } else if (motive::QuaternionOp(op.type)) {
      rotation[op.type - motive::kQuaternionW] = op.initial_value;
    } else if (motive::ScaleOp(op.type)) {
      scale[op.type - motive::kScaleX] = op.initial_value;
    }
  }
  return mathfu::mat4::Transform(translation, rotation.Normalized().ToMatrix(),
                                 scale);
}

static void TestSqtMotivator(const SqtInit& sqt_init, MotiveEngine* engine) {
  MatrixMotivator4f matrix_motivator(sqt_init, engine);
  const mat4 check_matrix = CreateMatrixForSqt(sqt_init);

  const mat4 init_matrix = matrix_motivator.Value();
  ExpectMatricesEqual(init_matrix, check_matrix, kMatrixEpsilon);

  engine->AdvanceFrame(kTimePerFrame);
  const mat4 motive_matrix = matrix_motivator.Value();
  ExpectMatricesEqual(motive_matrix, check_matrix, kMatrixEpsilon);

  // Output matrices for debugging.
  PrintMatrix("motivator", motive_matrix);
  PrintMatrix("check", check_matrix);
}

// Test the translation portion of the Sqt.
TEST_F(MotiveTests, SqtTranslate) {
  std::vector<MatrixOperationInit> ops;
  ops.emplace_back(0, motive::kTranslateX, spline_scalar_init, 2.0f);
  ops.emplace_back(0, motive::kTranslateY, spline_scalar_init, -3.0f);
  ops.emplace_back(0, motive::kTranslateZ, spline_scalar_init, -2.0f);
  TestSqtMotivator(SqtInit(ops), &engine_);
}

// Test the quaternion portion of the Sqt.
TEST_F(MotiveTests, SqtQuaternion) {
  std::vector<MatrixOperationInit> ops;
  ops.emplace_back(0, motive::kQuaternionW, spline_scalar_init, 0.2f);
  ops.emplace_back(0, motive::kQuaternionX, spline_scalar_init, 0.4f);
  ops.emplace_back(0, motive::kQuaternionY, spline_scalar_init, 0.6f);
  ops.emplace_back(0, motive::kQuaternionZ, spline_scalar_init, 0.8f);
  TestSqtMotivator(SqtInit(ops), &engine_);
}

// Test the scale portion of the Sqt.
TEST_F(MotiveTests, SqtScale) {
  std::vector<MatrixOperationInit> ops;
  ops.emplace_back(0, motive::kScaleX, spline_scalar_init, -3.0f);
  ops.emplace_back(0, motive::kScaleY, spline_scalar_init, -2.0f);
  ops.emplace_back(0, motive::kScaleZ, spline_scalar_init, 3.0f);
  TestSqtMotivator(SqtInit(ops), &engine_);
}
// Test the quaternion portion of the Sqt.
TEST_F(MotiveTests, SqtTranslateQuaternionScale) {
  std::vector<MatrixOperationInit> ops;
  ops.emplace_back(0, motive::kTranslateX, spline_scalar_init, 2.0f);
  ops.emplace_back(0, motive::kTranslateY, spline_scalar_init, -3.0f);
  ops.emplace_back(0, motive::kTranslateZ, spline_scalar_init, -2.0f);
  ops.emplace_back(0, motive::kQuaternionW, spline_scalar_init, 0.2f);
  ops.emplace_back(0, motive::kQuaternionX, spline_scalar_init, 0.4f);
  ops.emplace_back(0, motive::kQuaternionY, spline_scalar_init, 0.6f);
  ops.emplace_back(0, motive::kQuaternionZ, spline_scalar_init, 0.8f);
  ops.emplace_back(0, motive::kScaleX, spline_scalar_init, -3.0f);
  ops.emplace_back(0, motive::kScaleY, spline_scalar_init, -2.0f);
  ops.emplace_back(0, motive::kScaleZ, spline_scalar_init, 3.0f);
  TestSqtMotivator(SqtInit(ops), &engine_);
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
  MotivatorT angle(t.spline_angle_init(), &t.engine());
  angle.SetSplines(splines,
                   SplinePlayback(static_cast<float>(kStartTime), true));

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
  typedef typename MotivatorT::Vec Vec;
  static const MotiveTime kStartTime = 250;
  static const MotiveTime kDeltaTime = 500;

  const motive::CompactSpline* splines =
      t.simple_splines(MotivatorT::kDimensions);
  const MotiveTime end_time = static_cast<MotiveTime>(splines[0].EndX());

  // Create a motivator that plays `spline` from kStartTime.
  MotivatorT angle(t.spline_angle_init(), &t.engine());
  angle.SetSplines(splines, SplinePlayback(static_cast<float>(kStartTime)));

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
  MotivatorT angle(t.spline_angle_init(), &t.engine());

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

// Test the MotivatorVector::Splines() function.
template <class MotivatorT>
void Splines(MotiveTests& t) {
  static const MotiveTime kStartTime = 250;
  static const MotiveTime kDeltaTime = 500;

  const motive::CompactSpline* splines =
      t.simple_splines(MotivatorT::kDimensions);
  const MotiveTime end_time = static_cast<MotiveTime>(splines[0].EndX());

  // Two updates of kDeltaTime should wrap past end_time.
  assert(kStartTime + 2 * kDeltaTime > end_time);

  // Create a motivator that plays `spline` from kStartTime, and repeats
  // at the start when it reaches the end.
  MotivatorT angle(t.spline_angle_init(), &t.engine());
  angle.SetSplines(splines,
                   SplinePlayback(static_cast<float>(kStartTime), true));

  // Gather the currently active splines and compare them to the ones we set.
  const motive::CompactSpline* fetched_splines[MotivatorT::kDimensions];
  angle.Splines(fetched_splines);

  const motive::CompactSpline* s = splines;
  for (int i = 0; i < MotivatorT::kDimensions; ++i) {
    EXPECT_EQ(s, fetched_splines[i]);
    s = s->Next();
  }

  // Since the playback rate is 1, we expect the spline time to advance with
  // the delta time.
  t.engine().AdvanceFrame(kDeltaTime);
  EXPECT_EQ(angle.SplineTime(), kStartTime + kDeltaTime);

  // Should still have the same splines playing.
  s = splines;
  for (int i = 0; i < MotivatorT::kDimensions; ++i) {
    EXPECT_EQ(s, fetched_splines[i]);
    s = s->Next();
  }
}
TEST_ALL_VECTOR_MOTIVATORS_F(Splines)

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
