// Copyright 2019 Google Inc. All rights reserved.
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
#include "motive/matrix_anim.h"
#include "motive/matrix_op.h"
#include "motive/util/keyframe_converter.h"

using motive::KeyframeData;
using motive::MatrixAnim;
using motive::MatrixOperationInit;
using motive::MatrixOperationType;
using motive::MatrixOpId;

static const float kEpsilon = 0.001f;

class UtilTests : public ::testing::Test {
 protected:
  virtual void SetUp() {}
  virtual void TearDown() {}
};

// Test that three consecutive constants are added to the animation.
TEST_F(UtilTests, AddVector3ConstantsSimple) {
  MatrixAnim anim;
  const float values[3] = {1.f, 2.f, 3.f};
  const MatrixOperationType base_type = motive::kTranslateX;
  const MatrixOpId base_id = 1;
  motive::AddVector3Constants(&anim, base_type, base_id, values);

  const std::vector<MatrixOperationInit>& ops = anim.ops();
  EXPECT_EQ(ops.size(), 3);

  for (int i = 0; i < 3; ++i) {
    const MatrixOperationInit& op = ops[i];
    EXPECT_EQ(op.type, base_type + i);
    EXPECT_EQ(op.id, base_id + i);
    EXPECT_EQ(op.init, nullptr);
    EXPECT_EQ(op.union_type,
              MatrixOperationInit::UnionType::kUnionInitialValue);
    EXPECT_EQ(op.initial_value, values[i]);
  }
}

// Test that default translation values are not added to the animation.
TEST_F(UtilTests, AddVector3ConstantsDefaultTranslationValues) {
  MatrixAnim anim;
  // The only non-default is Y = 1.
  const float values[3] = {0.f, 1.f, 0.f};
  motive::AddVector3Constants(&anim, motive::kTranslateX, 1, values);

  const std::vector<MatrixOperationInit>& ops = anim.ops();
  EXPECT_EQ(ops.size(), 1);

  const MatrixOperationInit& op = ops[0];
  EXPECT_EQ(op.type, motive::kTranslateY);
  EXPECT_EQ(op.id, 2);
  EXPECT_EQ(op.init, nullptr);
  EXPECT_EQ(op.union_type, MatrixOperationInit::UnionType::kUnionInitialValue);
  EXPECT_EQ(op.initial_value, 1.f);
}

// Test that default scale values are not added to the animation.
TEST_F(UtilTests, AddVector3ConstantsDefaultScaleValues) {
  MatrixAnim anim;
  // The only non-default is Y = 0.
  const float values[3] = {1.f, 0.f, 1.f};
  motive::AddVector3Constants(&anim, motive::kScaleX, 1, values);

  const std::vector<MatrixOperationInit>& ops = anim.ops();
  EXPECT_EQ(ops.size(), 1);

  const MatrixOperationInit& op = ops[0];
  EXPECT_EQ(op.type, motive::kScaleY);
  EXPECT_EQ(op.id, 2);
  EXPECT_EQ(op.init, nullptr);
  EXPECT_EQ(op.union_type, MatrixOperationInit::UnionType::kUnionInitialValue);
  EXPECT_EQ(op.initial_value, 0.f);
}

// Test that four consecutive quaternion constants in order WXYZ are added to
// the animation.
TEST_F(UtilTests, AddQuaternionConstantsOrderWXYZ) {
  MatrixAnim anim;
  const float values[4] = {0.2f, 0.4f, 0.6f, 0.8f};
  const MatrixOpId base_id = 1;
  motive::AddQuaternionConstants(&anim, base_id, values, motive::kOrderWXYZ);

  const std::vector<MatrixOperationInit>& ops = anim.ops();
  EXPECT_EQ(ops.size(), 4);

  // Operations are added in WXYZ order.
  for (int i = 0; i < 4; ++i) {
    const MatrixOperationInit& op = ops[i];
    EXPECT_EQ(op.type, motive::kQuaternionW + i);
    EXPECT_EQ(op.id, base_id + i);
    EXPECT_EQ(op.init, nullptr);
    EXPECT_EQ(op.union_type,
              MatrixOperationInit::UnionType::kUnionInitialValue);
    EXPECT_EQ(op.initial_value, values[i]);
  }
}

// Test that four consecutive quaternion constants in order XYZW are added to
// the animation.
TEST_F(UtilTests, AddQuaternionConstantsOrderXYZW) {
  MatrixAnim anim;
  const float values[4] = {0.2f, 0.4f, 0.6f, 0.8f};
  const MatrixOpId base_id = 1;
  motive::AddQuaternionConstants(&anim, base_id, values, motive::kOrderXYZW);

  const std::vector<MatrixOperationInit>& ops = anim.ops();
  EXPECT_EQ(ops.size(), 4);

  // Operations are still added in WXYZ order, so use a different expected set
  // of values.
  const float expected_values[4] = {0.8f, 0.2f, 0.4f, 0.6f};
  for (int i = 0; i < 4; ++i) {
    const MatrixOperationInit& op = ops[i];
    EXPECT_EQ(op.type, motive::kQuaternionW + i);
    EXPECT_EQ(op.id, base_id + i);
    EXPECT_EQ(op.init, nullptr);
    EXPECT_EQ(op.union_type,
              MatrixOperationInit::UnionType::kUnionInitialValue);
    EXPECT_EQ(op.initial_value, expected_values[i]);
  }
}

// Test that default values are not added to the animation.
TEST_F(UtilTests, AddQuaternionConstantsOrderDefaultValues) {
  MatrixAnim anim;
  // The only non-default is Z = 1.
  const float values[4] = {1.f, 0.f, 0.f, 1.f};
  motive::AddQuaternionConstants(&anim, 1, values, motive::kOrderWXYZ);

  const std::vector<MatrixOperationInit>& ops = anim.ops();
  EXPECT_EQ(ops.size(), 1);

  const MatrixOperationInit& op = ops[0];
  EXPECT_EQ(op.type, motive::kQuaternionZ);
  EXPECT_EQ(op.id, 4);
  EXPECT_EQ(op.init, nullptr);
  EXPECT_EQ(op.union_type, MatrixOperationInit::UnionType::kUnionInitialValue);
  EXPECT_EQ(op.initial_value, 1.f);
}

// Test that three animation curves are added.
TEST_F(UtilTests, AddVector3CurvesSimple) {
  MatrixAnim anim;
  MatrixAnim::Spline* splines = anim.Construct(3);
  const MatrixOperationType base_type = motive::kTranslateX;
  const MatrixOpId base_id = 1;

  const float delta = 1.f;
  const size_t num_times = 3;
  const float times[num_times] = {delta, 2.f * delta, 3.f * delta};
  const float values[9] = {
      0.f, 0.f, 0.f, 1.f, 1.f, 1.f, 2.f, 2.f, 2.f,
  };

  KeyframeData data;
  data.times = times;
  data.values = values;
  data.count = num_times;
  data.interpolation_type = motive::kLinear;
  data.ms_per_time_unit = 1.f;

  motive::AddVector3Curves(&anim, splines, base_type, base_id, data);

  const std::vector<MatrixOperationInit>& ops = anim.ops();
  EXPECT_EQ(ops.size(), 3);

  for (int i = 0; i < 3; ++i) {
    const MatrixOperationInit& op = ops[i];
    EXPECT_EQ(op.type, base_type + i);
    EXPECT_EQ(op.id, base_id + i);
    EXPECT_NE(op.init, nullptr);
    EXPECT_EQ(op.union_type, MatrixOperationInit::UnionType::kUnionSpline);
    EXPECT_NE(op.spline, nullptr);

    // Because the utility may add more than one node per time value, evaluate
    // the spline at each time to ensure correctness.
    float ys[num_times];
    op.spline->Ys(times[0], delta, num_times, ys);
    for (int j = 0; j < num_times; ++j) {
      EXPECT_TRUE(motive::NearlyEqual(ys[j], values[j * 3 + i], kEpsilon));
    }
  }
}

// Test that four quaternion animation curves are added.
TEST_F(UtilTests, AddQuaternionCurvesSimple) {
  MatrixAnim anim;
  MatrixAnim::Spline* splines = anim.Construct(4);
  const MatrixOpId base_id = 1;

  const float delta = 1.f;
  const size_t num_times = 3;
  const float times[num_times] = {delta, 2.f * delta, 3.f * delta};
  const float values[12] = {
      0.f, 0.f, 0.f, 0.f, 1.f, 1.f, 1.f, 1.f, 2.f, 2.f, 2.f, 2.f,
  };

  KeyframeData data;
  data.times = times;
  data.values = values;
  data.count = num_times;
  data.interpolation_type = motive::kLinear;
  data.ms_per_time_unit = 1.f;

  motive::AddQuaternionCurves(&anim, splines, base_id, motive::kOrderWXYZ,
                              data);

  const std::vector<MatrixOperationInit>& ops = anim.ops();
  EXPECT_EQ(ops.size(), 4);

  for (int i = 0; i < 4; ++i) {
    const MatrixOperationInit& op = ops[i];
    EXPECT_EQ(op.type, motive::kQuaternionW + i);
    EXPECT_EQ(op.id, base_id + i);
    EXPECT_NE(op.init, nullptr);
    EXPECT_EQ(op.union_type, MatrixOperationInit::UnionType::kUnionSpline);
    EXPECT_NE(op.spline, nullptr);

    // Because the utility may add more than one node per time value, evaluate
    // the spline at each time to ensure correctness.
    float ys[num_times];
    op.spline->Ys(times[0], delta, num_times, ys);
    for (int j = 0; j < num_times; ++j) {
      EXPECT_TRUE(motive::NearlyEqual(ys[j], values[j * 4 + i], kEpsilon));
    }
  }
}

// Test that four quaternion animation curves are added. Ensure that the middle
// value (-1,-1,-1,-1) is flipped to (1,1,1,1).
TEST_F(UtilTests, AddQuaternionCurvesFlip) {
  MatrixAnim anim;
  MatrixAnim::Spline* splines = anim.Construct(4);
  const MatrixOpId base_id = 1;

  const float delta = 1.f;
  const size_t num_times = 3;
  const float times[num_times] = {delta, 2.f * delta, 3.f * delta};
  const float values[12] = {
      1.f, 1.f, 1.f, 1.f, -1.f, -1.f, -1.f, -1.f, 1.f, 1.f, 1.f, 1.f,
  };

  KeyframeData data;
  data.times = times;
  data.values = values;
  data.count = num_times;
  data.interpolation_type = motive::kLinear;
  data.ms_per_time_unit = 1.f;

  motive::AddQuaternionCurves(&anim, splines, base_id, motive::kOrderWXYZ,
                              data);

  const std::vector<MatrixOperationInit>& ops = anim.ops();
  EXPECT_EQ(ops.size(), 4);

  for (int i = 0; i < 4; ++i) {
    const MatrixOperationInit& op = ops[i];
    EXPECT_EQ(op.type, motive::kQuaternionW + i);
    EXPECT_EQ(op.id, base_id + i);
    EXPECT_NE(op.init, nullptr);
    EXPECT_EQ(op.union_type, MatrixOperationInit::UnionType::kUnionSpline);
    EXPECT_NE(op.spline, nullptr);

    // Because the utility may add more than one node per time value, evaluate
    // the spline at each time to ensure correctness.
    float ys[num_times];
    op.spline->Ys(times[0], delta, num_times, ys);
    for (int j = 0; j < num_times; ++j) {
      fprintf(stdout, "%f\n", ys[j]);
      // The -1.f's get flipped to 1.f since q and -q are the same.
      EXPECT_TRUE(motive::NearlyEqual(ys[j], 1.f, kEpsilon));
    }
  }
}

// Test that animation curves respect linear interpolation.
TEST_F(UtilTests, LinearInterpolation) {
  MatrixAnim anim;
  MatrixAnim::Spline* splines = anim.Construct(3);

  const size_t num_times = 3;
  const float times[num_times] = {0.f, 1.f, 2.f};
  const float values[9] = {
      0.f, 0.f, 0.f, 1.f, 1.f, 1.f, 0.f, 0.f, 0.f,
  };

  KeyframeData data;
  data.times = times;
  data.values = values;
  data.count = num_times;
  data.interpolation_type = motive::kLinear;
  data.ms_per_time_unit = 1.f;

  motive::AddVector3Curves(&anim, splines, motive::kTranslateX, 1, data);

  const std::vector<MatrixOperationInit>& ops = anim.ops();
  EXPECT_EQ(ops.size(), 3);

  for (int i = 0; i < 3; ++i) {
    const MatrixOperationInit& op = ops[i];

    // Evaluate the curve at a series of times and ensure the values are
    // correct. The first 10 times are linear from 0 to 1. The second 10 are
    // linear from 1 to 0.
    float ys[20];
    op.spline->Ys(times[0], 0.1f, 20, ys);
    for (int j = 0; j < 10; ++j) {
      EXPECT_TRUE(motive::NearlyEqual(ys[j], 0.1f * j, kEpsilon));
    }
    for (int j = 0; j < 10; ++j) {
      EXPECT_TRUE(motive::NearlyEqual(ys[j + 10], 1.f - 0.1f * j, kEpsilon));
    }
  }
}

// Test that animation curves respect step interpolation.
TEST_F(UtilTests, StepInterpolation) {
  MatrixAnim anim;
  MatrixAnim::Spline* splines = anim.Construct(3);

  const size_t num_times = 3;
  const float times[num_times] = {0.f, 1.f, 2.f};
  const float values[9] = {
      0.f, 0.f, 0.f, 1.f, 1.f, 1.f, 0.f, 0.f, 0.f,
  };

  KeyframeData data;
  data.times = times;
  data.values = values;
  data.count = num_times;
  data.interpolation_type = motive::kStep;
  data.ms_per_time_unit = 1.f;

  motive::AddVector3Curves(&anim, splines, motive::kTranslateX, 1, data);

  const std::vector<MatrixOperationInit>& ops = anim.ops();
  EXPECT_EQ(ops.size(), 3);

  for (int i = 0; i < 3; ++i) {
    const MatrixOperationInit& op = ops[i];

    // Evaluate the curve at a series of times and ensure the values are
    // correct. The first 10 times are 0. The second 10 are 1.
    float ys[20];
    // Start just past the beginning of the spline to avoid rounding errors.
    op.spline->Ys(times[0] + kEpsilon, 0.1f, 20, ys);
    for (int j = 0; j < 10; ++j) {
      EXPECT_TRUE(motive::NearlyEqual(ys[j], 0.f, kEpsilon));
    }
    for (int j = 10; j < 20; ++j) {
      EXPECT_TRUE(motive::NearlyEqual(ys[j], 1.f, kEpsilon));
    }
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
