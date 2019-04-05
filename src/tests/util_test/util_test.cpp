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
#include "third_party/motive/include/motive/util/keyframe_converter.h"

using motive::CompactSpline;
using motive::CompactSplineIndex;
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

// Test that callers can determine how many bytes a set of curves needs for each
// interpolation type.
TEST_F(UtilTests, GetRequiredBufferSize) {
  const size_t zero_node_size = CompactSpline::Size(0);
  EXPECT_EQ(GetRequiredBufferSize(0, 1, motive::kLinear), zero_node_size);
  EXPECT_EQ(GetRequiredBufferSize(0, 2, motive::kStep), zero_node_size * 2);
  EXPECT_EQ(GetRequiredBufferSize(0, 3, motive::kCubicSpline),
            zero_node_size * 3);

  const size_t six_node_size = CompactSpline::Size(6);
  EXPECT_EQ(GetRequiredBufferSize(3, 1, motive::kLinear), six_node_size);
  EXPECT_EQ(GetRequiredBufferSize(3, 2, motive::kStep), six_node_size * 2);
  EXPECT_EQ(GetRequiredBufferSize(3, 3, motive::kCubicSpline),
            six_node_size * 3);

  // Regardless of interpolation type, kMaxSplineIndex + 1 nodes gives a zero
  // buffer size.
  const CompactSplineIndex more_than_max = motive::kMaxSplineIndex + 1;
  EXPECT_EQ(GetRequiredBufferSize(more_than_max, 1, motive::kLinear), 0);
  EXPECT_EQ(GetRequiredBufferSize(more_than_max, 2, motive::kStep), 0);
  EXPECT_EQ(GetRequiredBufferSize(more_than_max, 3, motive::kCubicSpline), 0);
}

// Test that a pre-allocated buffer can be populated with CompactSplines that
// represent a set of keyframe data.
TEST_F(UtilTests, AddArrayCurves) {
  // Configure the animation data to have 3 channels and 3 keyframes.
  const float delta = 1.f;
  const size_t num_channels = 3;
  const size_t num_times = 3;
  const float times[num_times] = {delta, 2.f * delta, 3.f * delta};
  const float values[num_channels * num_times] = {
      0.f, 0.f, 0.f, 1.f, 1.f, 1.f, 2.f, 2.f, 2.f,
  };

  KeyframeData data;
  data.times = times;
  data.values = values;
  data.count = num_times;
  data.interpolation_type = motive::kLinear;
  data.ms_per_time_unit = 1.f;

  // Allocate a buffer large enough to hold 3 CompactSplines with the required
  // number of nodes.
  const size_t required_bytes = motive::GetRequiredBufferSize(
      num_times, num_channels, data.interpolation_type);
  std::vector<uint8_t> buffer(required_bytes);

  // Populate the buffer with 3 CompactSpline curves.
  const size_t bytes_used =
      motive::AddArrayCurves(buffer.data(), data, num_channels);
  EXPECT_EQ(bytes_used, required_bytes);

  const CompactSpline* splines =
      reinterpret_cast<const CompactSpline*>(buffer.data());

  for (int i = 0; i < num_channels; ++i) {
    // Because the utility may add more than one node per time value, evaluate
    // the spline at each time to ensure correctness.
    float ys[num_times];
    splines->NextAtIdx(i)->Ys(times[0], delta, num_times, ys);
    for (int j = 0; j < num_times; ++j) {
      EXPECT_TRUE(
          motive::NearlyEqual(ys[j], values[j * num_channels + i], kEpsilon));
    }
  }
}

// Test that a pre-allocated buffer can be populated with CompactSplines that
// represent a set of quaternion keyframe data, including re-ordering the
// quaternion components.
TEST_F(UtilTests, AddQuaternionCurvesReorder) {
  // Configure the animation data to have 4 channels and 3 keyframes.
  const float delta = 1.f;
  const size_t num_channels = 4;
  const size_t num_times = 3;
  const float times[num_times] = {delta, 2.f * delta, 3.f * delta};
  // Have the W component be different to ensure it is switched.
  const float values[num_channels * num_times] = {
      0.f, 0.f, 0.f, 3.f, 1.f, 1.f, 1.f, 4.f, 2.f, 2.f, 2.f, 5.f,
  };

  KeyframeData data;
  data.times = times;
  data.values = values;
  data.count = num_times;
  data.interpolation_type = motive::kLinear;
  data.ms_per_time_unit = 1.f;

  // Allocate a buffer large enough to hold 3 CompactSplines with the required
  // number of nodes.
  const size_t required_bytes = motive::GetRequiredBufferSize(
      num_times, num_channels, data.interpolation_type);
  std::vector<uint8_t> buffer(required_bytes);

  // Populate the buffer with the CompactSpline curves.
  const size_t bytes_used =
      motive::AddQuaternionCurves(buffer.data(), data, motive::kOrderXYZW);
  EXPECT_EQ(bytes_used, required_bytes);

  const CompactSpline* splines =
      reinterpret_cast<const CompactSpline*>(buffer.data());

  // Since AddQuaternionCurves() always adds in order WXYZ, we expect shifted
  // values.
  const float expected[num_channels * num_times] = {
      3.f, 0.f, 0.f, 0.f, 4.f, 1.f, 1.f, 1.f, 5.f, 2.f, 2.f, 2.f,
  };

  for (int i = 0; i < num_channels; ++i) {
    // Because the utility may add more than one node per time value, evaluate
    // the spline at each time to ensure correctness.
    float ys[num_times];
    splines->NextAtIdx(i)->Ys(times[0], delta, num_times, ys);
    for (int j = 0; j < num_times; ++j) {
      EXPECT_TRUE(
          motive::NearlyEqual(ys[j], expected[j * num_channels + i], kEpsilon));
    }
  }
}

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

// Test that animation curves respect cubicspline interpolation.
TEST_F(UtilTests, CubicSplineInterpolation) {
  MatrixAnim anim;
  MatrixAnim::Spline* splines = anim.Construct(3);

  const size_t num_times = 3;
  const float times[num_times] = {0.f, 1.f, 2.f};

  // Make the cubicsplines effectively linear.
  const float values[27] = {
      // (1,1,1) with left-tangent (0,0,0), right-tangent (1,1,1)
      0.f,  0.f,  0.f,  1.f, 1.f, 1.f, 1.f,  1.f,  1.f,
      // (2,2,2) with left-tangent (1,1,1), right-tangent (-1,-1,-1)
      1.f,  1.f,  1.f,  2.f, 2.f, 2.f, -1.f, -1.f, -1.f,
      // (1,1,1) with left-tangent (-1,-1,-1), right-tangent (0,0,0)
      -1.f, -1.f, -1.f, 1.f, 1.f, 1.f, 1.f,  1.f,  1.f,
  };

  KeyframeData data;
  data.times = times;
  data.values = values;
  data.count = num_times;
  data.interpolation_type = motive::kCubicSpline;
  data.ms_per_time_unit = 1.f;

  motive::AddVector3Curves(&anim, splines, motive::kTranslateX, 1, data);

  const std::vector<MatrixOperationInit>& ops = anim.ops();
  EXPECT_EQ(ops.size(), 3);

  for (int i = 0; i < 3; ++i) {
    const MatrixOperationInit& op = ops[i];

    // Evaluate the curve at a series of times and ensure the values are
    // correct. The first 10 times are linear from 1 to 2. The second 10 are
    // linear from 2 to 1.
    float ys[20];
    op.spline->Ys(times[0], 0.1f, 20, ys);
    for (int j = 0; j < 10; ++j) {
      EXPECT_TRUE(motive::NearlyEqual(ys[j], 1.f + 0.1f * j, kEpsilon));
    }
    for (int j = 0; j < 10; ++j) {
      EXPECT_TRUE(motive::NearlyEqual(ys[j + 10], 2.f - 0.1f * j, kEpsilon));
    }
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
