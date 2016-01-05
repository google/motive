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
#include "motive/common.h"
#include "motive/math/angle.h"
#include "motive/math/bulk_spline_evaluator.h"
#include "motive/math/compact_spline.h"

using motive::QuadraticCurve;
using motive::CubicCurve;
using motive::CubicInit;
using motive::Range;
using motive::CompactSpline;
using motive::CompactSplineIndex;
using motive::BulkSplineEvaluator;
using motive::Angle;
using motive::kPi;
using mathfu::vec2;
using mathfu::vec2i;
using mathfu::vec3;
using mathfu::vec3_packed;

// Print the curves in a format that can be cut-and-paste into a spreadsheet.
// Working in a spreadsheet is nice because of the graphing features.
#define PRINT_SPLINES_AS_CSV 0

// Draw an ASCII graph of the curves. Helpful for a quick visualization, though
// not very high fidelity, obviously.
#define PRINT_SPLINES_AS_ASCII_GRAPHS 1

struct GraphDerivatives {
  float first;
  float second;
  float third;
  GraphDerivatives() : first(0.0f), second(0.0f), third(0.0f) {}
  GraphDerivatives(float first, float second, float third)
      : first(first), second(second), third(third) {}
};

struct GraphData {
  std::vector<vec2> points;
  std::vector<GraphDerivatives> derivatives;
};

static const int kNumCheckPoints = motive::kDefaultGraphWidth;
static const vec2i kGraphSize(kNumCheckPoints, motive::kDefaultGraphHeight);
static const float kFixedPointEpsilon = 0.02f;
static const float kDerivativePrecision = 0.01f;
static const float kSecondDerivativePrecision = 0.26f;
static const float kThirdDerivativePrecision = 6.0f;
static const float kNodeYPrecision = 0.0001f;
static const float kXGranularityScale = 0.01f;
static const Range kAngleRange(-kPi, kPi);

// Use a ridiculous index that will never hit when doing a search.
// We use this to test the binary search algorithm, not the cache.
static const CompactSplineIndex kRidiculousSplineIndex = 10000;

static const CubicInit kSimpleSplines[] = {
    //    start_y         end_y        width_x
    //      start_derivative  end_derivative
    CubicInit(0.0f, 1.0f, 0.1f, 0.0f, 1.0f),
    CubicInit(1.0f, -8.0f, 0.0f, 0.0f, 1.0f),
    CubicInit(1.0f, -8.0f, -1.0f, 0.0f, 1.0f),
};
static const int kNumSimpleSplines =
    static_cast<int>(MOTIVE_ARRAY_SIZE(kSimpleSplines));

static const CubicInit CubicInitMirrorY(const CubicInit& init) {
  return CubicInit(-init.start_y, -init.start_derivative, -init.end_y,
                   -init.end_derivative, init.width_x);
}

static const CubicInit CubicInitScaleX(const CubicInit& init, float scale) {
  return CubicInit(init.start_y, init.start_derivative / scale, init.end_y,
                   init.end_derivative / scale, init.width_x * scale);
}

static Range CubicInitYRange(const CubicInit& init, float buffer_percent) {
  return motive::CreateValidRange(init.start_y, init.end_y)
      .Lengthen(buffer_percent);
}

static void InitializeSpline(const CubicInit& init, CompactSpline* spline) {
  const Range y_range = CubicInitYRange(init, 0.1f);
  spline->Init(y_range, init.width_x * kXGranularityScale, 3);
  spline->AddNode(0.0f, init.start_y, init.start_derivative);
  spline->AddNode(init.width_x, init.end_y, init.end_derivative);
}

static void ExecuteInterpolator(BulkSplineEvaluator& interpolator,
                                int num_points, GraphData* d) {
  const float y_precision =
      interpolator.SourceSpline(0)->RangeY().Length() * kFixedPointEpsilon;

  const Range range_x = interpolator.SourceSpline(0)->RangeX();
  const float delta_x = range_x.Length() / (num_points - 1);

  for (int i = 0; i < num_points; ++i) {
    const CubicCurve& c = interpolator.Cubic(0);
    const float x = interpolator.CubicX(0);

    EXPECT_NEAR(c.Evaluate(x), interpolator.Y(0), y_precision);
    EXPECT_NEAR(c.Derivative(x), interpolator.Derivative(0),
                kDerivativePrecision);

    const vec2 point(interpolator.X(0), interpolator.Y(0));
    const GraphDerivatives derivatives(interpolator.Derivative(0),
                                       c.SecondDerivative(x),
                                       c.ThirdDerivative(x));
    d->points.push_back(point);
    d->derivatives.push_back(derivatives);

    interpolator.AdvanceFrame(delta_x);
  }
}

static void PrintGraphDataAsCsv(const GraphData& d) {
  (void)d;
#if PRINT_SPLINES_AS_CSV
  for (size_t i = 0; i < d.points.size(); ++i) {
    printf("%f, %f, %f, %f, %f\n", d.points[i].x(), d.points[i].y(),
           d.derivatives[i].first, d.derivatives[i].second,
           d.derivatives[i].third);
  }
#endif  // PRINT_SPLINES_AS_CSV
}

static void PrintSplineAsAsciiGraph(const GraphData& d) {
  (void)d;
#if PRINT_SPLINES_AS_ASCII_GRAPHS
  printf("\n%s\n\n",
         motive::Graph2DPoints(&d.points[0], static_cast<int>(d.points.size()),
                               kGraphSize)
             .c_str());
#endif  // PRINT_SPLINES_AS_ASCII_GRAPHS
}

static void GatherGraphData(const CubicInit& init, GraphData* d,
                            bool is_angle = false) {
  CompactSpline spline;
  InitializeSpline(init, &spline);

  BulkSplineEvaluator interpolator;
  interpolator.SetNumIndices(1);
  if (is_angle) {
    interpolator.SetYRanges(0, 1, kAngleRange);
  }
  interpolator.SetSplines(0, 1, &spline, motive::SplinePlayback());

  ExecuteInterpolator(interpolator, kNumCheckPoints, d);

  PrintGraphDataAsCsv(*d);
  PrintSplineAsAsciiGraph(*d);
}

class SplineTests : public ::testing::Test {
 protected:
  virtual void SetUp() {
    short_spline_.Init(Range(0.0f, 1.0f), 0.01f, 4);
    short_spline_.AddNode(0.0f, 0.1f, 0.0f, motive::kAddWithoutModification);
    short_spline_.AddNode(1.0f, 0.4f, 0.0f, motive::kAddWithoutModification);
    short_spline_.AddNode(4.0f, 0.2f, 0.0f, motive::kAddWithoutModification);
    short_spline_.AddNode(40.0f, 0.2f, 0.0f, motive::kAddWithoutModification);
    short_spline_.AddNode(100.0f, 1.0f, 0.0f, motive::kAddWithoutModification);
  }
  virtual void TearDown() {}

  CompactSpline short_spline_;
};

// Ensure the index lookup is accurate for x's before the range.
TEST_F(SplineTests, IndexForXBefore) {
  EXPECT_EQ(motive::kBeforeSplineIndex,
            short_spline_.IndexForX(-1.0f, kRidiculousSplineIndex));
}

// Ensure the index lookup is accurate for x's barely before the range.
TEST_F(SplineTests, IndexForXJustBefore) {
  EXPECT_EQ(0, short_spline_.IndexForX(-0.0001f, kRidiculousSplineIndex));
}

// Ensure the index lookup is accurate for x's barely before the range.
TEST_F(SplineTests, IndexForXBiggerThanGranularityAtStart) {
  EXPECT_EQ(0, short_spline_.IndexForX(-0.011f, kRidiculousSplineIndex));
}

// Ensure the index lookup is accurate for x's after the range.
TEST_F(SplineTests, IndexForXAfter) {
  EXPECT_EQ(motive::kAfterSplineIndex,
            short_spline_.IndexForX(101.0f, kRidiculousSplineIndex));
}

// Ensure the index lookup is accurate for x's barely after the range.
TEST_F(SplineTests, IndexForXJustAfter) {
  EXPECT_EQ(motive::kAfterSplineIndex,
            short_spline_.IndexForX(100.0001f, kRidiculousSplineIndex));
}

// Ensure the index lookup is accurate for x right at start.
TEST_F(SplineTests, IndexForXStart) {
  EXPECT_EQ(0, short_spline_.IndexForX(0.0f, kRidiculousSplineIndex));
}

// Ensure the index lookup is accurate for x right at end.
TEST_F(SplineTests, IndexForXEnd) {
  EXPECT_EQ(motive::kAfterSplineIndex,
            short_spline_.IndexForX(100.0f, kRidiculousSplineIndex));
}

// Ensure the index lookup is accurate for x just inside end.
TEST_F(SplineTests, IndexForXAlmostEnd) {
  EXPECT_EQ(motive::kAfterSplineIndex,
            short_spline_.IndexForX(99.9999f, kRidiculousSplineIndex));
}

// Ensure the index lookup is accurate for x just inside end.
TEST_F(SplineTests, IndexForXBiggerThanGranularityAtEnd) {
  EXPECT_EQ(3, short_spline_.IndexForX(99.99f, kRidiculousSplineIndex));
}

// Ensure the index lookup is accurate for x in middle, right on the node.
TEST_F(SplineTests, IndexForXMidOnNode) {
  EXPECT_EQ(1, short_spline_.IndexForX(1.0f, kRidiculousSplineIndex));
}

// Ensure the index lookup is accurate for x in middle, in middle of segment.
TEST_F(SplineTests, IndexForXMidAfterNode) {
  EXPECT_EQ(1, short_spline_.IndexForX(1.1f, kRidiculousSplineIndex));
}

// Ensure the index lookup is accurate for x in middle, in middle of segment.
TEST_F(SplineTests, IndexForXMidSecondLast) {
  EXPECT_EQ(2, short_spline_.IndexForX(4.1f, kRidiculousSplineIndex));
}

// Ensure the splines don't overshoot their mark.
TEST_F(SplineTests, Overshoot) {
  for (int i = 0; i < kNumSimpleSplines; ++i) {
    const CubicInit& init = kSimpleSplines[i];

    GraphData d;
    GatherGraphData(init, &d);

    const Range x_range(-kXGranularityScale,
                        init.width_x * (1.0f + kXGranularityScale));
    const Range y_range = CubicInitYRange(init, 0.001f);
    for (size_t j = 0; j < d.points.size(); ++j) {
      EXPECT_TRUE(x_range.Contains(d.points[j].x()));
      EXPECT_TRUE(y_range.Contains(d.points[j].y()));
    }
  }
}

// Ensure that the curves are mirrored in y when node y's are mirrored.
TEST_F(SplineTests, MirrorY) {
  for (int i = 0; i < kNumSimpleSplines; ++i) {
    const CubicInit& init = kSimpleSplines[i];
    const CubicInit mirrored_init = CubicInitMirrorY(init);
    const float y_precision =
        fabs(init.start_y - init.end_y) * kFixedPointEpsilon;

    GraphData d, mirrored_d;
    GatherGraphData(init, &d);
    GatherGraphData(mirrored_init, &mirrored_d);

    EXPECT_EQ(d.points.size(), mirrored_d.points.size());
    const int num_points = static_cast<int>(d.points.size());
    for (int j = 0; j < num_points; ++j) {
      EXPECT_EQ(d.points[j].x(), mirrored_d.points[j].x());
      EXPECT_NEAR(d.points[j].y(), -mirrored_d.points[j].y(), y_precision);
      EXPECT_NEAR(d.derivatives[j].first, -mirrored_d.derivatives[j].first,
                  kDerivativePrecision);
      EXPECT_NEAR(d.derivatives[j].second, -mirrored_d.derivatives[j].second,
                  kSecondDerivativePrecision);
      EXPECT_NEAR(d.derivatives[j].third, -mirrored_d.derivatives[j].third,
                  kThirdDerivativePrecision);
    }
  }
}

// Ensure that the curves are scaled in x when node's x is scaled.
TEST_F(SplineTests, ScaleX) {
  static const float kScale = 100.0f;
  for (int i = 0; i < kNumSimpleSplines; ++i) {
    const CubicInit& init = kSimpleSplines[i];
    const CubicInit scaled_init = CubicInitScaleX(init, kScale);
    const float x_precision = init.width_x * kFixedPointEpsilon;
    const float y_precision =
        fabs(init.start_y - init.end_y) * kFixedPointEpsilon;

    GraphData d, scaled_d;
    GatherGraphData(init, &d);
    GatherGraphData(scaled_init, &scaled_d);

    EXPECT_EQ(d.points.size(), scaled_d.points.size());
    const int num_points = static_cast<int>(d.points.size());
    for (int j = 0; j < num_points; ++j) {
      EXPECT_NEAR(d.points[j].x(), scaled_d.points[j].x() / kScale,
                  x_precision);
      EXPECT_NEAR(d.points[j].y(), scaled_d.points[j].y(), y_precision);
      EXPECT_NEAR(d.derivatives[j].first,
                  scaled_d.derivatives[j].first * kScale, kDerivativePrecision);
      EXPECT_NEAR(d.derivatives[j].second,
                  scaled_d.derivatives[j].second * kScale * kScale,
                  kSecondDerivativePrecision);
      EXPECT_NEAR(d.derivatives[j].third,
                  scaled_d.derivatives[j].third * kScale * kScale * kScale,
                  kThirdDerivativePrecision);
    }
  }
}

// YCalculatedSlowly should return the key-point Y values at key-point X values.
TEST_F(SplineTests, YSlowAtNodes) {
  for (CompactSplineIndex i = 0; i < short_spline_.NumNodes(); ++i) {
    EXPECT_NEAR(short_spline_.NodeY(i),
                short_spline_.YCalculatedSlowly(short_spline_.NodeX(i)),
                kNodeYPrecision);
  }
}

// BulkYs should return the proper start and end values.
TEST_F(SplineTests, BulkYsStartAndEnd) {
  static const int kMaxBulkYs = 5;

  // Get bulk data at several delta_xs, but always starting at the start of the
  // spline and ending at the end of the spline.
  // Then compare returned `ys` with start end end values of spline.
  for (size_t num_ys = 2; num_ys < kMaxBulkYs; ++num_ys) {
    float ys[kMaxBulkYs];
    CompactSpline::BulkYs(&short_spline_, 1, 0.0f,
                          short_spline_.EndX() / (num_ys - 1), num_ys, ys);

    EXPECT_NEAR(short_spline_.StartY(), ys[0], kNodeYPrecision);
    EXPECT_NEAR(short_spline_.EndY(), ys[num_ys - 1], kNodeYPrecision);
  }
}

// BulkYs should return the proper start and end values.
TEST_F(SplineTests, BulkYsVsSlowYs) {
  static const int kMaxBulkYs = 15;

  // Get bulk data at several delta_xs, but always starting at the start of the
  // spline and ending at the end of the spline.
  // Then compare returned `ys` with start end end values of spline.
  for (size_t num_ys = 2; num_ys < kMaxBulkYs; ++num_ys) {
    // Collect `num_ys` evenly-spaced samples from short_spline_.
    float ys[kMaxBulkYs];
    const float delta_x = short_spline_.EndX() / (num_ys - 1);
    CompactSpline::BulkYs(&short_spline_, 1, 0.0f, delta_x, num_ys, ys);

    // Compare bulk samples to slowly calcuated samples.
    float x = 0.0f;
    for (size_t j = 0; j < num_ys; ++j) {
      EXPECT_NEAR(short_spline_.YCalculatedSlowly(x), ys[j], kNodeYPrecision);
      x += delta_x;
    }
  }
}

// BulkYs should return the proper start and end values.
TEST_F(SplineTests, BulkYsVec3) {
  static const int kDimensions = 3;
  static const int kNumYs = 16;

  // Make three copies of the spline data.
  CompactSpline splines[kDimensions];
  for (size_t d = 0; d < kDimensions; ++d) {
    splines[d] = short_spline_;
  }

  // Collect `num_ys` evenly-spaced samples from short_spline_.
  vec3_packed ys[kNumYs];
  memset(ys, 0xFF, sizeof(ys));
  const float delta_x = short_spline_.EndX() / (kNumYs - 1);
  CompactSpline::BulkYs<3>(splines, 0.0f, delta_x, kNumYs, ys);

  // Ensure all the values are being calculated.
  for (int j = 0; j < kNumYs; ++j) {
    const vec3 y(ys[j]);
    EXPECT_EQ(y.x(), y.y());
    EXPECT_EQ(y.y(), y.z());
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
