/*
* Copyright (c) 2015 Google, Inc.
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

#include <stdio.h>
#include <vector>
#include "motive/common.h"
#include "motive/engine.h"
#include "motive/math/angle.h"
#include "motive/math/curve.h"
#include "motive/math/compact_spline.h"
#include "motive/init.h"
#include "motive/util/benchmark.h"

using fpl::CompactSpline;
using fpl::CubicCurve;
using fpl::CubicInit;
using fpl::kPi;
using fpl::kTwoPi;
using fpl::QuadraticCurve;
using fpl::Range;
using fpl::SplinePlayback;
using mathfu::vec2;
using mathfu::vec2i;
using motive::MotiveEngine;
using motive::MatrixInit;
using motive::MotivatorMatrix4f;
using motive::SmoothInit;

static const SmoothInit kRotateInit(Range(-kPi, kPi), true);
static const SmoothInit kTranslateInit(Range(-1.0f, 1.0f), true);
static const int kNumBenchmarkIds = 10;

struct SplineNode {
  float x;
  float y;
  float derivative;
};

static const SplineNode kSinWave[] = {
  { 0.0f,        0.0f,  1.0f },
  { 0.5f * kPi,  1.0f,  0.0f },
  { kPi,         0.0f, -1.0f },
  { 1.5f * kPi, -1.0f,  0.0f },
  { kTwoPi,      0.0f,  1.0f }
};

static const SplineNode kStraightLine[] = {
  { 0.0f,  0.0f,  1.0f },
  { 1.0f,  1.0f,  1.0f }
};

static const float kLinearOrbitPeriod = 2000.0f;
static const float kOscillatingSlowlyPeriod = 500.0f;
static const float kOscillatingSlowlyAmplitude = 0.3f;
static const float kOscillatingQuicklyPeriod = 200.0f;
static const float kOscillatingQuicklyAmplitude = 0.1f;


// Take an array of SplineNodes (x, y, derivative) values and scale them
// to create a CompactSpline. We use Dual Cubic interpolation to ensure that
// the splines are well behaved.
static void CreateSpline(const SplineNode* nodes, size_t num_nodes,
                         float x_scale, float y_scale, CompactSpline* spline) {
  // Find y-extremes.
  float min = std::numeric_limits<float>::infinity();
  float max = -std::numeric_limits<float>::infinity();
  for (size_t i = 0; i < num_nodes; ++i) {
    min = std::min(nodes[i].y, min);
    max = std::max(nodes[i].y, max);
  }

  // Initialize the spline such that it's bounds are tight to the data.
  spline->Init(
      Range(y_scale * min, y_scale * max),
      CompactSpline::RecommendXGranularity(x_scale * nodes[num_nodes - 1].x));

  // Scale each node and add it to the curve.
  for (size_t i = 0; i < num_nodes; ++i) {
    const SplineNode& n = nodes[i];
    spline->AddNode(n.x * x_scale, n.y * y_scale, n.derivative / x_scale);
  }
}

// Create a large number of matrix motivators that are each driven by multiple
// one dimensional motivators. Then advance them over-and-over, gathering
// measuring the running time. Print the results in histograms, periodically.
class MotiveBenchmarker {
 public:
  MotiveBenchmarker() {
    MatrixInit::Register();
    SmoothInit::Register();

    // Create compact splines by modifying some basic functions (straight
    // line and sign wave). The straight line, in this case, represents an
    // angle that travels a total of 2pi, so it ends up where it started.
    CreateSpline(kStraightLine, ARRAYSIZE(kStraightLine), kLinearOrbitPeriod,
                 kTwoPi, &splines_[kLinearOrbit]);
    CreateSpline(kSinWave, ARRAYSIZE(kSinWave), kOscillatingSlowlyPeriod,
                 kOscillatingSlowlyAmplitude, &splines_[kOscillatingSlowly]);
    CreateSpline(kSinWave, ARRAYSIZE(kSinWave), kOscillatingQuicklyPeriod,
                 kOscillatingQuicklyAmplitude, &splines_[kOscillatingQuickly]);

    // Add some information for how to play-back the spline:
    // the start time (always 0 here), and whether to repeat (always true here).
    const SplinePlayback linearPlayback(splines_[kLinearOrbit], 0.0f, true);
    const SplinePlayback oscillatingSlowlyPlayback(
        splines_[kOscillatingSlowly], 0.0f, true);
    const SplinePlayback oscillatingQuicklyPlayback(
        splines_[kOscillatingQuickly], 0.0f, true);

    // Create a matrix initializer with a series of basic matrix operations.
    // The final matrix will be created by applying these operations, in turn.
    matrix_init_.AddOp(motive::kRotateAboutY, kRotateInit, linearPlayback);
    matrix_init_.AddOp(motive::kTranslateX, kTranslateInit,
                       oscillatingSlowlyPlayback);
    matrix_init_.AddOp(motive::kTranslateY, kTranslateInit,
                       oscillatingQuicklyPlayback);

    // Initialize the large array of matrix motivators. Note that the
    // 1-dimensional motivators that drive the matrix motivators are created
    // by the matrix motivators themselves.
    for (size_t i = 0; i < kNumMatrices; ++i) {
      MotivatorMatrix4f& m = matrices_[i];
      m.Initialize(matrix_init_, &engine_);
    }
  }

  void Run() {
    for (int i = 0; i < kNumReports; ++i) {
      // Advance the engine many times, gathering benchmark information on
      // each iteration.
      for (int j = 0; j < kNumIterationsPerReport; ++j) {
        engine_.AdvanceFrame(1);
      }

      // Output benchmark statistics and empty the stats counter.
      fpl::OutputBenchmarks();
      fpl::ClearBenchmarks();
    }
  }

 private:
  // Total number of matrices to animate.
  static const size_t kNumMatrices = 10000;

  // Number of times to run before quitting.
  static const int kNumReports = 100;

  // Number of benchmark samples to collect before analyzing.
  static const int kNumIterationsPerReport = 1000;

  enum ChildImpellers {
    kLinearOrbit,
    kOscillatingSlowly,
    kOscillatingQuickly,
    kNumChildImpellers
  };

  MotiveEngine engine_;
  MatrixInit matrix_init_;
  CompactSpline splines_[kNumChildImpellers];
  MotivatorMatrix4f matrices_[kNumMatrices];
};

int main() {
  fpl::InitBenchmarks(kNumBenchmarkIds);
  MotiveBenchmarker benchmarker;
  benchmarker.Run();
  return 0;
}

