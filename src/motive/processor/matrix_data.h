// Copyright 2014 Google Inc. All rights reserved.
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

#ifndef MOTIVE_MATRIX_DATA_H_
#define MOTIVE_MATRIX_DATA_H_

#include "mathfu/constants.h"
#include "motive/engine.h"
#include "motive/math/angle.h"
#include "motive/math/bulk_spline_evaluator.h"
#include "motive/matrix_init.h"
#include "motive/matrix_op.h"
#include "motive/util.h"

namespace motive {

// Perform a matrix rotation about
static inline void RotateAboutAxis(const float angle, mathfu::vec4* column0,
                                   mathfu::vec4* column1) {
  // TODO OPT: call platform-specific function to calculate both sin and cos
  // simultaneously.
  const float s = sin(angle);
  const float c = cos(angle);
  const mathfu::vec4 c0 = *column0;
  const mathfu::vec4 c1 = *column1;
  *column0 = c * c0 + s * c1;
  *column1 = c * c1 - s * c0;
}

// Hold a series of matrix operations, and their resultant matrix.
//
// This class is of variable size, to keep compact and to avoid cache misses
// caused by pointer chasing. The 'ops_[]' array is actually of length
// 'num_ops_'. Each item in 'ops_' is one matrix operation.
//
class MatrixData {
  MatrixData() {}  // Use Create() to create this class.
  ~MatrixData() {}

 public:

  // Execute the series of basic matrix operations in 'ops_'.
  // We break out the matrix into four column vectors to avoid matrix multiplies
  // (which are slow) in preference of operation-specific matrix math (which is
  // fast).
  mathfu::mat4 CalculateResultMatrix() const {
    // Start with the identity matrix.
    mathfu::vec4 c0 = mathfu::kAxisX4f;
    mathfu::vec4 c1 = mathfu::kAxisY4f;
    mathfu::vec4 c2 = mathfu::kAxisZ4f;
    mathfu::vec4 c3 = mathfu::kAxisW4f;

    for (int i = 0; i < num_ops_; ++i) {
      const MatrixOperation& op = ops_[i];
      const float value = op.Value();

      switch (op.Type()) {
        // ( |  |  |  |)(c -s  0  0)   (c*  c*   |   |)
        // (c0 c1 c2 c3)(s  c  0  0) = (c0+ c1- c2  c3)
        // ( |  |  |  |)(0  0  1  0)   (s*  s*   |   |)
        // ( |  |  |  |)(0  0  0  1)   (c1  c0   |   |)
        case kRotateAboutX:
          RotateAboutAxis(value, &c1, &c2);
          break;

        case kRotateAboutY:
          RotateAboutAxis(value, &c2, &c0);
          break;

        case kRotateAboutZ:
          RotateAboutAxis(value, &c0, &c1);
          break;

        // ( |  |  |  |)(1  0  0 tx)   ( |  |  | tx*c0+ )
        // (c0 c1 c2 c3)(0  1  0 ty) = (c0 c1 c2 ty*c1+ )
        // ( |  |  |  |)(0  0  1 tz)   ( |  |  | tz*c2+ )
        // ( |  |  |  |)(0  0  0  1)   ( |  |  |    c3  )
        case kTranslateX:
          c3 += value * c0;
          break;

        case kTranslateY:
          c3 += value * c1;
          break;

        case kTranslateZ:
          c3 += value * c2;
          break;

        // ( |  |  |  |)(sx 0  0  0)   ( |   |   |   |)
        // (c0 c1 c2 c3)(0  sy 0  0) = (sx* sy* sz*  |)
        // ( |  |  |  |)(0  0  sz 0)   (c0  c1  c2  c3)
        // ( |  |  |  |)(0  0  0  1)   ( |   |   |   |)
        case kScaleX:
          c0 *= value;
          break;

        case kScaleY:
          c1 *= value;
          break;

        case kScaleZ:
          c2 *= value;
          break;

        case kScaleUniformly:
          c0 *= value;
          c1 *= value;
          c2 *= value;
          break;

        default:
          assert(false);
      }
    }
    return mathfu::mat4(c0, c1, c2, c3);
  }

  void UpdateResultMatrix() { result_matrix_ = CalculateResultMatrix(); }

  void BlendToOps(const MatrixInit::OpVector& new_ops,
                  const motive::SplinePlayback& playback) {
    const int num_new_ops = static_cast<int>(new_ops.size());
    assert(num_ops_ >= num_new_ops);

    // Blend every operation to either the values in `ops` or, if `ops` doesn't
    // contain the operation, to the default value (0 for rotates and
    // translates, 1 for scales).
    int new_op_idx = 0;
    for (int i = 0; i < num_ops_; ++i) {
      MatrixOperation& op = ops_[i];
      if (new_op_idx < num_new_ops && op.Blendable(new_ops[new_op_idx])) {
        op.BlendToOp(new_ops[new_op_idx], playback);
        new_op_idx++;
      } else {
        op.BlendToDefault(static_cast<MotiveTime>(playback.blend_x));
      }
    }
    assert(new_op_idx == num_new_ops);
  }

  void SetPlaybackRate(float playback_rate) {
    for (int i = 0; i < num_ops_; ++i) {
      MatrixOperation& op = ops_[i];
      op.SetPlaybackRate(playback_rate);
    }
  }

  MotiveTime TimeRemaining() const {
    MotiveTime time = 0;
    for (int i = 0; i < num_ops_; ++i) {
      const MatrixOperation& op = ops_[i];
      time = std::max(time, op.TimeRemaining());
    }
    return time;
  }

  const MatrixOperation& Op(int child_index) const {
    assert(0 <= child_index && child_index < num_ops_);
    return ops_[child_index];
  }

  MatrixOperation& Op(int child_index) {
    assert(0 <= child_index && child_index < num_ops_);
    return ops_[child_index];
  }

  const mathfu::mat4& result_matrix() const { return result_matrix_; }
  int num_ops() const { return num_ops_; }

  static MatrixData* Create(const MatrixInit& init, MotiveEngine* engine) {
    // Allocate a buffer that is big enough to hold MatrixData.
    static const int kAlign = 16;
    const MatrixInit::OpVector& ops = init.ops();
    const int num_ops = static_cast<int>(ops.size());
    // Round up size to the next multiple of 16 to match minimum alignment.
    const size_t size = ((SizeOfClass(num_ops) + kAlign - 1) / kAlign) * kAlign;
    uint8_t* buffer = (uint8_t*)AlignedAlloc(size, kAlign);
    MatrixData* d = new (buffer) MatrixData();
    d->result_matrix_ = mathfu::mat4::Identity();
    d->num_ops_ = num_ops;
    for (int i = 0; i < num_ops; ++i) {
      new (&d->ops_[i]) MatrixOperation(ops[i], engine);
    }
    return d;
  }

  static void Destroy(MatrixData* d) {
    // Explicitly delete MatrixData the same way it was allocated.
    for (int i = 0; i < d->num_ops_; ++i) {
      d->ops_[i].~MatrixOperation();
    }
    d->~MatrixData();
    AlignedFree(d);
  }

 private:
  static size_t SizeOfClass(int num_ops) {
    return sizeof(MatrixData) +
           sizeof(MatrixOperation) * std::max(0, num_ops - 1);
  }

  /// Result of the most recent matrix update.
  mathfu::mat4 result_matrix_;

  /// Length of the `ops_` array below, which extends past the end of the
  /// defined class.
  int num_ops_;

  /// Matrix operations to perform. Of length `num_ops_`. Class is initialized
  /// in a chunk of memory that is big enough to hold
  /// ops_[0] .. ops_[num_ops_ - 1]
  MatrixOperation ops_[1];
};

}  // namespace motive

#endif  // MOTIVE_MATRIX_DATA_H_
