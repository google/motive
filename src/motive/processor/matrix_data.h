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

  void UpdateResultMatrix() {
    result_matrix_ = MatrixOperation::CalculateResultMatrix(ops_, num_ops_);
  }

  void BlendToOps(const std::vector<MatrixOperationInit>& new_ops,
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
    const std::vector<MatrixOperationInit>& ops = init.ops();
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
