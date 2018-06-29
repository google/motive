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
class MatrixData {
 public:
  MatrixData() : result_matrix_(mathfu::mat4::Identity()), ops_(0) {}
  MatrixData(MatrixData&& rhs) = default;
  MatrixData& operator=(MatrixData&& rhs) = default;

  void Initialize(const MatrixInit& init, MotiveEngine* engine) {
    const std::vector<MatrixOperationInit>& ops = init.ops();
    int num_ops = static_cast<int>(ops.size());
    ops_.reserve(num_ops);
    for (int i = 0; i < num_ops; ++i) {
      ops_.emplace_back(ops[i], engine);
    }
  }

  void Reset() {
    result_matrix_ = mathfu::mat4::Identity();
    ops_.resize(0);
  }

  void UpdateResultMatrix() {
    result_matrix_ = MatrixOperation::CalculateResultMatrix(
        ops_.data(), ops_.size(), &scale_);
  }

  void BlendToOps(const std::vector<MatrixOperationInit>& new_ops,
                  const motive::SplinePlayback& playback,
                  MotiveEngine* engine) {
    // Ops are always stored in order of ascending IDs. Scan through the old
    // and new ops trying to match IDs.
    int old_idx = 0;
    int new_idx = 0;
    const int num_new_ops = static_cast<int>(new_ops.size());

    while (old_idx < ops_.size() && new_idx < num_new_ops) {
      MatrixOperation& old_op = ops_[old_idx];
      const MatrixOperationInit& new_op = new_ops[new_idx];

      // Ops are blendable if they have identical IDs. If not, handle whichever
      // has the lower ID since it cannot possibly have a Blendable op in the
      // other list.
      if (old_op.Blendable(new_op)) {
        old_op.BlendToOp(new_op, playback, engine);
        ++old_idx;
        ++new_idx;
      } else if (old_op.Id() < new_op.id) {
        // Old ops blend to default.
        old_op.BlendToDefault(static_cast<MotiveTime>(playback.blend_x));
        ++old_idx;
      } else {
        // New ops are inserted in order. `old_idx` points to the old op with
        // the next highest ID compared to `new_op`, meaning it also provides
        // the correct insertion point.
        ops_.emplace(ops_.begin() + old_idx, new_op, engine);
        ++new_idx;
        // Advance `old_idx` so it still points to the same `old_op` now that
        // one has been inserted before it.
        ++old_idx;
      }
    }

    // Fill out remaining old ops by blending them to default.
    int num_ops = ops_.size();
    while (old_idx < num_ops) {
      ops_[old_idx++].BlendToDefault(static_cast<MotiveTime>(playback.blend_x));
    }

    // Fill out remaining new ops by inserting them.
    while (new_idx < num_new_ops) {
      ops_.emplace_back(new_ops[new_idx++], engine);
    }
  }

  void SetPlaybackRate(float playback_rate) {
    for (int i = 0, num_ops = ops_.size(); i < num_ops; ++i) {
      MatrixOperation& op = ops_[i];
      op.SetPlaybackRate(playback_rate);
    }
  }

  MotiveTime TimeRemaining() const {
    MotiveTime time = 0;
    for (int i = 0, num_ops = ops_.size(); i < num_ops; ++i) {
      const MatrixOperation& op = ops_[i];
      time = std::max(time, op.TimeRemaining());
    }
    return time;
  }

  const MatrixOperation& Op(int child_index) const {
    assert(0 <= child_index && child_index < ops_.size());
    return ops_[child_index];
  }

  MatrixOperation& Op(int child_index) {
    assert(0 <= child_index && child_index < ops_.size());
    return ops_[child_index];
  }

  const mathfu::mat4& result_matrix() const { return result_matrix_; }

  mathfu::vec3 result_translation() const {
    return result_matrix_.TranslationVector3D();
  }

  mathfu::quat result_rotation() const {
    // Extract the rotation matrix from the upper-left 3x3 of the result matrix
    // and remove the pre-computed scale from it.
    const mathfu::vec3 inv_scale = mathfu::kOnes3f / scale_;
    const mathfu::mat3 rot(
        result_matrix_(0, 0) * inv_scale.x, result_matrix_(1, 0) * inv_scale.x,
        result_matrix_(2, 0) * inv_scale.x, result_matrix_(0, 1) * inv_scale.y,
        result_matrix_(1, 1) * inv_scale.y, result_matrix_(2, 1) * inv_scale.y,
        result_matrix_(0, 2) * inv_scale.z, result_matrix_(1, 2) * inv_scale.z,
        result_matrix_(2, 2) * inv_scale.z);
    return mathfu::quat::FromMatrix(rot);
  }

  mathfu::vec3 result_scale() const { return scale_; }

  int num_ops() const { return ops_.size(); }

 private:
  /// Result of the most recent matrix update.
  mathfu::mat4 result_matrix_;

  // The pre-extracted scale components to make result_rotation() and
  // result_scale() faster.
  mathfu::vec3 scale_;

  /// Matrix operations to perform.
  std::vector<MatrixOperation> ops_;
};

}  // namespace motive

#endif  // MOTIVE_MATRIX_DATA_H_
