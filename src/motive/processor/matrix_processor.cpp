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

#include "motive/matrix_processor.h"
#include "mathfu/constants.h"
#include "motive/engine.h"
#include "motive/matrix_init.h"
#include "motive/math/angle.h"
#include "motive/math/bulk_spline_evaluator.h"
#include "motive/processor/matrix_data.h"

namespace motive {

// See comments on MatrixInit for details on this class.
class MatrixMotiveProcessor : public MatrixProcessor4f {
 public:
  MatrixMotiveProcessor() : time_(0), engine_(nullptr) {}

  virtual ~MatrixMotiveProcessor() {
    RemoveIndices(0, NumIndices());
  }

  virtual void AdvanceFrame(MotiveTime delta_time) {
    Defragment();

    // Process the series of matrix operations for each index.
    const MotiveIndex num_indices = NumIndices();
    for (MotiveIndex index = 0; index < num_indices; ++index) {
      MatrixData& d = Data(index);
      d.UpdateResultMatrix();
    }

    // Update our global time. It shouldn't matter if this wraps
    // around, since we only calculate times relative to it.
    time_ += delta_time;
  }

  virtual MotivatorType Type() const { return MatrixInit::kType; }
  virtual int Priority() const { return 2; }

  virtual const mathfu::mat4& Value(MotiveIndex index) const {
    return Data(index).result_matrix();
  }

  virtual void Value(MotiveIndex index, mathfu::vec3* translation,
                     mathfu::vec4* rotation, mathfu::vec3* scale) const {
    const MatrixData& data = Data(index);
    *translation = data.result_translation();
    *scale = data.result_scale();

    // Pack the quaternion into a vec4 to accomadate Motivator's API.
    const mathfu::quat quat = data.result_rotation();
    *rotation = mathfu::vec4(quat.vector(), quat.scalar());
  }

  virtual int NumChildren(MotiveIndex index) const {
    return Data(index).num_ops();
  }

  virtual void ChildValues(MotiveIndex index, MotiveChildIndex child_index,
                           MotiveChildIndex count, float* values) const {
    const MatrixData& d = Data(index);
    for (MotiveChildIndex i = 0; i < count; ++i) {
      values[i] = d.Op(child_index + i).Value();
    }
  }

  virtual const Motivator1f* ChildMotivator1f(
      MotiveIndex index, MotiveChildIndex child_index) const {
    return Data(index).Op(child_index).ValueMotivator();
  }

  virtual void SetChildTarget1f(MotiveIndex index, MotiveChildIndex child_index,
                                const MotiveTarget1f& t) {
    Data(index).Op(child_index).SetTarget1f(t);
    // TODO: Update end time.
  }

  virtual void SetChildValues(MotiveIndex index, MotiveChildIndex child_index,
                              MotiveChildIndex count, const float* values) {
    MatrixData& d = Data(index);
    for (MotiveChildIndex i = 0; i < count; ++i) {
      d.Op(child_index + i).SetValue1f(values[i]);
    }
  }

  virtual void BlendToOps(MotiveIndex index,
                          const std::vector<MatrixOperationInit>& ops,
                          const motive::SplinePlayback& playback) {
    assert(engine_);
    Data(index).BlendToOps(ops, playback, engine_);
  }

  virtual void SetPlaybackRate(MotiveIndex index, float playback_rate) {
    Data(index).SetPlaybackRate(playback_rate);
  }

  virtual MotiveTime TimeRemaining(MotiveIndex index) const {
    return Data(index).TimeRemaining();
  }

 protected:
  MotiveIndex NumIndices() const {
    return static_cast<MotiveIndex>(data_.size());
  }

  virtual void InitializeIndices(const MotivatorInit& init, MotiveIndex index,
                                 MotiveDimension dimensions,
                                 MotiveEngine* engine) {
    // Hold onto the engine for use in BlendToOps().
    engine_ = engine;
    RemoveIndices(index, dimensions);

    // TODO OPT: Create only one MatrixData that holds `dimensions` matrices,
    //           so that we can process in bulk.
    auto init_params = static_cast<const MatrixInit&>(init);
    for (MotiveIndex i = index; i < index + dimensions; ++i) {
      data_[i].Initialize(init_params, engine);
    }
  }

  virtual void RemoveIndices(MotiveIndex index, MotiveDimension dimensions) {
    // Callers depend on indices staying consistent between calls to this
    // function, so just reset the MatrixData states to empty instead of erasing
    // them.
    for (MotiveIndex i = index; i < index + dimensions; ++i) {
      data_[i].Reset();
    }
  }

  virtual void MoveIndices(MotiveIndex old_index, MotiveIndex new_index,
                           MotiveDimension dimensions) {
    MotiveIndex old_i = old_index;
    MotiveIndex new_i = new_index;
    for (MotiveDimension i = 0; i < dimensions; ++i, ++new_i, ++old_i) {
      using std::swap;
      swap(data_[new_i], data_[old_i]);
    }
  }

  virtual void SetNumIndices(MotiveIndex num_indices) {
    // Ensure old items are deleted.
    const MotiveIndex old_num_indices = NumIndices();
    if (old_num_indices > num_indices) {
      RemoveIndices(num_indices, old_num_indices - num_indices);
    }

    // Default-inserts new empty MatrixDatas.
    data_.resize(num_indices);
  }

  const MatrixData& Data(MotiveIndex index) const {
    assert(ValidIndex(index));
    return data_[index];
  }

  MatrixData& Data(MotiveIndex index) {
    assert(ValidIndex(index));
    return data_[index];
  }

  std::vector<MatrixData> data_;
  MotiveTime time_;
  MotiveEngine* engine_;
};

MOTIVE_INSTANCE(MatrixInit, MatrixMotiveProcessor);

}  // namespace motive
