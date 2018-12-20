// Copyright 2018 Google Inc. All rights reserved.
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

#ifndef MOTIVE_SQT_DATA_H_
#define MOTIVE_SQT_DATA_H_

#include "mathfu/constants.h"
#include "motive/engine.h"
#include "motive/matrix_op.h"
#include "motive/sqt_init.h"
#include "third_party/motive/include/motive/matrix_op.h"

namespace motive {

// Hold a set of animations representing the translation, quaternion rotation,
// and scale components of a transform.
class SqtData {
 public:
  SqtData()
      : result_matrix_(mathfu::mat4::Identity()),
        rotation_(mathfu::quat::identity),
        scale_(mathfu::kOnes3f),
        ops_(0) {}

  void Initialize(const SqtInit& init, MotiveEngine* engine) {
    const std::vector<MatrixOperationInit>& ops = init.ops();
    int num_ops = static_cast<int>(ops.size());
    ops_.reserve(num_ops);
    for (int i = 0; i < num_ops; ++i) {
      // Rotation ops must use the MatrixMotiveProcessor.
      assert(!RotateOp(ops[i].type));
      ops_.emplace_back(ops[i], engine);
    }

    // Initialize the result matrix to the default value. This ensures that
    // RigMotivators can initialize their transforms to the default pose of the
    // defining animation.
    UpdateResultMatrix();
  }

  void Reset() {
    rotation_ = DefaultOpsQuaternion();
    scale_ = DefaultOpsScale();
    result_matrix_ = mathfu::mat4::Transform(DefaultOpsTranslation(),
                                             rotation_.ToMatrix(), scale_);
    ops_.resize(0);
  }

  void UpdateResultMatrix() {
    // Ensure the transforms are default to start since the primary reason for
    // missing ops is when they are constant default values.
    mathfu::vec3 translation = DefaultOpsTranslation();
    rotation_ = DefaultOpsQuaternion();
    scale_ = DefaultOpsScale();

    for (size_t i = 0; i < ops_.size(); ++i) {
      const MatrixOperation& op = ops_[i];
      const float value = op.Value();

      switch (op.Type()) {
        case kTranslateX:
          translation.x = value;
          break;

        case kTranslateY:
          translation.y = value;
          break;

        case kTranslateZ:
          translation.z = value;
          break;

        case kScaleX:
          scale_.x = value;
          break;

        case kScaleY:
          scale_.y = value;
          break;

        case kScaleZ:
          scale_.z = value;
          break;

        case kScaleUniformly:
          scale_.x = value;
          scale_.y = value;
          scale_.z = value;
          break;

        case kQuaternionW:
          rotation_.set_scalar(value);
          break;

        case kQuaternionX: {
          mathfu::vec3 v = rotation_.vector();
          v.x = value;
          rotation_.set_vector(v);
          break;
        }

        case kQuaternionY: {
          mathfu::vec3 v = rotation_.vector();
          v.y = value;
          rotation_.set_vector(v);
          break;
        }

        case kQuaternionZ: {
          mathfu::vec3 v = rotation_.vector();
          v.z = value;
          rotation_.set_vector(v);
          break;
        }

        default:
          // All other operations, including RotateAbout, are not supported.
          assert(false);
      }
    }

    // Quaternion values may be interpolated and result in a non-unit
    // quaternion, meaning they must be normalized.
    rotation_.Normalize();
    result_matrix_ =
        mathfu::mat4::Transform(translation, rotation_.ToMatrix(), scale_);
  }

  void BlendToOps(const std::vector<MatrixOperationInit>& new_ops,
                  const motive::SplinePlayback& playback,
                  MotiveEngine* engine) {
    // Since q and -q represent the same orientation, the current quaternion
    // values may need to be negated to ensure the blend doesn't wildly change
    // individual component values.
    AlignQuaternionOps(new_ops);

    // Ops are always stored in order of ascending IDs. Scan through the old
    // and new ops trying to match IDs.
    int old_idx = 0;
    int new_idx = 0;
    const int num_new_ops = static_cast<int>(new_ops.size());

    while (old_idx < ops_.size() && new_idx < num_new_ops) {
      MatrixOperation& old_op = ops_[old_idx];
      const MatrixOperationInit& new_op = new_ops[new_idx];

      // Rotation ops must use the MatrixMotiveProcessor.
      assert(!RotateOp(new_op.type));

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
        ops_.emplace(ops_.begin() + old_idx, new_op, playback, engine);
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
      ops_.emplace_back(new_ops[new_idx++], playback, engine);
    }
  }

  void SetPlaybackRate(float playback_rate) {
    for (int i = 0, num_ops = ops_.size(); i < num_ops; ++i) {
      ops_[i].SetPlaybackRate(playback_rate);
    }
  }

  void SetRepeating(bool repeat) {
    for (int i = 0, num_ops = ops_.size(); i < num_ops; ++i) {
      ops_[i].SetRepeating(repeat);
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
  mathfu::quat result_rotation() const { return rotation_; }
  mathfu::vec3 result_scale() const { return scale_; }

  int num_ops() const { return ops_.size(); }

 private:
  // Ensures that the current quaternion values are close to the initial values
  // in `ops`. This function should be called prior to blending to `ops` to
  // ensure quaternion blends work.
  void AlignQuaternionOps(const std::vector<MatrixOperationInit>& new_ops) {
    // Determine the first quaternion from the new animation.
    mathfu::quat next = mathfu::quat::identity;
    for (int i = 0; i < new_ops.size(); ++i) {
      const MatrixOperationInit& op = new_ops[i];
      if (QuaternionOp(op.type)) {
        if (op.type == kQuaternionW) {
          next.set_scalar(op.StartValue());
        } else {
          mathfu::vec3 v = next.vector();
          if (op.type == kQuaternionX) {
            v.x = op.StartValue();
          } else if (op.type == kQuaternionY) {
            v.y = op.StartValue();
          } else if (op.type == kQuaternionZ) {
            v.z = op.StartValue();
          }
          next.set_vector(v);
        }
      }
    }
    next.Normalize();

    // Since q and -q represent the same orientation, we can negate the current
    // quaternion operations if it will make them closer to `next`.
    if (mathfu::quat::DotProduct(rotation_, next) < 0.f) {
      for (int i = 0; i < ops_.size(); ++i) {
        MatrixOperation& op = ops_[i];
        if (QuaternionOp(op.Type())) {
          const float value = op.Value();
          // Ops driven by motivators should have both values and velocities
          // negated. Constants only need to negate their values.
          if (op.ValueMotivator()) {
            op.SetTarget1f(Current1f(-value, -op.Velocity()));
          } else {
            op.SetValue1f(-value);
          }
        }
      }
    }
  }

  /// Result of the most recent matrix update.
  mathfu::mat4 result_matrix_;

  // The pre-extracted rotation and scale components to make result_rotation()
  // and result_scale() faster. The translation component is fetched directly
  // from result_matrix_ and doesn't need to be stored separately.
  mathfu::quat rotation_;
  mathfu::vec3 scale_;

  /// Operations representing the individual components of the translation,
  /// quaternion rotation, and scale.
  std::vector<MatrixOperation> ops_;
};

}  // namespace motive

#endif  // MOTIVE_SQT_DATA_H_
