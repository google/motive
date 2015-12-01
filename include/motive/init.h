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

#ifndef MOTIVE_INIT_H_
#define MOTIVE_INIT_H_

#include "mathfu/constants.h"
#include "motive/util.h"
#include "motive/math/range.h"

namespace motive {

class RigAnim;

enum MatrixOperationType {
  kInvalidMatrixOperation,
  kRotateAboutX,
  kRotateAboutY,
  kRotateAboutZ,
  kTranslateX,
  kTranslateY,
  kTranslateZ,
  kScaleX,
  kScaleY,
  kScaleZ,
  kScaleUniformly,
  kNumMatrixOperationTypes
};

/// Returns true if the operation is a rotate.
inline bool RotateOp(MatrixOperationType op) {
  return kRotateAboutX <= op && op <= kRotateAboutZ;
}

/// Returns true if the operation is a translate.
inline bool TranslateOp(MatrixOperationType op) {
  return kTranslateX <= op && op <= kTranslateZ;
}

/// Returns true if the operation is a scale.
inline bool ScaleOp(MatrixOperationType op) {
  return kScaleX <= op && op <= kScaleUniformly;
}

/// Returns the default value of the operation. That is, the value of the
/// operation that does nothing to the transformation. Any operation that
/// constantly returns the default value can be removed.
inline float OperationDefaultValue(MatrixOperationType op) {
  return RotateOp(op) || TranslateOp(op) ? 0.0f : 1.0f;
}

/// Returns true if this operation uses modular arithmetic.
/// Rotations are equivalence every 2pi, so they are modular.
inline bool ModularOp(MatrixOperationType op) { return RotateOp(op); }

/// Returns the range of the matrix operation's spline. Most ranges are just
/// the extents of the splines, but rotations we want to normalize within
/// +-pi before blending to another curve.
inline Range RangeOfOp(MatrixOperationType op,
                                const Range& extents) {
  return ModularOp(op) ? kAngleRange : extents;
}

/// Return a string with the operation name. Used for debugging.
const char* MatrixOpName(const MatrixOperationType op);

/// @class OvershootInit
/// @brief Initialize a MotivatorNf move towards a target using spring physics.
///
/// Call MotivatorNf::SetTargets() to set the target that we swing towards.
/// The name comes from the movement overshooting the target then coming
/// back, the way a dampened oscillator overshoots its resting point.
class OvershootInit : public MotivatorInit {
 public:
  MOTIVE_INTERFACE();

  OvershootInit()
      : MotivatorInit(kType),
        range_(Range::Full()),
        modular_(false),
        max_velocity_(0.0f),
        accel_per_difference_(0.0f),
        wrong_direction_multiplier_(0.0f),
        max_delta_time_(0) {}

  /// Ensure velocity is within the reasonable limits.
  float ClampVelocity(float velocity) const {
    return mathfu::Clamp(velocity, -max_velocity_, max_velocity_);
  }

  /// Ensure the Motivator's 'value' doesn't increment by more than 'max_delta'.
  /// This is different from ClampVelocity because it is independent of time.
  /// No matter how big the timestep, the delta will not be too great.
  float ClampDelta(float delta) const {
    return mathfu::Clamp(delta, -max_delta_, max_delta_);
  }

  /// Return true if we're close to the target and almost stopped.
  /// The definition of "close to" and "almost stopped" are given by the
  /// "at_target" member.
  bool AtTarget(float dist, float velocity) const {
    return at_target_.Settled(dist, velocity);
  }

  const Range& range() const { return range_; }
  void set_range(const Range& r) { range_ = r; }
  bool modular() const { return modular_; }
  void set_modular(bool modular) { modular_ = modular; }
  float max_velocity() const { return max_velocity_; }
  float max_delta() const { return max_delta_; }
  const Settled1f& at_target() const { return at_target_; }
  Settled1f& at_target() { return at_target_; }
  float accel_per_difference() const { return accel_per_difference_; }
  float wrong_direction_multiplier() const {
    return wrong_direction_multiplier_;
  }
  MotiveTime max_delta_time() const { return max_delta_time_; }

  void set_max_velocity(float max_velocity) { max_velocity_ = max_velocity; }
  void set_max_delta(float max_delta) { max_delta_ = max_delta; }
  void set_at_target(const Settled1f& at_target) { at_target_ = at_target; }
  void set_accel_per_difference(float accel_per_difference) {
    accel_per_difference_ = accel_per_difference;
  }
  void set_wrong_direction_multiplier(float wrong_direction_multiplier) {
    wrong_direction_multiplier_ = wrong_direction_multiplier;
  }
  void set_max_delta_time(MotiveTime max_delta_time) {
    max_delta_time_ = max_delta_time;
  }

 private:
  /// Minimum and maximum values for Motivator::Value().
  /// Clamp (if modular_ is false) or wrap-around (if modular_ is true) when
  /// we reach these boundaries.
  Range range_;

  /// A modular value wraps around from min to max. For example, an angle
  /// is modular, where -pi is equivalent to +pi. Setting this to true ensures
  /// that arithmetic wraps around instead of clamping to min/max.
  bool modular_;

  /// Maximum speed at which the value can change. That is, maximum value for
  /// the Motivator::Velocity(). In units/tick.
  /// For example, if the value is an angle, then this is the max angular
  /// velocity, and the units are radians/tick.
  float max_velocity_;

  /// Maximum that Motivator::Value() can be altered on a single call to
  /// MotiveEngine::AdvanceFrame(), regardless of velocity or delta_time.
  float max_delta_;

  /// Cutoff to determine if the Motivator's current state has settled on the
  /// target. Once it has settled, Value() is set to TargetValue() and
  /// Velocity() is set to zero.
  Settled1f at_target_;

  /// Acceleration is a multiple of abs('state_.position' - 'target_.position').
  /// Bigger differences cause faster acceleration.
  float accel_per_difference_;

  /// When accelerating away from the target, we multiply our acceleration by
  /// this amount. We need counter-acceleration to be stronger so that the
  /// amplitude eventually dies down; otherwise, we'd just have a pendulum.
  float wrong_direction_multiplier_;

  /// The algorithm is iterative. When the iteration step gets too big, the
  /// behavior becomes erratic. This value clamps the iteration step.
  MotiveTime max_delta_time_;
};

/// @class SplineInit
/// @brief Initialize a MotivatorNf to follow a spline.
///
/// Call MotivatorNf::SetSplines() to follow predefined splines,
/// or call MotivatorNf::SetTargets() to dynamically generate a spline that
/// travels through several key points.
class SplineInit : public MotivatorInit {
 public:
  MOTIVE_INTERFACE();

  SplineInit() : MotivatorInit(kType), range_(Range::Full()), modular_(false) {}

  /// The derived type must call this constructor with it's MotivatorType
  /// identifier.
  /// @param range The valid range of a normalized value. Ignored when modular
  ///              is false. Note: it's possible for MotivatorNf::Value() to
  ///              return a value outside of this range, though all splines
  ///              start inside the valid range.
  /// @param modular Option to use modular arithmetic when initializing
  ///                splines. If true, the initial spline value is always
  ///                within `range`.
  SplineInit(const Range& range, bool modular)
      : MotivatorInit(kType), range_(range), modular_(modular) {}

  const Range& range() const { return range_; }
  void set_range(const Range& r) { range_ = r; }

  bool modular() const { return modular_; }
  void set_modular(bool modular) { modular_ = modular; }

 private:
  /// Minimum and maximum values for Motivator::Value().
  /// Clamp (if modular_ is false) or wrap-around (if modular_ is true) when
  /// we reach these boundaries.
  Range range_;

  /// A modular value wraps around from min to max. For example, an angle
  /// is modular, where -pi is equivalent to +pi. Setting this to true ensures
  /// that arithmetic wraps around instead of clamping to min/max.
  bool modular_;
};

/// @class MatrixOperationInit
/// @brief Init params for a basic operation on a matrix.
struct MatrixOperationInit {
  enum UnionType {
    kUnionEmpty,
    kUnionInitialValue,
    kUnionTarget,
    kUnionSpline
  };

  /// Matrix operation never changes. Always use 'const_value'.
  MatrixOperationInit(MatrixOperationType type, float const_value)
      : init(nullptr),
        type(type),
        union_type(kUnionInitialValue),
        initial_value(const_value) {}

  /// Matrix operation is driven by Motivator defined by 'init'.
  MatrixOperationInit(MatrixOperationType type, const MotivatorInit& init)
      : init(&init), type(type), union_type(kUnionEmpty) {}

  /// Matrix operation is driven by Motivator defined by 'init'. Specify initial
  /// value as well.
  MatrixOperationInit(MatrixOperationType type, const MotivatorInit& init,
                      float initial_value)
      : init(&init),
        type(type),
        union_type(kUnionInitialValue),
        initial_value(initial_value) {}

  MatrixOperationInit(MatrixOperationType type, const MotivatorInit& init,
                      const MotiveTarget1f& target)
      : init(&init), type(type), union_type(kUnionTarget), target(&target) {}

  MatrixOperationInit(MatrixOperationType type, const MotivatorInit& init,
                      const CompactSpline& spline)
      : init(&init), type(type), union_type(kUnionSpline), spline(&spline) {}

  const MotivatorInit* init;
  MatrixOperationType type;
  UnionType union_type;
  union {
    float initial_value;
    const MotiveTarget1f* target;
    const CompactSpline* spline;
  };
};

/// @class MatrixInit
/// @brief Initialize a MatrixMotivator4f to generate its matrix from
///        a series of operations.
///
/// Initialize a MatrixMotivator4f with these initialization parameters to
/// create a motivator that generates a 4x4 matrix from a series of basic
/// matrix operations. The basic matrix operations are driven by one dimensional
/// motivators.
///
/// The series of operations can transform an object from the coordinate space
/// in which it was authored, to world (or local) space. For example, if you
/// have a penguin that is authored at (0,0,0) facing up the x-axis, you can
/// move it to it's target position with four operations:
///
///      kScaleUniformly --> to make penguin the correct size
///      kRotateAboutY --> to make penguin face the correct direction
///      kTranslateX } --> to move penguin along to ground to target position
///      kTranslateZ }
class MatrixOpArray {
 public:
  typedef std::vector<MatrixOperationInit> OpVector;

  // Guess at the number of operations we'll have. Better to high-ball a little
  // so that we don't have to reallocate the `ops_` vector.
  static const int kDefaultExpectedNumOps = 8;

  /// By default expect a relatively high number of ops. Cost for allocating
  /// a bit too much temporary memory is small compared to cost of reallocating
  /// that memory.
  explicit MatrixOpArray(int expected_num_ops = kDefaultExpectedNumOps) {
    ops_.reserve(expected_num_ops);
  }

  /// Remove all matrix operations from the sequence.
  void Clear(int expected_num_ops = kDefaultExpectedNumOps) {
    ops_.clear();
    ops_.reserve(expected_num_ops);
  }

  /// Operation is constant. For example, use to put something flat on the
  /// ground, with 'type' = kRotateAboutX and 'const_value' = pi/2.
  void AddOp(MatrixOperationType type, float const_value) {
    ops_.push_back(MatrixOperationInit(type, const_value));
  }

  /// Operation is driven by a one dimensional motivator. For example, you can
  /// control the face angle of a standing object with 'type' = kRotateAboutY
  /// and 'init' a curve specified by SplineInit.
  void AddOp(MatrixOperationType type, const MotivatorInit& init) {
    ops_.push_back(MatrixOperationInit(type, init));
  }

  /// Operation is driven by a one dimensional motivator, and initial value
  /// is specified.
  void AddOp(MatrixOperationType type, const MotivatorInit& init,
             float initial_value) {
    ops_.push_back(MatrixOperationInit(type, init, initial_value));
  }

  /// Operation is driven by a one dimensional motivator, which is initialized
  /// to traverse the key points specified in `target`.
  void AddOp(MatrixOperationType type, const MotivatorInit& init,
             const MotiveTarget1f& target) {
    ops_.push_back(MatrixOperationInit(type, init, target));
  }

  /// Operation is driven by a one dimensional motivator, which is initialized
  /// to follow the predefined curve specified in `spline`.
  void AddOp(MatrixOperationType type, const MotivatorInit& init,
             const CompactSpline& spline) {
    ops_.push_back(MatrixOperationInit(type, init, spline));
  }

  // Maximum duration of any of the splines.
  MotiveTime EndTime() const {
    MotiveTime end_time = 0;
    for (size_t i = 0; i < ops_.size(); ++i) {
      const MatrixOperationInit& op = ops_[i];
      if (op.union_type == MatrixOperationInit::kUnionSpline) {
        end_time =
            std::max(end_time, static_cast<MotiveTime>(op.spline->EndX()));
      }
    }
    return end_time;
  }

  const OpVector& ops() const { return ops_; }

 private:
  OpVector ops_;
};

class MatrixInit : public MotivatorInit {
 public:
  MOTIVE_INTERFACE();
  typedef std::vector<MatrixOperationInit> OpVector;

  explicit MatrixInit(const MatrixOpArray& ops)
      : MotivatorInit(kType),
        ops_(&ops),
        start_transform_(&mathfu::kAffineIdentity) {}
  MatrixInit(const MatrixOpArray& ops,
             const mathfu::AffineTransform& start_transform)
      : MotivatorInit(kType), ops_(&ops), start_transform_(&start_transform) {}

  const OpVector& ops() const { return ops_->ops(); }
  const mathfu::AffineTransform& start_transform() const {
    return *start_transform_;
  }

 private:
  /// Reference to the union of all operations that this matrix will be able
  /// to execute. Later calls to MotivatorMatrix4f::BlendToOps() must provide
  /// operations that are a subset of those in `ops_`.
  /// In `RigAnim`, these represent operations in the defining anim.
  const MatrixOpArray* ops_;

  /// Constant transform from which to start applying `ops_`.
  /// For example, `RigAnim`s use it to represent the constant transformation
  /// from a bone to its parent.
  const mathfu::AffineTransform* start_transform_;
};

class RigInit : public MotivatorInit {
 public:
  MOTIVE_INTERFACE();

  RigInit(const RigAnim& defining_anim,
          const mathfu::AffineTransform* bone_transforms,
          const BoneIndex* bone_parents, BoneIndex num_bones);
  const RigAnim& defining_anim() const { return *defining_anim_; }
  const mathfu::AffineTransform* bone_transforms() const {
    return bone_transforms_;
  }

  // Utility functions. Ensure that animations are compatible with rigs.
  static bool MatchesHierarchy(const BoneIndex* parents_a, BoneIndex len_a,
                               const BoneIndex* parents_b, BoneIndex len_b);
  static bool MatchesHierarchy(const RigAnim& anim, const BoneIndex* parents_b,
                               BoneIndex len_b);
  static bool MatchesHierarchy(const RigAnim& anim_a, const RigAnim& anim_b);

 private:
  /// Animation defining hierarchy and the union of matrix ops (across all
  /// animations).
  const RigAnim* defining_anim_;

  /// Array defining default pose. That is, the transformation from a bone to
  /// its parent. With just these, you can reconstruct the model in the pose
  /// it was exported in (i.e. its default pose).
  /// These transforms are used as the `start_transform_`s of the underlying
  /// `MatrixInit`s. All the matrix operations are applied from the origin of
  /// the bone they're animating.
  const mathfu::AffineTransform* bone_transforms_;
};

}  // namespace motive

#endif  // MOTIVE_INIT_H_
