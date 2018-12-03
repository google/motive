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

#ifndef MOTIVE_MATRIX_OP_H_
#define MOTIVE_MATRIX_OP_H_

#include "motive/engine.h"
#include "motive/math/angle.h"
#include "motive/math/compact_spline.h"
#include "motive/math/range.h"
#include "motive/vector_motivator.h"

namespace motive {

/// @typedef MatrixOpId
/// Identify an operation in an animation so that it can be blended with the
/// same operation in another animation. For example, an animation may have
/// three kTranslateX operations for a single matrix: one for translating to
/// the scale pivot, one for translating from the scale pivot, and one for the
/// final SQT translation. If another animation has no scale operations,
/// however, that other animation may have only the one SQT translation.
/// We need the MatrixOpId id so that we know how to match the SQT translations
/// when blending from one animation to the other.
typedef uint8_t MatrixOpId;
static const MatrixOpId kMaxMatrixOpId = 254;
static const MatrixOpId kInvalidMatrixOpId = 255;
static const float kEpsilon = 0.001f;

// Same as MatrixOperationTypeFb in schemas/anim.fbs.
/// Quaternion and RotateAbout ops cannot be present in the same animation.
/// If quaternion ops are present, there should only be at most one op of each
/// type and the list of operations should be processor by the
/// SqtMotiveProcessor instead of the MatrixMotiveProcessor. Quaternion
/// operations are included here (rather than creating a separate type) so that
/// RigAnims can easily be represented by either Matrices or Sqts without much
/// code change.
enum MatrixOperationType {
  kInvalidMatrixOperation,
  // RotateAbout ops only function in the MatrixMotiveProcessor.
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
  // Quaternion ops only function in the SqtMotiveProcessor.
  kQuaternionW,
  kQuaternionX,
  kQuaternionY,
  kQuaternionZ,
  // Not found in anim.fbs, included for convenience.
  kNumMatrixOperationTypes
};

/// Returns true if two values are nearly equal based on a given precision.
inline bool NearlyEqual(float a, float b, float precision) {
  const float diff = a - b;
  return -precision <= diff && diff <= precision;
}

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

/// Returns true if the operation is a quaternion component.
inline bool QuaternionOp(MatrixOperationType op) {
  return kQuaternionW <= op && op <= kQuaternionZ;
}

/// Returns a vector  containing the default operation values.
mathfu::vec3 DefaultOpsTranslation();

/// Returns a quaternion containing the default quaternion values.
mathfu::quat DefaultOpsQuaternion();

/// Returns a vector containing the default scale values.
mathfu::vec3 DefaultOpsScale();

/// Returns the default value of the operation. That is, the value of the
/// operation that does nothing to the transformation. Any operation that
/// constantly returns the default value can be removed.
inline float OperationDefaultValue(MatrixOperationType op) {
  return (ScaleOp(op) || op == kQuaternionW) ? 1.0f : 0.0f;
}



/// Returns the range of the matrix operation's spline. Most ranges are just
/// the extents of the splines, but rotations we want to normalize within
/// +-pi before blending to another curve.
inline Range RangeOfOp(MatrixOperationType op) {
  return RotateOp(op) ? kAngleRange : kInvalidRange;
}

/// Return a string with the operation name. Used for debugging.
const char* MatrixOpName(const MatrixOperationType op);

/// @class MatrixOperationInit
/// @brief Init params for a basic operation on a matrix.
struct MatrixOperationInit {
  /// Enum to indicate which value in the union is valid: none, `initial_value`,
  // `target`, or, `spline`.
  enum UnionType {
    kUnionEmpty,
    kUnionInitialValue,
    kUnionTarget,
    kUnionSpline
  };

  /// Matrix operation never changes. Always use 'const_value'.
  MatrixOperationInit(MatrixOpId id, MatrixOperationType type,
                      float const_value)
      : init(nullptr),
        id(id),
        type(type),
        union_type(kUnionInitialValue),
        initial_value(const_value) {}

  /// Matrix operation is driven by Motivator defined by 'init'.
  MatrixOperationInit(MatrixOpId id, MatrixOperationType type,
                      const MotivatorInit& init)
      : init(&init), id(id), type(type), union_type(kUnionEmpty) {}

  /// Matrix operation is driven by Motivator defined by 'init'. Specify initial
  /// value as well.
  MatrixOperationInit(MatrixOpId id, MatrixOperationType type,
                      const MotivatorInit& init, float initial_value)
      : init(&init),
        id(id),
        type(type),
        union_type(kUnionInitialValue),
        initial_value(initial_value) {}

  MatrixOperationInit(MatrixOpId id, MatrixOperationType type,
                      const MotivatorInit& init, const MotiveTarget1f& target)
      : init(&init),
        id(id),
        type(type),
        union_type(kUnionTarget),
        target(&target) {}

  MatrixOperationInit(MatrixOpId id, MatrixOperationType type,
                      const MotivatorInit& init, const CompactSpline& spline)
      : init(&init),
        id(id),
        type(type),
        union_type(kUnionSpline),
        spline(&spline) {}

  float StartValue() const {
    switch (union_type) {
      case kUnionEmpty:
        return OperationDefaultValue(type);
      case kUnionInitialValue:
        return initial_value;
      case kUnionTarget:
        return target->Node(0).value;
      case kUnionSpline:
        return spline->StartY();
    }
  }

  const MotivatorInit* init;
  MatrixOpId id;
  MatrixOperationType type;
  UnionType union_type;
  union {
    float initial_value;
    const MotiveTarget1f* target;
    const CompactSpline* spline;
  };
};

// Runtime structure to hold one operation and the input value of that
// operation. Kept as small as possible to conserve memory, since every
// matrix will be constructed by a series of these.
class MatrixOperation {
 public:
  MatrixOperation() { SetType(kInvalidMatrixOperation); }

  MatrixOperation(const MatrixOperationInit& init, MotiveEngine* engine)
      : MatrixOperation(init, motive::SplinePlayback(), engine) {}

  MatrixOperation(const MatrixOperationInit& init,
                  const motive::SplinePlayback& playback,
                  MotiveEngine* engine) {
    SetId(init.id);
    SetType(init.type);

    const_value_ = OperationDefaultValue(Type());

    // Only create a Motivator if an initializer struct is present.
    if (init.init != nullptr) {
      motivator_ = Motivator1f(*init.init, engine);
      // Initialize the spline so BlendToOp() can safely check Value() and
      // Velocity() for uninitialized splines.
      motivator_.SetTarget(Current1f(const_value_));
    }

    // Initialize the value. For defining animations, init.union_type will
    // be kUnionEmpty, so this will not set up any splines.
    BlendToOp(init, playback, engine);
  }

  MatrixOperation(MatrixOperation&& rhs) noexcept { *this = std::move(rhs); }

  MatrixOperation& operator=(MatrixOperation&& rhs) noexcept {
    if (this != &rhs) {
      SetId(rhs.Id());
      SetType(rhs.Type());
      motivator_ = std::move(rhs.motivator_);
      const_value_ = rhs.const_value_;
      rhs.SetType(kInvalidMatrixOperation);
    }
    return *this;
  }

  MatrixOperation(const MatrixOperation& rhs) { *this = rhs; }

  MatrixOperation& operator=(const MatrixOperation& rhs) {
    SetId(rhs.Id());
    SetType(rhs.Type());
    motivator_.CloneFrom(
        reinterpret_cast<const Motivator*>(rhs.ValueMotivator()));
    const_value_ = rhs.const_value_;
    return *this;
  }

  // Return the id identifying the operation between animations.
  MatrixOpId Id() const { return matrix_operation_id_; }

  // Return the type of operation we are animating.
  MatrixOperationType Type() const {
    return static_cast<MatrixOperationType>(matrix_operation_type_);
  }

  // Return the value we are animating.
  float Value() const {
    return motivator_.Valid() ? motivator_.Value() : const_value_;
  }

  // Return the current velocity of the animated value.
  float Velocity() const {
    return motivator_.Valid() ? motivator_.Velocity() : 0.f;
  }

  // Return true if we can blend to `op`.
  bool Blendable(const MatrixOperationInit& init) const {
    return matrix_operation_id_ == init.id;
  }

  // Return the child motivator if it is valid. Otherwise, return nullptr.
  Motivator1f* ValueMotivator() {
    return motivator_.Valid() ? &motivator_ : nullptr;
  }

  const Motivator1f* ValueMotivator() const {
    return motivator_.Valid() ? &motivator_ : nullptr;
  }

  void SetTarget1f(const MotiveTarget1f& t) {
    assert(motivator_.Valid());
    motivator_.SetTarget(t);
  }

  void SetValue1f(float value) {
    assert(!motivator_.Valid() &&
           (!RotateOp(Type()) || Angle::IsAngleInRange(value)));
    const_value_ = value;
  }

  void BlendToOp(const MatrixOperationInit& init,
                 const motive::SplinePlayback& playback, MotiveEngine* engine) {
    switch (init.union_type) {
      case MatrixOperationInit::kUnionEmpty:
        break;

      case MatrixOperationInit::kUnionInitialValue:
        // Blending constant to constant happens immediately. Blending a spline
        // already at the correct constant and with 0 velocity transforms this
        // op into a constant. In other cases, set a new target.
        if (!motivator_.Valid()) {
          const_value_ = init.initial_value;
        } else if (NearlyEqual(motivator_.Value(), init.initial_value,
                               kEpsilon) &&
                   NearlyEqual(motivator_.Velocity(), 0.f, kEpsilon)) {
          motivator_.Invalidate();
          const_value_ = init.initial_value;
        } else {
          motivator_.SetTarget(
              Target1f(init.initial_value, 0.0f,
                       static_cast<MotiveTime>(playback.blend_x)));
        }
        break;

      case MatrixOperationInit::kUnionTarget:
        // Re-initialize the Motivator at the current constant if necessary.
        if (!motivator_.Valid()) {
          motivator_ = Motivator1f(*init.init, engine);
          motivator_.SetTarget(Current1f(const_value_));
        }
        motivator_.SetTarget(*init.target);
        break;

      case MatrixOperationInit::kUnionSpline:
        // Re-initialize the Motivator at the current constant if necessary.
        if (!motivator_.Valid()) {
          motivator_ = Motivator1f(*init.init, engine);
          motivator_.SetTarget(Current1f(const_value_));
        }
        motivator_.SetSpline(*init.spline, playback);
        break;

      default:
        assert(false);
    }
  }

  void BlendToDefault(MotiveTime blend_time) {
    const float default_value = OperationDefaultValue(Type());
    if (!motivator_.Valid()) {
      // Snap to the default_value.
      const_value_ = default_value;
    } else {
      // Create spline that eases out to the default_value.
      const MotiveTarget1f target =
          blend_time == 0 ? Current1f(default_value)
                          : Target1f(default_value, 0.0f, blend_time);
      motivator_.SetTarget(target);
    }
  }

  void SetPlaybackRate(float playback_rate) {
    if (!motivator_.Valid()) return;
    motivator_.SetSplinePlaybackRate(playback_rate);
  }

  void SetRepeating(bool repeat) {
    if (!motivator_.Valid()) return;
    motivator_.SetSplineRepeating(repeat);
  }

  MotiveTime TimeRemaining() const {
    if (motivator_.Valid()) {
      // Return the time time to reach the target for the motivator.
      return motivator_.TargetTime();
    } else {
      // Constant animations are always at the "end" of their animation.
      return 0;
    }
  }

  // Execute the series of basic matrix operations in 'ops'.
  // We break out the matrix into four column vectors to avoid matrix multiplies
  // (which are slow) in preference of operation-specific matrix math (which is
  // fast).
  static mathfu::mat4 CalculateResultMatrix(const MatrixOperation* ops,
                                            size_t num_ops) {
    return CalculateResultMatrix(ops, num_ops, nullptr);
  }

  // Execute the series of basic matrix operations in 'ops' and returns the
  // scale of the matrix in `out_scale`.
  static mathfu::mat4 CalculateResultMatrix(const MatrixOperation* ops,
                                            size_t num_ops,
                                            mathfu::vec3* out_scale) {
    // Start with the identity matrix.
    mathfu::vec4 c0 = mathfu::kAxisX4f;
    mathfu::vec4 c1 = mathfu::kAxisY4f;
    mathfu::vec4 c2 = mathfu::kAxisZ4f;
    mathfu::vec4 c3 = mathfu::kAxisW4f;

    // Separately keep track of the scale.
    mathfu::vec3 scale = mathfu::kOnes3f;

    for (size_t i = 0; i < num_ops; ++i) {
      const MatrixOperation& op = ops[i];
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
          scale.x *= value;
          break;

        case kScaleY:
          c1 *= value;
          scale.y *= value;
          break;

        case kScaleZ:
          c2 *= value;
          scale.z *= value;
          break;

        case kScaleUniformly:
          c0 *= value;
          c1 *= value;
          c2 *= value;
          scale *= value;
          break;

        default:
          // All other operations, including quaternions, are not supported.
          assert(false);
      }
    }
    if (out_scale) {
      *out_scale = scale;
    }
    return mathfu::mat4(c0, c1, c2, c3);
  }

 private:
  // Perform a matrix rotation about
  static inline void RotateAboutAxis(const float angle, mathfu::vec4* column0,
                                     mathfu::vec4* column1) {
    // TODO OPT: call platform-specific function to calculate both sin and cos
    // simultaneously.
    const float s = sinf(angle);
    const float c = cosf(angle);
    const mathfu::vec4 c0 = *column0;
    const mathfu::vec4 c1 = *column1;
    *column0 = c * c0 + s * c1;
    *column1 = c * c1 - s * c0;
  }

  void SetId(MatrixOpId id) {
    assert(id <= kMaxMatrixOpId);
    matrix_operation_id_ = id;
  }

  void SetType(MatrixOperationType type) {
    matrix_operation_type_ = static_cast<uint8_t>(type);
  }

  // Identify an operation so that it can be matched across different
  // animations, and thus blended.
  MatrixOpId matrix_operation_id_;

  // Enum MatrixOperationType compressed to 8-bits to save memory.
  // The matrix operation that we're performing.
  uint8_t matrix_operation_type_;

  // Motivator for the value being animated.
  Motivator1f motivator_;

  // Constant value override when `motivator_` is invalid.
  float const_value_;
};

}  // namespace motive

#endif  // MOTIVE_MATRIX_OP_H_
