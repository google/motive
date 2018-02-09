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
  return ScaleOp(op) ? 1.0f : 0.0f;
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
  MatrixOperation() {
    SetType(kInvalidMatrixOperation);
    SetAnimationType(kInvalidAnimationType);
  }

  MatrixOperation(const MatrixOperationInit& init, MotiveEngine* engine) {
    const AnimationType animation_type =
        init.init == nullptr ? kConstValueAnimation : kMotivatorAnimation;
    SetId(init.id);
    SetType(init.type);
    SetAnimationType(animation_type);

    // Manually construct the motivator in the union's memory buffer.
    if (animation_type == kMotivatorAnimation) {
      new (value_.motivator_memory) Motivator1f(*init.init, engine);
    }

    // Initialize the value. For defining animations, init.union_type will
    // be kUnionEmpty, so this will not set up any splines.
    BlendToOp(init, motive::SplinePlayback());
  }

  ~MatrixOperation() {
    // Manually call the Motivator destructor, since the union hides it.
    if (animation_type_ == kMotivatorAnimation) {
      Motivator().~Motivator1f();
    }
  }

  // Return the id identifying the operation between animations.
  MatrixOpId Id() const { return matrix_operation_id_; }

  // Return the type of operation we are animating.
  MatrixOperationType Type() const {
    return static_cast<MatrixOperationType>(matrix_operation_type_);
  }

  // Return the value we are animating.
  float Value() const {
    return animation_type_ == kMotivatorAnimation ? Motivator().Value()
                                                  : value_.const_value;
  }

  // Return true if we can blend to `op`.
  bool Blendable(const MatrixOperationInit& init) const {
    return matrix_operation_id_ == init.id;
  }

  // Return the child motivator if it is valid. Otherwise, return nullptr.
  Motivator1f* ValueMotivator() {
    return animation_type_ == kMotivatorAnimation ? &Motivator() : nullptr;
  }

  const Motivator1f* ValueMotivator() const {
    return animation_type_ == kMotivatorAnimation ? &Motivator() : nullptr;
  }

  void SetTarget1f(const MotiveTarget1f& t) { Motivator().SetTarget(t); }
  void SetValue1f(float value) {
    assert(animation_type_ == kConstValueAnimation &&
           (!RotateOp(Type()) || Angle::IsAngleInRange(value)));
    value_.const_value = value;
  }

  void BlendToOp(const MatrixOperationInit& init,
                 const motive::SplinePlayback& playback) {
    switch (animation_type_) {
      case kMotivatorAnimation: {
        Motivator1f& motivator = Motivator();

        // Initialize the state if required.
        switch (init.union_type) {
          case MatrixOperationInit::kUnionEmpty:
            break;

          case MatrixOperationInit::kUnionInitialValue:
            motivator.SetTarget(
                Target1f(init.initial_value, 0.0f,
                         static_cast<MotiveTime>(playback.blend_x)));
            break;

          case MatrixOperationInit::kUnionTarget:
            motivator.SetTarget(*init.target);
            break;

          case MatrixOperationInit::kUnionSpline:
            motivator.SetSpline(*init.spline, playback);
            break;

          default:
            assert(false);
        }
        break;
      }

      case kConstValueAnimation:
        // If this value is not driven by an motivator, it must have a constant
        // value.
        assert(init.union_type == MatrixOperationInit::kUnionInitialValue);

        // Record the const value into the union. There is no blending for
        // constant values.
        value_.const_value = init.initial_value;
        break;

      default:
        assert(false);
    }
  }

  void BlendToDefault(MotiveTime blend_time) {
    // Don't touch const value ones. Their default value is their const value.
    if (animation_type_ == kConstValueAnimation) return;
    assert(animation_type_ == kMotivatorAnimation);

    // Create spline that eases out to the default_value.
    Motivator1f& motivator = Motivator();
    const float default_value = OperationDefaultValue(Type());
    const MotiveTarget1f target =
        blend_time == 0 ? Current1f(default_value)
                        : Target1f(default_value, 0.0f, blend_time);
    motivator.SetTarget(target);
  }

  void SetPlaybackRate(float playback_rate) {
    if (animation_type_ == kConstValueAnimation) return;
    assert(animation_type_ == kMotivatorAnimation);
    Motivator().SetSplinePlaybackRate(playback_rate);
  }

  MotiveTime TimeRemaining() const {
    if (animation_type_ == kMotivatorAnimation) {
      // Return the time time to reach the target for the motivator.
      return Motivator().TargetTime();
    } else {
      // Constant animations are always at the "end" of their animation.
      return 0;
    }
  }

  // Execute the series of basic matrix operations in 'ops_'.
  // We break out the matrix into four column vectors to avoid matrix multiplies
  // (which are slow) in preference of operation-specific matrix math (which is
  // fast).
  static mathfu::mat4 CalculateResultMatrix(const MatrixOperation* ops,
                                            size_t num_ops) {
    // Start with the identity matrix.
    mathfu::vec4 c0 = mathfu::kAxisX4f;
    mathfu::vec4 c1 = mathfu::kAxisY4f;
    mathfu::vec4 c2 = mathfu::kAxisZ4f;
    mathfu::vec4 c3 = mathfu::kAxisW4f;

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

 private:
  enum AnimationType {
    kInvalidAnimationType,
    kMotivatorAnimation,
    kConstValueAnimation
  };

  // Disable copies so we don't have to worry about copying the Motivator1f in
  // the union.
  MatrixOperation(const MatrixOperation& rhs);
  MatrixOperation& operator=(const MatrixOperation& rhs);

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

  // Motivator1f has non-trivial constructors, destructors, and copy operators,
  // so we don't use it in the union. C++11 supports these kinds of unions,
  // but not all compilers (notably, Visual Studio) have complex union support
  // yet.
  union AnimatedValue {
    uint8_t motivator_memory[sizeof(Motivator1f)];
    float const_value;
  };

  void SetId(MatrixOpId id) {
    assert(id <= kMaxMatrixOpId);
    matrix_operation_id_ = id;
  }

  void SetType(MatrixOperationType type) {
    matrix_operation_type_ = static_cast<uint8_t>(type);
  }

  void SetAnimationType(AnimationType animation_type) {
    animation_type_ = static_cast<uint8_t>(animation_type);
  }

  Motivator1f& Motivator() {
    assert(animation_type_ == kMotivatorAnimation);
    return *reinterpret_cast<Motivator1f*>(value_.motivator_memory);
  }

  const Motivator1f& Motivator() const {
    assert(animation_type_ == kMotivatorAnimation);
    return *reinterpret_cast<const Motivator1f*>(value_.motivator_memory);
  }

  // Identify an operation so that it can be matched across different
  // animations, and thus blended.
  MatrixOpId matrix_operation_id_;

  // Enum MatrixOperationType compressed to 8-bits to save memory.
  // The matrix operation that we're performing.
  uint8_t matrix_operation_type_;

  // Enum AnimationType compressed to 8-bits to save memory.
  // The union parameter in 'value_' that is currently valid.
  uint8_t animation_type_;

  // The value being animated. Union because value can come from several
  // sources. The currently valid union member is specified by animation_type_.
  AnimatedValue value_;
};

}  // namespace motive

#endif  // MOTIVE_MATRIX_OP_H_
