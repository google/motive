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

#ifndef MOTIVE_MATRIX_MOTIVATOR_H_
#define MOTIVE_MATRIX_MOTIVATOR_H_

#include "motive/matrix_init.h"
#include "motive/matrix_op.h"
#include "motive/matrix_processor.h"
#include "motive/vector_motivator.h"

namespace motive {

/// @class MatrixMotivator4fTemplate
/// @brief Drive a 4x4 float matrix from a series of basic transformations.
///
/// The underlying basic transformations can be animated with
/// SetChildTarget1f(), and set to fixed values with SetChildValue1f() and
/// SetChildValue3f().
///
/// Internally, we use mathfu::mat4 as our matrix type and mathfu::vec3 as
/// our vector type, but external we allow any matrix type to be specified
/// via the VectorConverter template parameter.
///
template <class VectorConverter>
class MatrixMotivator4fTemplate : public Motivator {
 public:
  typedef VectorConverter C;
  typedef typename VectorConverter::Matrix4 Mat4;
  typedef typename VectorConverter::Vector3 Vec3;
  typedef typename VectorConverter::Vector4 Vec4;
  typedef MotivatorXfTemplate<C, 1> Mot1f;

  MatrixMotivator4fTemplate() {}
  MatrixMotivator4fTemplate(const MotivatorInit& init, MotiveEngine* engine)
      : Motivator(init, engine, 1) {}

  /// Initialize to the type specified by `init`.
  void Initialize(const MotivatorInit& init, MotiveEngine* engine) {
    InitializeWithDimension(init, engine, 1);
  }

  /// Return the current value of the Motivator. The processor returns a
  /// vector-aligned matrix, so the cast should be valid for any user-defined
  /// matrix type.
  const Mat4& Value() const {
    // TODO: Return by value here. Necessary for matrix formats that are not
    //       byte-wise compatible. For example, row-major matrices.
    return reinterpret_cast<const Mat4&>(Processor().Value(index_));
  }

  /// Return the current value of the Motivator decomposed into a translation,
  /// rotation quaternion, and scale. The quaternion is packed into a Vec4 with
  /// the vector component in rotation.xyz and vector component in rotation.w.
  void Value(Vec3* translation, Vec4* rotation, Vec3* scale) const {
    Processor().Value(index_, translation, rotation, scale);
  }

  /// Return the translation component of the matrix.
  /// The matrix is a 3D affine transform, so the translation component is the
  /// fourth column.
  Vec3 Position() const {
    const mathfu::vec3 position =
        Processor().Value(index_).TranslationVector3D();
    return C::FromPtr(&position[0], Vec3());
  }

  /// Return the time remaining in the current spline animation.
  /// Time units are defined by the user.
  MotiveTime TimeRemaining() const {
    return Valid() ? Processor().TimeRemaining(index_) : 0;
  }

  /// Query the number of matrix operations. This equals the number of
  /// operations in the `init` initializer.
  int NumChildren() const { return Processor().NumChildren(index_); }

  /// Return the current value of the `child_index`th basic transform that
  /// drives this matrix.
  /// @param child_index The index into MatrixInit::ops(). The ops() array
  ///                    is a series of basic transformation operations that
  ///                    compose this matrix. Each basic transformation has
  ///                    a current value. We gather this value here.
  float ChildValue1f(MotiveChildIndex child_index) const {
    float r;
    Processor().ChildValues(index_, child_index, 1, &r);
    return r;
  }

  /// Returns the current values of the basic transforms at indices
  /// (child_index, child_index + 1, child_index + 2).
  /// Useful when you drive all the (x,y,z) components of a translation, scale,
  /// or rotation.
  /// @param child_index The first index into MatrixInit::ops(). The value at
  ///                    this index is returned in the x component of Vec3.
  ///                    y gets child_index + 1's value, and
  ///                    z gets child_index + 2's value.
  Vec3 ChildValue3f(MotiveChildIndex child_index) const {
    Vec3 r;
    Processor().ChildValues(index_, child_index, 3, C::ToPtr(r));
    return r;
  }

  /// Returns the Motivator1f that's driving this child, if it's driven by
  /// a Motivator1f. Otherwise, returns nullptr.
  const Mot1f* ChildMotivator1f(MotiveChildIndex child_index) const {
    return static_cast<const Mot1f*>(
        Processor().ChildMotivator1f(index_, child_index));
  }

  /// Set the target the 'child_index'th basic transform.
  /// Each basic transform can be driven by a child motivator.
  /// This call lets us control those child motivators.
  /// @param child_index The index into the MatrixInit::ops() that was passed
  ///                    into Initialize(). This operation must have been
  ///                    initialized with a MotivatorInit, not a constant value.
  /// @param t The target values for the basic transform to animate towards.
  ///          Also, optionally, the current value for it to jump to.
  void SetChildTarget1f(MotiveChildIndex child_index, const MotiveTarget1f& t) {
    Processor().SetChildTarget1f(index_, child_index, t);
  }

  /// Set the constant value of a child. Each basic matrix transformation
  /// can be driven by a constant value instead of a Motivator.
  /// This call lets us set those constant values.
  /// @param child_index The index into the MatrixInit::ops() that was passed
  ///                    into Initialize(). This operation must have been
  ///                    initialized with a constant value, not a MotivatorInit.
  /// @param value The new constant value of this operation.
  void SetChildValue1f(MotiveChildIndex child_index, float value) {
    Processor().SetChildValues(index_, child_index, 1, &value);
  }

  /// Set the constant values of the basic transforms at indices
  /// (child_index, child_index + 1, child_index + 2).
  /// @param child_index The first index into MatrixInit::ops(). The constant
  ///                    value at this index is set to the x component of
  ///                    `value`.
  ///                    child_index + 1's constant is set to value.y, and
  ///                    child_index + 2's constant is set to value.z.
  /// @param value The new constant value for this 3-dimensional operation.
  void SetChildValue3f(MotiveChildIndex child_index, const Vec3& value) {
    Processor().SetChildValues(index_, child_index, 3, C::ToPtr(value));
  }

  /// Match existing MatrixOps with those in `ops` and smoothly transition
  /// to the new parameters in `ops`.
  void BlendToOps(const std::vector<MatrixOperationInit>& ops,
                  const SplinePlayback& playback) {
    Processor().BlendToOps(index_, ops, playback);
  }

  void SetPlaybackRate(float playback_rate) {
    if (Valid()) {
      Processor().SetPlaybackRate(index_, playback_rate);
    }
  }

  void SetRepeating(bool repeat) {
    if (Valid()) {
      Processor().SetRepeating(index_, repeat);
    }
  }

 private:
  MatrixProcessor4f& Processor() {
    return *static_cast<MatrixProcessor4f*>(processor_);
  }
  const MatrixProcessor4f& Processor() const {
    return *static_cast<const MatrixProcessor4f*>(processor_);
  }
};


// These Motivator types use mathfu in their external API.
typedef MatrixMotivator4fTemplate<MathFuVectorConverter> MatrixMotivator4f;

}  // namespace motive

#endif  // MOTIVE_MATRIX_MOTIVATOR_H_
