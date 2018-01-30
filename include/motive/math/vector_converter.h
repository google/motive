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

#ifndef MOTIVE_MATH_VECTOR_CONVERTER_H_
#define MOTIVE_MATH_VECTOR_CONVERTER_H_

#include "mathfu/glsl_mappings.h"

namespace motive {

/// @class MathFuVectorConverter
/// @brief Convert mathfu types to float pointers.
///
/// Create your own converter if you'd like to use your own vector types in
/// your Motivators' external API.
class MathFuVectorConverter {
 public:
  typedef mathfu::vec2 Vector2;
  typedef mathfu::vec3 Vector3;
  typedef mathfu::vec4 Vector4;
  typedef mathfu::mat4 Matrix4;

  static float* ToPtr(float& f) { return &f; }
  static float* ToPtr(Vector2& v) { return &v[0]; }
  static float* ToPtr(Vector3& v) { return &v[0]; }
  static float* ToPtr(Vector4& v) { return &v[0]; }
  static float* ToPtr(Matrix4& m) { return &m(0); }

  static const float* ToPtr(const float& f) { return &f; }
  static const float* ToPtr(const Vector2& v) { return &v[0]; }
  static const float* ToPtr(const Vector3& v) { return &v[0]; }
  static const float* ToPtr(const Vector4& v) { return &v[0]; }
  static const float* ToPtr(const Matrix4& m) { return &m(0); }

  static float FromPtr(const float* f, float) { return *f; }
  static Vector2 FromPtr(const float* f, const Vector2&) { return Vector2(f); }
  static Vector3 FromPtr(const float* f, const Vector3&) { return Vector3(f); }
  static Vector4 FromPtr(const float* f, const Vector4&) { return Vector4(f); }
  static Matrix4 FromPtr(const float* f, const Matrix4&) { return Matrix4(f); }
};

// Map a dimension onto the external vector type.
// External types are specified by the VectorConverter, which is customizable.
template <class VectorConverter, int>
struct VectorT {
  typedef void type;
};
template <class VectorConverter>
struct VectorT<VectorConverter, 1> {
  typedef float type;
};
template <class VectorConverter>
struct VectorT<VectorConverter, 2> {
  typedef typename VectorConverter::Vector2 type;
};
template <class VectorConverter>
struct VectorT<VectorConverter, 3> {
  typedef typename VectorConverter::Vector3 type;
};
template <class VectorConverter>
struct VectorT<VectorConverter, 4> {
  typedef typename VectorConverter::Vector4 type;
};

}  // namespace motive

#endif  // MOTIVE_MATH_VECTOR_CONVERTER_H_
