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

#ifndef MOTIVE_MATH_VECTOR_CONVERTER_H
#define MOTIVE_MATH_VECTOR_CONVERTER_H

#include "mathfu/glsl_mappings.h"

namespace fpl {

/// @class PassThroughVectorConverter
/// @brief No-op conversion from mathfu types to mathfu types.
/// External types are also mathfu in this converter. Create your own converter
/// if you'd like to use your own vector types in your Motivators'
/// external API.
class PassThroughVectorConverter {
 public:
  typedef mathfu::vec2 ExternalVector2;
  typedef mathfu::vec3 ExternalVector3;
  typedef mathfu::vec4 ExternalVector4;
  typedef mathfu::mat4 ExternalMatrix4;

  static ExternalVector2 To(const mathfu::vec2& v) { return v; }
  static ExternalVector3 To(const mathfu::vec3& v) { return v; }
  static ExternalVector4 To(const mathfu::vec4& v) { return v; }
  static const ExternalMatrix4& To(const mathfu::mat4& m) { return m; }
  static float To(const float v) { return v; }

  static const mathfu::vec2& From(const ExternalVector2& v) { return v; }
  static const mathfu::vec3& From(const ExternalVector3& v) { return v; }
  static const mathfu::vec4& From(const ExternalVector4& v) { return v; }
  static float From(const float v) { return v; }
};

// Map a dimension onto the external vector type.
// External types are specified by the VectorConverter, which is customizable.
template <class VectorConverter, int>
struct ExternalVectorT {
  typedef void type;
};
template <class VectorConverter>
struct ExternalVectorT<VectorConverter, 1> {
  typedef float type;
};
template <class VectorConverter>
struct ExternalVectorT<VectorConverter, 2> {
  typedef typename VectorConverter::ExternalVector2 type;
};
template <class VectorConverter>
struct ExternalVectorT<VectorConverter, 3> {
  typedef typename VectorConverter::ExternalVector3 type;
};
template <class VectorConverter>
struct ExternalVectorT<VectorConverter, 4> {
  typedef typename VectorConverter::ExternalVector4 type;
};

// Map a dimension onto the internal vector type.
// Internal types are from mathfu.
template <int>
struct InternalVectorT {
  typedef void type;
};
template <>
struct InternalVectorT<1> {
  typedef float type;
};
template <>
struct InternalVectorT<2> {
  typedef mathfu::vec2 type;
};
template <>
struct InternalVectorT<3> {
  typedef mathfu::vec3 type;
};
template <>
struct InternalVectorT<4> {
  typedef mathfu::vec4 type;
};

}  // namespace fpl

#endif  // MOTIVE_MATH_VECTOR_CONVERTER_H
