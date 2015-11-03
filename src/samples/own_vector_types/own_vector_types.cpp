// Copyright 2015 Google Inc. All rights reserved.
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

#include "motive/engine.h"
#include "motive/init.h"
#include "motive/math/angle.h"
#include "motive/math/curve.h"

using fpl::Angle;
using fpl::Range;
using fpl::kPi;
using fpl::Graph2DPoints;
using mathfu::vec2;
using motive::MatrixMotivator4fTemplate;


//! [Own Vector Types]
struct MyMatrix4 {
  float m[4][4];
};

template <int kDimension>
struct MyVecTemplate {
  float v[kDimension];

  explicit MyVecTemplate(const float* pointer) {
    memcpy(v, pointer, sizeof(v));
  }
};

typedef MyVecTemplate<2> MyVec2;
typedef MyVecTemplate<3> MyVec3;
typedef MyVecTemplate<4> MyVec4;
//! [Own Vector Types]

//! [Own Vector Converter]
class MyVectorConverter {
 public:
  // MatrixMotivator4fTemplate needs these External.
  typedef MyMatrix4 ExternalMatrix4;
  typedef MyVec2 ExternalVector2;
  typedef MyVec3 ExternalVector3;
  typedef MyVec4 ExternalVector4;

  // Casting from mathfu::mat4 is ok. The alignment restrictions on mathfu::mat4
  // (16-bytes) are stricter than for MyMatrix4 (4-bytes). Also, strict aliasing
  // is not a problem here since 'm' was written on the other side of a virtual
  // function call (see "Strict Aliasing" discussion in documentation).
  static const MyMatrix4& To(const mathfu::mat4& m) {
    return reinterpret_cast<const MyMatrix4&>(m);
  }

  // This call results in a read of 'v' and then a write to the stack of
  // MyVec3. The optimizer will almost certainly eliminate this extra
  // read-write, since it does nothing.
  static MyVec2 To(const mathfu::vec2& v) { return MyVec2(&v[0]); }
  static MyVec3 To(const mathfu::vec3& v) { return MyVec3(&v[0]); }
  static MyVec4 To(const mathfu::vec4& v) { return MyVec4(&v[0]); }

  // Here we have to call the constructor for matfu::vec3, because the alignment
  // restrictions are more strict for mathfu types (16-bytes) than for MyVec3
  // (4-bytes). The optimizer *may not* be able to eliminate this read-write
  // since 'v' might arrive misaligned. This function may result in overhead,
  // therefore.
  static const mathfu::vec2 From(const MyVec2& v) {
    return mathfu::vec2(&v.v[0]);
  }
  static const mathfu::vec3 From(const MyVec3& v) {
    return mathfu::vec3(&v.v[0]);
  }
  static const mathfu::vec4 From(const MyVec4& v) {
    return mathfu::vec4(&v.v[0]);
  }
};
//! [Own Vector Converter]

//! [Own MatrixMotivator]
typedef MatrixMotivator4fTemplate<MyVectorConverter> MyMatrixMotivator4f;
//! [Own MatrixMotivator]


int main() {
  // Since we will be using the ‘smooth’ animation algorithm, we must register it
  // with the engine.
  motive::SmoothInit::Register();
  motive::MatrixInit::Register();

  // The engine is the central place where animation data is stored and processed.
  motive::MotiveEngine engine;

  const motive::SmoothInit angle_init(Range(-kPi, kPi), true);
  const motive::SmoothInit position_init(Range(-10.0f, 10.0f), false);
  const motive::MotiveTime target_time = 1000;
  const motive::MotiveTarget1f angle_target =
      motive::CurrentToTarget1f(0, 0, kPi, 0, target_time);
  const motive::MotiveTarget1f position_target =
      motive::CurrentToTarget1f(0, 0.2f, 10, 0, target_time);

  motive::MatrixOpArray ops(2);
  ops.AddOp(motive::kRotateAboutY, angle_init, angle_target);
  ops.AddOp(motive::kTranslateZ, position_init, position_target);
  const motive::MatrixInit matrix_init(ops);

  MyMatrixMotivator4f matrix(matrix_init, &engine);

  const motive::MotiveTime delta_time = 250;
  for (motive::MotiveTime t = 0; t <= target_time; t += delta_time) {
    // Print matrix. It is column first.
    const MyMatrix4& my_matrix = matrix.Value();
    const float* m = &my_matrix.m[0][0];
    printf("\n(%f %f %f %f)\n(%f %f %f %f)\n(%f %f %f %f)\n(%f %f %f %f)\n",
           m[0], m[4], m[8], m[12], m[1], m[5], m[9], m[13],
           m[2], m[6], m[10], m[14], m[3], m[7], m[11], m[15]);

    // All Motivators that have been registered with 'engine' are updated in
    // one call. This allows us to more easily take advantage of SIMD or
    // multi-threading opportunities.
    engine.AdvanceFrame(delta_time);
  }
  return 0;
}
