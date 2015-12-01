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

using motive::Angle;
using motive::Range;
using motive::kPi;
using motive::Graph2DPoints;
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
  // Define our vector types in the public section.
  typedef MyVec2 Vector2;
  typedef MyVec3 Vector3;
  typedef MyVec4 Vector4;
  typedef MyMatrix4 Matrix4;

  // Convert the data types to pointers. Note that we currently assume that
  // external matrices are in column-major format, the same as mathfu's.
  // A future change will store the matrices as row-major AffineTransforms
  // instead of 4x4 matrices.
  static float* ToPtr(float& f) { return &f; }
  static float* ToPtr(Vector2& v) { return &v.v[0]; }
  static float* ToPtr(Vector3& v) { return &v.v[0]; }
  static float* ToPtr(Vector4& v) { return &v.v[0]; }
  static float* ToPtr(Matrix4& m) { return &m.m[0][0]; }

  // This call results in a read of 'f' and then a write to the stack of
  // the return value. The optimizer will almost certainly eliminate this extra
  // read-write, since it does nothing.
  static float FromPtr(const float* f, float) { return *f; }
  static Vector2 FromPtr(const float* f, Vector2) { return Vector2(f); }
  static Vector3 FromPtr(const float* f, Vector3) { return Vector3(f); }
  static Vector4 FromPtr(const float* f, Vector4) { return Vector4(f); }
};
//! [Own Vector Converter]

//! [Own MatrixMotivator]
typedef MatrixMotivator4fTemplate<MyVectorConverter> MyMatrixMotivator4f;
//! [Own MatrixMotivator]


int main() {
  // Since we will be using the ‘smooth’ animation algorithm, we must register it
  // with the engine.
  motive::SplineInit::Register();
  motive::MatrixInit::Register();

  // The engine is the central place where animation data is stored and processed.
  motive::MotiveEngine engine;

  const motive::SplineInit angle_init(Range(-kPi, kPi), true);
  const motive::SplineInit position_init(Range(-10.0f, 10.0f), false);
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
