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

#ifndef MOTIVE_MATRIX_INIT_H_
#define MOTIVE_MATRIX_INIT_H_

#include "motive/common.h"
#include "motive/matrix_init.h"
#include "motive/matrix_op.h"
#include "motive/target.h"

namespace motive {

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
class MatrixInit : public MotivatorInit {
 public:
  MOTIVE_INTERFACE();

  explicit MatrixInit(const std::vector<MatrixOperationInit>& ops)
      : MotivatorInit(kType), ops_(&ops) {}

  const std::vector<MatrixOperationInit>& ops() const { return *ops_; }

 private:
  /// Reference to the union of all operations that this matrix will be able
  /// to execute. Later calls to MotivatorMatrix4f::BlendToOps() must provide
  /// operations that are a subset of those in `ops_`.
  /// In `RigAnim`, these represent operations in the defining anim.
  const std::vector<MatrixOperationInit>* ops_;
};

}  // namespace motive

#endif  // MOTIVE_MATRIX_INIT_H_
