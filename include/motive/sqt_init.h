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

#ifndef MOTIVE_SQT_INIT_H_
#define MOTIVE_SQT_INIT_H_

#include "motive/common.h"
#include "motive/matrix_op.h"

namespace motive {

/// @class SqtInit
/// @brief Initialize a MatrixMotivator4f to generate its matrix from an Sqt,
///        which is a matrix decomposed into translation, quaternion rotation,
///        and scale components.
///
/// Initialize a MatrixMotivator4f with these initialization parameters to
/// create a Motivator that generates a 4x4 matrix from up to 10 individual
/// animations representing the translation x/y/z, quaternion rotation x/y/z/w,
/// and scale x/y/z components of a transform. The individual components are
/// driven by one-dimensional Motivators and are combined using
/// mathfu::mat4::Transform().
///
/// This class uses MatrixOperationInit to make it easy to have a Rig animation
/// backed by either MatrixInits or SqtInits.
class SqtInit : public MotivatorInit {
 public:
  MOTIVE_INTERFACE();

  explicit SqtInit(const std::vector<MatrixOperationInit>& ops)
      : MotivatorInit(kType), ops_(&ops) {}

  const std::vector<MatrixOperationInit>& ops() const { return *ops_; }

 private:
  /// Reference to the component animations that drive this animation.
  const std::vector<MatrixOperationInit>* ops_;
};

}  // namespace motive

#endif  // MOTIVE_SQT_INIT_H_
