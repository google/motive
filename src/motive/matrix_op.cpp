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

#include "motive/matrix_op.h"

namespace motive {

static const mathfu::vec3 kDefaultOpsTranslation = mathfu::vec3(
    OperationDefaultValue(kTranslateX), OperationDefaultValue(kTranslateY),
    OperationDefaultValue(kTranslateZ));

mathfu::vec3 DefaultOpsTranslation() {
  return kDefaultOpsTranslation;
}

static const mathfu::quat kDefaultOpsQuaternion = mathfu::quat(
    OperationDefaultValue(kQuaternionW), OperationDefaultValue(kQuaternionX),
    OperationDefaultValue(kQuaternionY), OperationDefaultValue(kQuaternionZ));

mathfu::quat DefaultOpsQuaternion() {
  return kDefaultOpsQuaternion;
}

static const mathfu::vec3 kDefaultOpsScale = mathfu::vec3(
    OperationDefaultValue(kScaleX), OperationDefaultValue(kScaleY),
    OperationDefaultValue(kScaleZ));

mathfu::vec3 DefaultOpsScale() {
  return kDefaultOpsScale;
}

static const char* kMatrixOpNames[] = {
    "Invalid Matrix Operation",  // kInvalidMatrixOperation
    "Rotate About X",            // kRotateAboutX
    "Rotate About Y",            // kRotateAboutY
    "Rotate About Z",            // kRotateAboutZ
    "Translate X",               // kTranslateX
    "Translate Y",               // kTranslateY
    "Translate Z",               // kTranslateZ
    "Scale X",                   // kScaleX
    "Scale Y",                   // kScaleY
    "Scale Z",                   // kScaleZ
    "Scale Uniformly",           // kScaleUniformly
    "Quaternion W",              // kQuaternionW
    "Quaternion X",              // kQuaternionX
    "Quaternion Y",              // kQuaternionY
    "Quaternion Z",              // kQuaternionZ
};
static_assert(MOTIVE_ARRAY_SIZE(kMatrixOpNames) == kNumMatrixOperationTypes,
              "Adjust kMatrixOpNames to match enumeration");

const char* MatrixOpName(const motive::MatrixOperationType op) {
  assert(0 <= op && op < kNumMatrixOperationTypes);
  return kMatrixOpNames[op];
}

}  // namespace motive
