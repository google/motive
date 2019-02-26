// Copyright 2019 Google Inc. All rights reserved.
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

#include "motive/util/keyframe_converter.h"

#include <vector>

namespace motive {

namespace {

/// Converts |time| to Motive's millisecond time unit using |ms_per_time_unit|
/// as the conversion factor.
float ConvertToMilliseconds(float time, float ms_per_time_unit) {
  return time * ms_per_time_unit;
}

/// Determines the index of a float* representing a quaternion to access for
/// a particular |type| and |order|.
int IndexForQuaternionComponent(MatrixOperationType type,
                                QuaternionOrder order) {
  if (order == kOrderWXYZ) {
    return static_cast<int>(type - kQuaternionW);
  } else if (type == kQuaternionW) {
    return 3;
  } else {
    return static_cast<int>(type - kQuaternionX);
  }
}

}  // namespace

void AddVector3Constants(MatrixAnim* anim, MatrixOperationType base_type,
                         MatrixOpId base_id, const float* vector) {
  std::vector<MatrixOperationInit>& ops = anim->ops();
  for (int i = 0; i < 3; ++i) {
    const MatrixOperationType type =
        static_cast<MatrixOperationType>(base_type + i);
    const float value = vector[i];
    if (!IsOperationDefaultValue(type, value)) {
      ops.emplace_back(base_id + i, type, value);
    }
  }
}

void AddQuaternionConstants(MatrixAnim* anim, MatrixOpId base_id,
                            const float* quat, QuaternionOrder order) {
  std::vector<MatrixOperationInit>& ops = anim->ops();
  for (int i = 0; i < 4; ++i) {
    const MatrixOperationType type =
        static_cast<MatrixOperationType>(kQuaternionW + i);
    const int spline_index = IndexForQuaternionComponent(type, order);
    const float value = quat[spline_index];
    if (!IsOperationDefaultValue(type, value)) {
      ops.emplace_back(base_id + i, type, value);
    }
  }
}

void AddVector3Curves(MatrixAnim* anim, MatrixAnim::Spline* splines,
                      MatrixOperationType base_type, MatrixOpId base_id,
                      const KeyframeData& data) {
  std::vector<MatrixOperationInit>& ops = anim->ops();

  // Create a list of uncompressed nodes per channel.
  std::vector<UncompressedNode> nodes[3] = {
      std::vector<UncompressedNode>(data.count),
      std::vector<UncompressedNode>(data.count),
      std::vector<UncompressedNode>(data.count),
  };

  // Traverse the data in sample order and populate the nodes for each channel.
  for (int i = 0; i < data.count; ++i) {
    const int base_i = i * 3;
    const float time =
        ConvertToMilliseconds(data.times[i], data.ms_per_time_unit);
    for (int j = 0; j < 3; ++j) {
      auto& node = nodes[j][i];
      node.x = time;
      node.y = data.values[base_i + j];
      // TODO(b/124466599): generate derivatives based on interpolation type.
      node.derivative = 0.f;
    }
  }

  // Create splines and operations for each curve.
  for (int i = 0; i < 3; ++i) {
    splines[i].spline =
        CompactSpline::CreateFromNodes(nodes[i].data(), nodes[i].size());
    const MatrixOperationType type =
        static_cast<MatrixOperationType>(base_type + i);
    ops.emplace_back(base_id + i, type, splines[i].init, *splines[i].spline);
  }
}

void AddQuaternionCurves(MatrixAnim* anim, MatrixAnim::Spline* splines,
                         MatrixOpId base_id, QuaternionOrder order,
                         const KeyframeData& data) {
  std::vector<MatrixOperationInit>& ops = anim->ops();

  // Create a list of uncompressed nodes per channel.
  std::vector<UncompressedNode> nodes[4] = {
      std::vector<UncompressedNode>(data.count),
      std::vector<UncompressedNode>(data.count),
      std::vector<UncompressedNode>(data.count),
      std::vector<UncompressedNode>(data.count),
  };

  // Traverse the data in sample order and populate the nodes for each channel.
  for (int i = 0; i < data.count; ++i) {
    const int base_i = i * 4;
    const float time =
        ConvertToMilliseconds(data.times[i], data.ms_per_time_unit);
    for (int j = 0; j < 4; ++j) {
      auto& node = nodes[j][i];
      node.x = time;
      node.y = data.values[base_i + j];
      // TODO(b/124466599): generate derivatives based on interpolation type.
      node.derivative = 0.f;
    }
  }

  // Create splines and operations for each curve.
  for (int i = 0; i < 4; ++i) {
    const MatrixOperationType type =
        static_cast<MatrixOperationType>(kQuaternionW + i);
    const int spline_index = IndexForQuaternionComponent(type, order);

    // This function allocates a new spline from the heap. MatrixAnim's
    // destructor will destroy it with CompactSpline::Destroy().
    splines[spline_index].spline = CompactSpline::CreateFromNodes(
        nodes[spline_index].data(), nodes[spline_index].size());
    ops.emplace_back(base_id + i, type, splines[spline_index].init,
                     *splines[spline_index].spline);
  }
}

}  // namespace motive
