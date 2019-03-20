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

/// Fetches three values from |data| corresponding to the index |frame| and
/// places them into |output|. |previous| is ignored and supplied for template
/// convenience.
void GetVectorValue(const KeyframeData& data, size_t frame, float* output,
                    const float* previous) {
  const size_t frame_start = frame * 3;
  output[0] = data.values[frame_start];
  output[1] = data.values[frame_start + 1];
  output[2] = data.values[frame_start + 2];
}

/// Fetches four values from |data| corresponding to the index |frame| and
/// places them into |output|. |previous| should contain the previous frame's
/// values.
///
/// Quaternion interpolation is often done with SLERP, which ensures that
/// intermediate values are "sane". However, Motive must do linear quaternion
/// interpolation with NLERP since it represents each quaternion as a separate
/// spline. Interpolating between q and -q, which represent the same
/// orientation, can have odd results with NLERP, so we ensure that consecutive
/// keyframes don't have any of these "quaternion flips".
///
/// Step data always has 0 derivatives and CubicSpline data provides its own
/// derivatives, so such flips are only relevant in Linear interpolation.
void GetQuaternionValue(const KeyframeData& data, size_t frame, float* output,
                        const float* previous) {
  const size_t frame_start = frame * 4;
  output[0] = data.values[frame_start];
  output[1] = data.values[frame_start + 1];
  output[2] = data.values[frame_start + 2];
  output[3] = data.values[frame_start + 3];

  // Check for quaternion flips between this frame and the last when processing
  // linear interpolation data.
  if (frame > 0 && data.interpolation_type == kLinear) {
    const float dotprod = output[0] * previous[0] + output[1] * previous[1] +
                          output[2] * previous[2] + output[3] * previous[3];
    if (dotprod < 0.f) {
      output[0] *= -1.f;
      output[1] *= -1.f;
      output[2] *= -1.f;
      output[3] *= -1.f;
    }
  }
}

template <typename GetValueFn>
void AddLinearKeyframeData(std::vector<UncompressedNode>* nodes_per_channel,
                           size_t channel_count, const KeyframeData& data,
                           GetValueFn value_fn) {
  // Resize the lists of nodes to the proper size. We can "fake" linear
  // interpolation by having two spline nodes at every keyframe: the first uses
  // the left tangent as it's derivative and the second uses the right tangent.
  const size_t num_nodes = data.count * 2;
  for (size_t i = 0; i < channel_count; ++i) {
    nodes_per_channel[i].resize(num_nodes);
  }

  // Traverse the data in sample order and populate the nodes for each channel.
  std::vector<float> values(channel_count);
  std::vector<float> previous_values(channel_count);
  for (size_t frame = 0; frame < data.count; ++frame) {
    const float time =
        ConvertToMilliseconds(data.times[frame], data.ms_per_time_unit);
    value_fn(data, frame, values.data(), previous_values.data());
    for (size_t channel = 0; channel < channel_count; ++channel) {
      const int left_node_index = frame * 2;
      const float value = values[channel];

      // Populate the left and right nodes with the same times and values.
      auto& left_node = nodes_per_channel[channel][left_node_index];
      left_node.x = time;
      left_node.y = value;

      auto& right_node = nodes_per_channel[channel][left_node_index + 1];
      right_node.x = time;
      right_node.y = value;

      // Compute this left node's derivative and the previous right node's
      // derivative as the tangent line between the two.
      if (left_node_index == 0) {
        // The first left node has a zero left derivative.
        left_node.derivative = 0.f;
      } else {
        auto& prev_right_node = nodes_per_channel[channel][left_node_index - 1];
        const float delta_time = time - prev_right_node.x;
        const float tangent =
            delta_time > 0.f ? (value - prev_right_node.y) / delta_time : 0.f;
        prev_right_node.derivative = tangent;
        left_node.derivative = tangent;

        // The last right node has a zero right derivative.
        if (frame == data.count - 1) {
          right_node.derivative = 0.f;
        }
      }
    }
    previous_values = values;
  }
}

template <typename GetValueFn>
void AddStepKeyframeData(std::vector<UncompressedNode>* nodes_per_channel,
                         size_t channel_count, const KeyframeData& data,
                         GetValueFn value_fn) {
  // Resize the lists of nodes to the proper size. We can "fake" step
  // interpolation by having two spline nodes at every keyframe: the first uses
  // the value of the previous keyframe and the second uses the current.
  const size_t num_nodes = data.count * 2;
  for (size_t i = 0; i < channel_count; ++i) {
    nodes_per_channel[i].resize(num_nodes);
  }

  // Traverse the data in sample order and populate the nodes for each channel.
  std::vector<float> values(channel_count);
  std::vector<float> previous_values(channel_count);
  for (size_t frame = 0; frame < data.count; ++frame) {
    const float time =
        ConvertToMilliseconds(data.times[frame], data.ms_per_time_unit);
    value_fn(data, frame, values.data(), previous_values.data());
    for (size_t channel = 0; channel < channel_count; ++channel) {
      const int left_node_index = frame * 2;
      const float value = values[channel];

      // Populate the left and right nodes with the same times and derivatives.
      // The right node always takes the current value.
      auto& left_node = nodes_per_channel[channel][left_node_index];
      left_node.x = time;
      left_node.derivative = 0.f;

      auto& right_node = nodes_per_channel[channel][left_node_index + 1];
      right_node.x = time;
      right_node.y = value;
      right_node.derivative = 0.f;

      // If this is the first keyframe, assign the left node the same value.
      // Otherwise, use the previous frame's value.
      if (left_node_index == 0) {
        left_node.y = value;
      } else {
        left_node.y = previous_values[channel];
      }
    }
    previous_values = values;
  }
}

template <typename GetValueFn>
void AddKeyframeData(std::vector<UncompressedNode>* nodes_per_channel,
                     size_t channel_count, const KeyframeData& data,
                     GetValueFn value_fn) {
  switch (data.interpolation_type) {
    case kLinear:
      AddLinearKeyframeData(nodes_per_channel, channel_count, data, value_fn);
      break;
    case kStep:
      AddStepKeyframeData(nodes_per_channel, channel_count, data, value_fn);
      break;
    case kCubicSpline:
      // TODO(b/124466599): support cubicspline interpolation.
      break;
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
  // Create a list of uncompressed nodes per channel and populate them.
  std::vector<UncompressedNode> nodes[3];
  AddKeyframeData(nodes, 3, data, GetVectorValue);

  // Create splines and operations for each curve.
  std::vector<MatrixOperationInit>& ops = anim->ops();
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
  // Create a list of uncompressed nodes per channel and populate them.
  std::vector<UncompressedNode> nodes[4];
  AddKeyframeData(nodes, 4, data, GetQuaternionValue);

  // Create splines and operations for each curve.
  std::vector<MatrixOperationInit>& ops = anim->ops();
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
