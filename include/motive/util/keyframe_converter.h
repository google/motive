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

#ifndef MOTIVE_UTIL_KEYFRAME_CONVERTER_H_
#define MOTIVE_UTIL_KEYFRAME_CONVERTER_H_

#include "motive/matrix_anim.h"
#include "motive/matrix_op.h"

/// A set of functions for converting keyframe animation data into spline curves
/// and adding those curves to animations at runtime.
namespace motive {

/// Indicates the order of quaternion values in KeyframeData.
enum QuaternionOrder {
  kOrderWXYZ,
  kOrderXYZW,
};

/// Indicates the interpolation type between keyframes in KeyframeData.
enum InterpolationType {
  kLinear,
  kStep,
  kCubicSpline,
};

/// Represents |count| keyframes of animation data, each of which is consists of
/// a time and a value. |times| should be exactly |count| in length. |values|
/// is interpreted differently depending on the function handling it. |type|
/// determines the interpolation type used between keyframes. |ms_per_time_unit|
/// is a conversion factor indicating how many milliseconds there are per unit
/// that |times| is stored in. The default, 1000, assumes |times| is in seconds.
///
/// If |type| is kCubicSpline, all conversion functions assume keyframes are
/// tightly packed as [left derivative, value, right derivative].
struct KeyframeData {
  const float* times = nullptr;
  const float* values = nullptr;
  size_t count = 0;
  InterpolationType interpolation_type = kLinear;
  float ms_per_time_unit = 1000.f;
};

/// Returns the buffer size required to store |channel_count| CompactSpline
/// curves each with |keyframe_count| keyframes using a specific interpolation
/// |type|. Useful for allocating buffers to be used with AddArrayCurves() and
/// AddQuaternionCurves().
size_t GetRequiredBufferSize(size_t keyframe_count, size_t channel_count,
                             InterpolationType type);

/// Creates |channel_count| CompactSplines contiguously in the memory starting
/// at |buffer| using the provided keyframe |data| and returns the number of
/// bytes used to create them.
///
/// |data.values| is assumed to be in array-of-structs format. Specifically,
/// |data.values[i + j * channel_count]| is the j'th keyframe of the i'th
/// channel.
///
/// This function performs no bounds checking on |buffer|. Use
/// GetRequiredBufferSize() to determine how large a buffer is necessary.
size_t AddArrayCurves(uint8_t* buffer, const KeyframeData& data,
                      size_t channel_count);

/// Identical to AddArrayCurves(buffer, data, 4), but treats |data| as
/// containing quaternion curves and filters for quaternion "flips".
///
/// Curves are always added in the following order: W, X, Y, Z. |order|
/// determines which value of |quat| is used for each component.
size_t AddQuaternionCurves(uint8_t* buffer, const KeyframeData& data,
                           QuaternionOrder order);

/// Adds 3 constant-value operations to |anim|. For i in [0,3), the i'th new
/// operation will have type |base_type + i|, id |base_id + i|, and |vector[i]|
/// as its value. If the value is nearly the default for the given type, no
/// operation is added.
void AddVector3Constants(MatrixAnim* anim, MatrixOperationType base_type,
                         MatrixOpId base_id, const float* vector);

/// Adds 4 constant-value quaternion operations to |anim|. Operations are always
/// added in the following order: W, X, Y, Z. The W component is assigned an id
/// of |base_id|, the X component |base_id + 1|, and so on. |order| determines
/// which value of |quat| is used for each component. If the value is nearly the
/// default for the given type, no operation is added.
void AddQuaternionConstants(MatrixAnim* anim, MatrixOpId base_id,
                            const float* quat, QuaternionOrder order);

/// Adds 3 curve-backed operations to |anim|. For i in [0, 3), the i'th new
/// operation will have type |base_type + i| and id |base_id + i|. |data.values|
/// is assumed to be in array-of-structs format. Specifically,
/// |data.values[i + j * 3]| is the j'th keyframe of the i'th operation. Splines
/// are stored in |splines[i]|, which should be the return value of
/// anim.Construct().
void AddVector3Curves(MatrixAnim* anim, MatrixAnim::Spline* splines,
                      MatrixOperationType base_type, MatrixOpId base_id,
                      const KeyframeData& data);

/// Adds 4 curve-backed quaternion operations to |anim|. Operations are always
/// added in the following order: W, X, Y, Z. The W component is assigned an id
/// of |base_id|, the X component |base_id + 1|, and so on. |order| determines
/// which value of each sample is used for each component. |data.values| is
/// assumed to be in array-of-structs format. Specifically,
/// |data.values[i + j * 4]| is the j'th keyframe of the i'th operation. Splines
/// are stored in |splines[i]|, which should be the return value of
/// anim.Construct().
void AddQuaternionCurves(MatrixAnim* anim, MatrixAnim::Spline* splines,
                         MatrixOpId base_id, QuaternionOrder order,
                         const KeyframeData& data);

}  // namespace motive

#endif  // MOTIVE_UTIL_KEYFRAME_CONVERTER_H_
