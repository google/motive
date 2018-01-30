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

#ifndef MOTIVE_SIMPLE_INIT_H_
#define MOTIVE_SIMPLE_INIT_H_

#include "mathfu/constants.h"
#include "motive/math/range.h"
#include "motive/math/vector_converter.h"
#include "motive/util.h"

namespace motive {

/// @class SimpleInit
/// @brief Base class of Init classes for MotiveProcessors that derive from
///        SimpleProcessorTemplate.
///
/// You cannot initialize a Motivator with this class because it has no
/// MotivatorType. Instead, use one of the Init classes below that derive from
/// SimpleInit.
struct SimpleInit : public MotivatorInit {
  explicit SimpleInit(MotivatorType type)
      : MotivatorInit(type),
        start_values(nullptr),
        start_derivatives(nullptr) {}

  SimpleInit(MotivatorType type, const float* start_values,
             const float* start_derivatives = nullptr)
      : MotivatorInit(type),
        start_values(start_values),
        start_derivatives(start_derivatives) {}

  /// The starting value of each curve for each dimension. Array of length equal
  /// to the number of dimensions. This points at external values and the caller
  /// is responsible for ensuring these external values live as long as this
  /// struct.
  const float* start_values;

  /// The starting derivative of each curve for each dimension. Array of length
  /// equal to the number of dimensions. This points at external values and the
  /// caller is responsible for ensuring these external values live as long as
  /// this struct.
  const float* start_derivatives;
};

/// @class SimpleInitTemplate
/// @brief A version of SimpleInit for Motivators with kDimensions.
/// Use this class to initialize a Motivator with vector types, instead of using
/// the float arrays required by the base class SimpleInit.
/// For example, use a derivation of SimpleInit3f to initialize a Motivator3f.
template <class BaseT, class VectorConverter, MotiveDimension kDimensionsParam>
struct SimpleInitTemplate : public BaseT {
  static const MotiveDimension kDimensions = kDimensionsParam;

  typedef VectorConverter C;
  typedef typename VectorT<C, kDimensions>::type Vec;

  SimpleInitTemplate()
      : BaseT(C::ToPtr(start_values), C::ToPtr(start_derivatives)),
        start_values(0.0f),
        start_derivatives(0.0f) {}

  SimpleInitTemplate(const Vec& start_values_param,
                     const Vec& start_derivatives_param)
      : BaseT(C::ToPtr(start_values), C::ToPtr(start_derivatives)),
        start_values(start_values_param),
        start_derivatives(start_derivatives_param) {}

  const Vec start_values;
  const Vec start_derivatives;
};

}  // namespace motive

#endif  // MOTIVE_SIMPLE_INIT_H_
