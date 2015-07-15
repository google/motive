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

#ifndef MOTIVE_COMMON_H_
#define MOTIVE_COMMON_H_

#include <cstdint>

#include "mathfu/glsl_mappings.h"

namespace motive {

class MotiveProcessor;
class Motivator;

/// @typedef MotivatorType
/// MotivatorType is used for run-time type information. It's implemented as a
/// pointer to a string. Each derivation of MotivatorInit supplies a `kType`
/// static member that's identifies the corresponding MotiveProcessor.
/// We put `kType` in MotivatorInit instead of MotiveProcessor because
/// only MotivatorInit is in the external API.
typedef const char** MotivatorType;
static const MotivatorType kMotivatorTypeInvalid = nullptr;

/// @typedef MotiveIndex
/// The MotiveIndex identifies an Motivator inside a MotiveProcessor. The
/// MotiveProcessor holds all Motivators of its type. Calls to Motivators are
/// proxied to the MotiveProcessor, along with their MotiveIndex.
typedef int32_t MotiveIndex;
static const MotiveIndex kMotiveIndexInvalid = static_cast<MotiveIndex>(-1);

/// @typedef MotiveDimension
/// Identify how many slots in the MotiveProcessor a Motivator occupies.
/// A Motivator3f occupies three slots, for example. Arithmetic is mixed
/// between MotiveIndex and MotiveDimension in fpl::IndexAllocator, so they
/// should be the same type.
typedef MotiveIndex MotiveDimension;

/// @typedef MotiveChildIndex
/// Motivators can have child components. For example, a matrix motivator is
/// composed of a series of basic matrix operations. Each operation is a
/// child component.
typedef uint32_t MotiveChildIndex;

/// @typedef MotiveTime
/// Time units are defined by the user. We use integer instead of floating
/// point to avoid a loss of precision as time accumulates.
typedef int MotiveTime;

/// @class MotivatorInit
/// Base class for Motivator parameterization. Every motivator type has a
/// its own init class that derives from MotivatorInit. In the derivation,
/// the MOTIVE_INTERFACE() macro should be added to the public interface.
/// This macro will define `kName`, `kType`, and Register() calls for the
/// motivator type.
class MotivatorInit {
 public:
  /// The derived class's constructor should set 'type'.
  explicit MotivatorInit(MotivatorType type) : type_(type) {}

  MotivatorType type() const { return type_; }
  void set_type(MotivatorType type) { type_ = type; }

 private:
  MotivatorType type_;
};

/// Add this to the public interface of your derivation of Init. It defines
/// a unique identifier for this type as kType. Your derivation's constructor
/// should construct base class with Init(kType).
#define MOTIVE_INTERFACE()                  \
  static const char* kName;                 \
  static const motive::MotivatorType kType; \
  static void Register()

/// Add this to the source file with your processor code. It instantiates the
/// static variables and functions declared in MOTIVE_INTERFACE.
/// Example usage,
///    MOTIVE_INSTANCE(AwesomeInit, AwesomeMotiveProcessor);
#define MOTIVE_INSTANCE(InitType, ProcessorType)                               \
  static motive::MotiveProcessor* ProcessorType##Create() {                    \
    return new ProcessorType();                                                \
  }                                                                            \
  static void ProcessorType##Destroy(motive::MotiveProcessor* p) { delete p; } \
  void InitType::Register() {                                                  \
    const motive::MotiveProcessorFunctions functions(ProcessorType##Create,    \
                                                     ProcessorType##Destroy);  \
    motive::MotiveEngine::RegisterProcessorFactory(InitType::kType,            \
                                                   functions);                 \
  }                                                                            \
  const char* InitType::kName = #ProcessorType;                                \
  const motive::MotivatorType InitType::kType = &InitType::kName

/// Return the number of elements in an array 'a', as type `size_t`.
/// If 'a' is not an array, generates an error by dividing by zero.
#define MOTIVE_ARRAY_SIZE(a)    \
  ((sizeof(a) / sizeof(*(a))) / \
   static_cast<size_t>(!(sizeof(a) % sizeof(*(a)))))

}  // namespace motive

#endif  // MOTIVE_COMMON_H_
