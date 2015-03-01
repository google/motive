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

namespace impel {

class ImpelProcessor;
class Impeller;

// Impeller type is used for run-time type information. It's implemented as a
// pointer to a string, in each derivation of ImpelInit.
typedef const char** ImpellerType;
static const ImpellerType kImpelTypeInvalid = nullptr;

// The ImpelIndex identifies an Impeller inside an ImpelProcessor. The
// ImpelProcessor holds all Impellers of its type. Calls to Impellers are
// proxied to the ImpelProcessor.
typedef uint32_t ImpelIndex;
static const ImpelIndex kImpelIndexInvalid = static_cast<ImpelIndex>(-1);

// Impellers can have child components. For example, a matrix impeller is
// composed of a series of basic matrix operations. Each operation is a
// child component.
typedef uint32_t ImpelChildIndex;

// Time units are defined by the user. We use integer instead of floating
// point to avoid a loss of precision as time accumulates.
typedef int ImpelTime;

// Base class for Impeller parameterization. Every impeller type has a different
// set of parameters that define its movement. Every impeller type derives its
// own Init class from ImpelInit, to define those parameters.
class ImpelInit {
 public:
  // The derived class's constructor should set 'type'.
  explicit ImpelInit(ImpellerType type) : type_(type) {}

  ImpellerType type() const { return type_; }
  void set_type(ImpellerType type) { type_ = type; }

 private:
  ImpellerType type_;
};

// Add this to the public interface of your derivation of ImpelInit. It defines
// a unique identifier for this type as kType. Your derivation's constructor
// should construct base class with ImpelInit(kType).
#define IMPEL_INTERFACE()          \
  static const char* kName;        \
  static const ImpellerType kType; \
  static void Register()

// Add this to the source file with your processor code. It instantiates the
// static variables and functions declared in IMPEL_INTERFACE.
// Example usage,
//    IMPEL_INSTANCE(AwesomeImpelInit, AwesomeImpelProcessor);
#define IMPEL_INSTANCE(InitType, ProcessorType)                        \
  static impel::ImpelProcessor* ProcessorType##Create() {              \
    return new ProcessorType();                                        \
  }                                                                    \
  static void ProcessorType##Destroy(ImpelProcessor* p) { delete p; }  \
  void InitType::Register() {                                          \
    const ImpelProcessorFunctions functions(ProcessorType##Create,     \
                                            ProcessorType##Destroy);   \
    ImpelEngine::RegisterProcessorFactory(InitType::kType, functions); \
  }                                                                    \
  const char* InitType::kName = #ProcessorType;                        \
  const impel::ImpellerType InitType::kType = &InitType::kName


// ARRAYSIZE performs essentially the same calculation as arraysize,
// but can be used on anonymous types or types defined inside
// functions.  It's less safe than arraysize as it accepts some
// (although not all) pointers.  Therefore, you should use arraysize
// whenever possible.
//
// The expression ARRAYSIZE(a) is a compile-time constant of type
// size_t.
//
// ARRAYSIZE catches a few type errors.  If you see a compiler error
//
//   "warning: division by zero in ..."
//
// when using ARRAYSIZE, you are (wrongfully) giving it a pointer.
// You should only use ARRAYSIZE on statically allocated arrays.
//
// The following comments are on the implementation details, and can
// be ignored by the users.
//
// ARRAYSIZE(arr) works by inspecting sizeof(arr) (the # of bytes in
// the array) and sizeof(*(arr)) (the # of bytes in one array
// element).  If the former is divisible by the latter, perhaps arr is
// indeed an array, in which case the division result is the # of
// elements in the array.  Otherwise, arr cannot possibly be an array,
// and we generate a compiler error to prevent the code from
// compiling.
//
// Since the size of bool is implementation-defined, we need to cast
// !(sizeof(a) & sizeof(*(a))) to size_t in order to ensure the final
// result has type size_t.
//
// This macro is not perfect as it wrongfully accepts certain
// pointers, namely where the pointer size is divisible by the pointee
// size.  Since all our code has to go through a 32-bit compiler,
// where a pointer is 4 bytes, this means all pointers to a type whose
// size is 3 or greater than 4 will be (righteously) rejected.
//
// Kudos to Jorg Brown for this simple and elegant implementation.
//
// - wan 2005-11-16
//
// Starting with Visual C++ 2005, WinNT.h includes ARRAYSIZE.
#if !defined(_WIN32)
#define ARRAYSIZE(a)            \
  ((sizeof(a) / sizeof(*(a))) / \
   static_cast<size_t>(!(sizeof(a) % sizeof(*(a)))))
#endif  // !defined(_WIN32)


}  // namespace impel

#endif  // MOTIVE_COMMON_H_
