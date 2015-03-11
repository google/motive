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

// Motivator type is used for run-time type information. It's implemented as a
// pointer to a string, in each derivation of Init.
typedef const char** MotivatorType;
static const MotivatorType kMotivatorTypeInvalid = nullptr;

// The MotiveIndex identifies an Motivator inside an MotiveProcessor. The
// MotiveProcessor holds all Motivators of its type. Calls to Motivators are
// proxied to the MotiveProcessor.
typedef uint32_t MotiveIndex;
static const MotiveIndex kMotiveIndexInvalid = static_cast<MotiveIndex>(-1);

// Motivators can have child components. For example, a matrix motivator is
// composed of a series of basic matrix operations. Each operation is a
// child component.
typedef uint32_t MotiveChildIndex;

// Time units are defined by the user. We use integer instead of floating
// point to avoid a loss of precision as time accumulates.
typedef int MotiveTime;

// Base class for Motivator parameterization. Every motivator type has a
// different
// set of parameters that define its movement. Every motivator type derives its
// own Init class from Init, to define those parameters.
class MotivatorInit {
 public:
  // The derived class's constructor should set 'type'.
  explicit MotivatorInit(MotivatorType type) : type_(type) {}

  MotivatorType type() const { return type_; }
  void set_type(MotivatorType type) { type_ = type; }

 private:
  MotivatorType type_;
};

// Add this to the public interface of your derivation of Init. It defines
// a unique identifier for this type as kType. Your derivation's constructor
// should construct base class with Init(kType).
#define MOTIVE_INTERFACE()                  \
  static const char* kName;                 \
  static const motive::MotivatorType kType; \
  static void Register()

// Add this to the source file with your processor code. It instantiates the
// static variables and functions declared in MOTIVE_INTERFACE.
// Example usage,
//    MOTIVE_INSTANCE(AwesomeInit, AwesomeMotiveProcessor);
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

}  // namespace motive

#endif  // MOTIVE_COMMON_H_
