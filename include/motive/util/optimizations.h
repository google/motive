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

#ifndef MOTIVE_UTIL_OPTIMIZATIONS_H_
#define MOTIVE_UTIL_OPTIMIZATIONS_H_

namespace fpl {

enum ProcessorOptimization {
  kNoOptimizations,
  kNeonOptimizations,  /// NEON is a SIMD instruction set for ARM processors
  kSse3Optimizations,  /// SSE is a SIMD instruction set for x86 processors
  kSsse3Optimizations  /// SSSE3 is an extension of SSE3
};

/// Look at the capabilities of the CPU and return the most performant set of
/// processor optimizations. For example, on Android, return kNeonOptimizations
/// if the CPU supports the NEON instruction set. On x86 (once we have x86
/// optimizations), return kSse2Optimizations if it's supported, or if it's not,
/// return kSse2Optimizations if it's supported. If none of the processors are
/// supported, return kNoOptimizations.
ProcessorOptimization BestProcessorOptimization();

}  // namespace fpl

#endif  // MOTIVE_UTIL_OPTIMIZATIONS_H_