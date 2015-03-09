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

#include "motive/util/optimizations.h"

#if defined(__ANDROID__)
#include <cpu-features.h>
#endif  // defined(__ANDROID__)

namespace fpl {

ProcessorOptimization BestProcessorOptimization() {
// TODO: Add checks for other operating systems.
#if defined(__ANDROID__)
  const uint64_t features = android_getCpuFeatures();
  switch (android_getCpuFamily()) {
    case ANDROID_CPU_FAMILY_ARM:
      return features & ANDROID_CPU_ARM_FEATURE_NEON ? kNeonOptimizations
                                                     : kNoOptimizations;

    case ANDROID_CPU_FAMILY_X86:
      return features & ANDROID_CPU_X86_FEATURE_SSSE3 ? kSsse3Optimizations
                                                      : kSse3Optimizations;

    default:
      break;
  }
#endif  // defined(__ANDROID__)

  return kNoOptimizations;
}

}  // namespace fpl
