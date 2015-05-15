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

#ifndef MOTIVE_ENGINE_H_
#define MOTIVE_ENGINE_H_

#include <map>
#include <set>

#include "motive/common.h"
#include "motive/processor.h"
#include "motive/util/benchmark.h"

namespace motive {

struct MotiveProcessorFunctions;
struct MotiveVersion;

/// @class MotiveEngine
/// @brief Hold and update all animation data.
///
/// The engine holds all of the MotiveProcessors, and updates them all when
/// AdvanceFrame() is called. The processing is kept central, in this manner,
/// for scalability. The engine is not a singleton, but you should try to
/// minimize the number of engines in your game. As more Motivators are added to
/// the processors, you start to get economies of scale.
class MotiveEngine {
  struct ProcessorDetails {
    MotiveProcessor* processor;
    int benchmark_id;
    bool operator<(const ProcessorDetails& rhs) const {
      return processor->Priority() < rhs.processor->Priority();
    }
  };
  typedef std::map<MotivatorType, MotiveProcessor*> ProcessorMap;
  typedef std::pair<MotivatorType, MotiveProcessor*> ProcessorPair;
  typedef std::multiset<ProcessorDetails> ProcessorSet;
  typedef std::map<MotivatorType, MotiveProcessorFunctions> FunctionMap;
  typedef std::pair<MotivatorType, MotiveProcessorFunctions> FunctionPair;

 public:
  MotiveEngine();
  ~MotiveEngine() { Reset(); }

  /// Deallocate all MotiveProcessors, which, in turn, resets all Motivators
  /// that use those MotiveProcessors.
  void Reset();

  /// Update all the MotiveProcessors by 'delta_time'. This advances all
  /// Motivators created with this MotiveEngine.
  /// @param delta_time Elapsed time since the last call to AdvanceFrame().
  ///                   Time units are defined by the user. When using spline
  ///                   animations, for instance, the time unit is the unit of
  ///                   the x-axis.
  void AdvanceFrame(MotiveTime delta_time);

  /// @private For internal use only.
  MotiveProcessor* Processor(MotivatorType type);

  /// @private For internal use only.
  static void RegisterProcessorFactory(MotivatorType type,
                                       const MotiveProcessorFunctions& fns);

 private:
  /// Map from the MotivatorType to the MotiveProcessor. Only one
  /// MotiveProcessor per type per engine. This is to maximize centralization
  /// of data.
  ProcessorMap mapped_processors_;

  /// Sort the MotiveProcessors by priority. Low numbered priorities run first.
  /// This allows high number priorities to have child motivators, as long as
  /// the child motivators have lower priority.
  ProcessorSet sorted_processors_;

  /// Current version of the Motive Animation System.
  const MotiveVersion* version_;

  /// ProcessorMap from the MotivatorType to the factory that creates the
  /// MotiveProcessor. We only create an MotiveProcessor when one is needed.
  static FunctionMap function_map_;
};

}  // namespace motive

#endif  // MOTIVE_ENGINE_H_
