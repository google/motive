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

#include "motive/motivator.h"
#include "motive/engine.h"

namespace motive {

void Motivator::InitializeWithDimension(const MotivatorInit& init,
                                        MotiveEngine* engine,
                                        MotiveDimension dimensions) {
  // Unregister ourselves with our existing MotiveProcessor.
  Invalidate();

  // The MotiveProcessors are held centrally in the MotiveEngine. There is only
  // one processor per type. Get that processor.
  MotiveProcessor* processor = engine->Processor(init.type());

  // Register and initialize ourselves with the MotiveProcessor.
  processor->InitializeMotivator(init, engine, this, dimensions);
}

}  // namespace motive
