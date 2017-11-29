// Copyright 2017 Google Inc. All rights reserved.
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

#ifndef MOTIVE_ANIM_PIPELINE_H_
#define MOTIVE_ANIM_PIPELINE_H_

#include <string>
#include "fbx_common/fbx_common.h"

namespace motive {

enum RepeatPreference { kRepeatIfRepeatable, kAlwaysRepeat, kNeverRepeat };

/// @brief Amount the output curves are allowed to deviate from the input
///        curves.
struct Tolerances {
  Tolerances();

  /// Amount output scale curves can deviate, unitless.
  float scale;

  /// Amount output rotate curves can deviate, in radians.
  float rotate;

  /// Amount output translate curves can deviate, in scene's distance units.
  float translate;

  /// Amount derivative--converted to an angle in x/y--can deviate, in radians.
  float derivative_angle;

  /// Amount derivative--converted to an angle in x/y--can deviate, in radians,
  /// This value used when determining if an animation repeats or not.
  float repeat_derivative_angle;
};

struct AnimPipelineArgs {
  AnimPipelineArgs();

  std::string fbx_file;         /// FBX input file to convert.
  std::string output_file;      /// File to write .fplanim to.
  fplutil::LogLevel log_level;  /// Amount of logging to dump during conversion.
  Tolerances tolerances;        /// Amount output curves can deviate from input.
  RepeatPreference repeat_preference;  /// Loop back to start when reaches end.
  bool stagger_end_times;    /// Allow each channel to end at authored time.
  bool preserve_start_time;  /// Don't shift channels to start at time 0.
  bool root_bones_only;      /// Output bone that has path of animation only.
  bool no_uniform_scale;     /// If true, never collapse scale channels.
  fplutil::AxisSystem axis_system;  /// Which axes are up, front, left.
  float distance_unit_scale;        /// This number of cm is set to one unit.
  int debug_time;  /// If >0 output animation state at this time.
};

int RunAnimPipeline(const AnimPipelineArgs& args, fplutil::Logger& log);

}  // namespace motive

#endif  // MOTIVE_ANIM_PIPELINE_H_
