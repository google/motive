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

#include "anim_pipeline.h"

#include "anim_generated.h"
#include "fplutil/file_utils.h"
#include "motive/math/angle.h"

using fplutil::kLogError;
using fplutil::kLogImportant;
using fplutil::kLogInfo;
using fplutil::kLogVerbose;
using fplutil::kLogWarning;

static void LogUsage(fplutil::Logger* log) {
  static const char kOptionIndent[] = "                           ";
  log->Log(
      kLogImportant,
      "Usage: anim_pipeline [-v|-d|-i] [-o OUTPUT_FILE]\n"
      "                     [-st SCALE_TOLERANCE] [-rt ROTATE_TOLERANCE]\n"
      "                     [-tt TRANSLATE_TOLERANCE]\n"
      "                     [-at DERIVATIVE_TOLERANCE] [--repeat|--norepeat]\n"
      "                     [--stagger] [--start] [-a AXES]\n"
      "                     [-u (unit)|(scale)] [--roots] [--debug_time TIME]\n"
      "                     FBX_FILE\n"
      "\n"
      "Pipeline to convert FBX animations into FlatBuffer animations.\n"
      "Outputs a .motiveanim file with the same base name as FBX_FILE.\n\n"
      "Options:\n"
      "  -v, --verbose output all informative messages\n"
      "  -d, --details output important informative messages\n"
      "  -i, --info    output more than details, less than verbose.\n"
      "  -o, --out OUTPUT_FILE\n"
      "                file to write .motiveanim file to.\n"
      "                Can be an absolute or relative path.\n"
      "                when unspecified, uses base FBX name + .motiveanim.\n"
      "  -st, --scale SCALE_TOLERANCE\n"
      "                max deviation of output scale curves from input\n"
      "                scale curves; unitless\n"
      "  -rt, --rotate ROTATE_TOLERANCE\n"
      "                max deviation of output rotate curves from intput\n"
      "                rotate curves; in degrees\n"
      "  -tt, --translate TRANSLATE_TOLERANCE\n"
      "                max deviation of output translate curves from input\n"
      "                translate curves; in scene's distance unit\n"
      "  -at, --angle DERIVATIVE_TOLERANCE\n"
      "                max deviation of curve derivatives,\n"
      "                considered as an angle in the x/y plane\n"
      "                (e.g. derivative 1 ==> 45 degrees); in degrees.\n"
      "  --repeat, --norepeat\n"
      "                mark the animation as repeating or not repeating.\n"
      "                A repeating animation cycles over and over.\n"
      "                If neither option is specified, the animation\n"
      "                is marked as repeating when it starts and ends\n"
      "                with the same pose and derivatives.\n"
      "  --stagger, --stagger_end_times\n"
      "                allow every channel to end at its authored time,\n"
      "                instead of adding extra spline nodes to plum-up\n"
      "                every channel.\n"
      "                This may cause strage behavior with animations that\n"
      "                repeat, since the shorter channels will start\n"
      "                to repeat before the longer ones.\n"
      "  --start, --preserve_start_time\n"
      "                start the animation at the same time as in the source.\n"
      "                By default, the animation is shifted such that its\n"
      "                start time is zero.\n"
      "  --nouniformscale\n"
      "                prevents scale X/Y/Z channels from being collapse into\n"
      "                a single uniform scale channel even if they have\n"
      "                identical curves.\n"
      "  -a, --axes AXES\n"
      "                coordinate system of exported file, in format\n"
      "                    (up-axis)(front-axis)(left-axis) \n"
      "                where,\n"
      "                    'up' = [x|y|z]\n"
      "                    'front' = [+x|-x|+y|-y|+z|-z], is the axis\n"
      "                      pointing out of the front of the mesh.\n"
      "                      For example, the vector pointing out of a\n"
      "                      character's belly button.\n"
      "                    'left' = [+x|-x|+y|-y|+z|-z], is the axis\n"
      "                      pointing out the left of the mesh.\n"
      "                      For example, the vector from the character's\n"
      "                      neck to his left shoulder.\n"
      "                For example, 'z+y+x' is z-axis up, positive y-axis\n"
      "                out of a character's belly button, positive x-axis\n"
      "                out of a character's left side.\n"
      "                If unspecified, use file's coordinate system.\n"
      "  -u, --unit (unit)|(scale)\n"
      "                output animation in target units. You can override the\n"
      "                FBX file's distance unit with this option.\n"
      "                For example, if your game runs in meters,\n"
      "                specify '-u m' to ensure the output .fplmesh file\n"
      "                is in meters, no matter the distance unit of the\n"
      "                FBX file.\n"
      "                (unit) can be one of the following:\n");
  LogOptions(kOptionIndent, fplutil::DistanceUnitNames(), log);
  log->Log(
      kLogImportant,
      "                (scale) is the number of centimeters in your\n"
      "                distance unit. For example, instead of '-u inches',\n"
      "                you could also use '-u 2.54'.\n"
      "                If unspecified, use FBX file's unit.\n"
      "  --roots, --root_bones_only\n"
      "                output only the root bones of each mesh.\n"
      "                Each mesh gets its animation file.\n"
      "                Useful for pulling just the path data from an\n"
      "                animation.\n"
      "  --debug_time TIME\n"
      "                output the local transforms for each bone in\n"
      "                the animation at TIME, in ms, and then exit.\n"
      "                Useful for debugging situations where the\n"
      "                runtime doesn't match source data.\n");
}

static bool ParseAnimPipelineArgs(int argc, char** argv, fplutil::Logger& log,
                                  motive::AnimPipelineArgs* args) {
  bool valid_args = true;

  // Last parameter is used as file name.
  if (argc > 1) {
    args->fbx_file = std::string(argv[argc - 1]);
    args->output_file = fplutil::RemoveExtensionFromName(args->fbx_file) + "." +
                        motive::RigAnimFbExtension();
  }

  // Ensure file name is valid.
  const bool valid_fbx_file =
      args->fbx_file.length() > 0 && args->fbx_file[0] != '-';
  if (!valid_fbx_file) {
    valid_args = false;
  }

  // Parse switches.
  for (int i = 1; i < argc - 1; ++i) {
    const std::string arg = argv[i];

    if (arg == "-v" || arg == "--verbose") {
      args->log_level = kLogVerbose;

    } else if (arg == "-d" || arg == "--details") {
      args->log_level = kLogImportant;

    } else if (arg == "-i" || arg == "--info") {
      args->log_level = kLogInfo;

    } else if (arg == "-o" || arg == "--out") {
      if (i + 1 < argc - 1) {
        args->output_file = std::string(argv[i + 1]);
        i++;
      } else {
        valid_args = false;
      }

    } else if (arg == "-st" || arg == "--scale") {
      if (i + 1 < argc - 1) {
        args->tolerances.scale = static_cast<float>(atof(argv[i + 1]));
        if (args->tolerances.scale <= 0.0f) {
          log.Log(kLogError, "scale_tolerance must be > 0.");
          valid_args = false;
        }
        i++;
      } else {
        valid_args = false;
      }

    } else if (arg == "-rt" || arg == "--rotate") {
      if (i + 1 < argc - 1) {
        const float degrees = static_cast<float>(atof(argv[i + 1]));
        if (degrees <= 0.0f || degrees > 180.0f) {
          log.Log(kLogError, "rotate_tolerance must be >0 and <=180.");
          valid_args = false;
        } else {
          args->tolerances.rotate =
              motive::Angle::FromDegrees(degrees).ToRadians();
          i++;
        }
      } else {
        valid_args = false;
      }

    } else if (arg == "-tt" || arg == "--translate") {
      if (i + 1 < argc - 1) {
        args->tolerances.translate = static_cast<float>(atof(argv[i + 1]));
        if (args->tolerances.translate <= 0.0f) {
          log.Log(kLogError, "translate_tolerance must be > 0.");
          valid_args = false;
        }
        i++;
      } else {
        valid_args = false;
      }

    } else if (arg == "-at" || arg == "--angle") {
      if (i + 1 < argc - 1) {
        const float degrees = static_cast<float>(atof(argv[i + 1]));
        if (degrees <= 0.0f || degrees > 90.0f) {
          log.Log(kLogError, "derivative_tolerance must be >0 and <=90.");
          valid_args = false;
        } else {
          args->tolerances.derivative_angle =
              motive::Angle::FromDegrees(degrees).ToRadians();
        }
        i++;
      } else {
        valid_args = false;
      }

    } else if (arg == "--repeat" || arg == "--norepeat") {
      const motive::RepeatPreference repeat_preference =
          arg == "--repeat" ? motive::kAlwaysRepeat : motive::kNeverRepeat;
      if (args->repeat_preference != motive::kRepeatIfRepeatable &&
          args->repeat_preference != repeat_preference) {
        log.Log(kLogError,
                "Only one of --repeat and --norepeat can be specified.\n");
        valid_args = false;
      } else {
        args->repeat_preference = repeat_preference;
      }

    } else if (arg == "--nouniformscale") {
      args->no_uniform_scale = true;

    } else if (arg == "--stagger" || arg == "--stagger_end_times") {
      args->stagger_end_times = true;

    } else if (arg == "--start" || arg == "--preserve_start_time") {
      args->preserve_start_time = true;

    } else if (arg == "-a" || arg == "--axes") {
      if (i + 1 < argc - 1) {
        args->axis_system = fplutil::AxisSystemFromName(argv[i + 1]);
        valid_args = args->axis_system >= 0;
        if (!valid_args) {
          log.Log(kLogError, "Unknown coordinate system: %s\n\n", argv[i + 1]);
        }
        i++;
      } else {
        valid_args = false;
      }

    } else if (arg == "-u" || arg == "--unit") {
      if (i + 1 < argc - 1) {
        args->distance_unit_scale = fplutil::DistanceUnitFromName(argv[i + 1]);
        valid_args = args->distance_unit_scale > 0.0f;
        if (!valid_args) {
          log.Log(kLogError, "Unknown distance unit: %s\n\n", argv[i + 1]);
        }
        i++;
      } else {
        valid_args = false;
      }

    } else if (arg == "--roots" || arg == "--root_bones_only") {
      args->root_bones_only = true;

    } else if (arg == "--debug_time") {
      if (i + 1 < argc - 1) {
        args->debug_time = atoi(argv[i + 1]);
        args->log_level = kLogInfo;
        if (args->debug_time < 0) {
          log.Log(kLogError, "debug time must be >0.");
          valid_args = false;
        }
        i++;
      } else {
        valid_args = false;
      }

    } else {
      log.Log(kLogError, "Unknown parameter: %s\n", arg.c_str());
      valid_args = false;
    }
  }

  // Print usage.
  if (!valid_args) {
    LogUsage(&log);
  }
  return valid_args;
}

int main(int argc, char** argv) {
  fplutil::Logger log;

  motive::AnimPipelineArgs args;
  if (!ParseAnimPipelineArgs(argc, argv, log, &args)) return 1;

  return motive::RunAnimPipeline(args, log);
}
