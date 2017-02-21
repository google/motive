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

#include "motive/version.h"

namespace motive {

#define MOTIVE_VERSION_MAJOR 1
#define MOTIVE_VERSION_MINOR 1
#define MOTIVE_VERSION_REVISION 1

// Turn X into a string literal.
#define MOTIVE_STRING_EXPAND(X) #X
#define MOTIVE_STRING(X) MOTIVE_STRING_EXPAND(X)

/// @var kVersion
/// @brief String which identifies the current version of Motive.
///
/// @ref kVersion is used by Google developers to identify which applications
/// uploaded to Google Play are using this library. This allows the development
/// team at Google to determine the popularity of the library.
/// How it works: Applications that are uploaded to the Google Play Store are
/// scanned for this version string. We track which applications are using it
/// to measure popularity. You are free to remove it (of course) but we would
/// appreciate if you left it in.
///
static const MotiveVersion kVersion = {
  MOTIVE_VERSION_MAJOR,
  MOTIVE_VERSION_MINOR,
  MOTIVE_VERSION_REVISION,
  "Motive animation system "
  MOTIVE_STRING(MOTIVE_VERSION_MAJOR) "."
  MOTIVE_STRING(MOTIVE_VERSION_MINOR) "."
  MOTIVE_STRING(MOTIVE_VERSION_REVISION)
};

const MotiveVersion& Version() { return kVersion; }

}  // namespace motive
