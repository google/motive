// Copyright 2016 Google Inc. All rights reserved.
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

#include "motive/math/angle.h"
#include "motive/common.h"

namespace motive {

static const mathfu::vec3 kUpVectors[] = {
    mathfu::kAxisZ3f,   // kAngleToVectorXY
    mathfu::kAxisY3f,   // kAngleToVectorXZ
    mathfu::kAxisX3f,   // kAngleToVectorYZ
    -mathfu::kAxisZ3f,  // kAngleToVectorYX
    -mathfu::kAxisY3f,  // kAngleToVectorZX
    -mathfu::kAxisX3f,  // kAngleToVectorZY
};
static_assert(MOTIVE_ARRAY_SIZE(kUpVectors) == kAngleToVectorCount,
              "kUpVectors out of sync with AngleToVectorSystem enum");

// static
const mathfu::vec3& VectorSystemUp(const AngleToVectorSystem system) {
  assert(0 <= system && system < kAngleToVectorCount);
  return kUpVectors[system];
}

mathfu::vec3 LatitudeAndLongitudeToUnitSphere(const Angle& latitude,
                                              const Angle& longitude,
                                              AngleToVectorSystem system) {
  // Get the radius of the circle parallel to equator, at our latitude,
  // and the height above the equator.
  const mathfu::vec2 radius_and_up_dist = latitude.ToXYVector2f();
  const float radius = fabs(radius_and_up_dist.x());
  const float up_dist = radius_and_up_dist.y();

  // Get the location on the equator, for our longitude.
  const mathfu::vec3 unit_circle = longitude.ToVectorSystem(system);

  // Put the two together.
  const mathfu::vec3 unit_sphere =
      radius * unit_circle + up_dist * VectorSystemUp(system);
  return unit_sphere;
}

}  // namespace motive
