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

#include "motive/anim.h"
#include "motive/init.h"

namespace motive {

static const char* kMatrixOpNames[] = {
    "Invalid Matrix Operation",  // kInvalidMatrixOperation
    "Rotate About X",            // kRotateAboutX
    "Rotate About Y",            // kRotateAboutY
    "Rotate About Z",            // kRotateAboutZ
    "Translate X",               // kTranslateX
    "Translate Y",               // kTranslateY
    "Translate Z",               // kTranslateZ
    "Scale X",                   // kScaleX
    "Scale Y",                   // kScaleY
    "Scale Z",                   // kScaleZ
    "Scale Uniformly",           // kScaleUniformly
};
static_assert(MOTIVE_ARRAY_SIZE(kMatrixOpNames) == kNumMatrixOperationTypes,
              "Adjust kMatrixOpNames to match enumeration");

const char* MatrixOpName(const motive::MatrixOperationType op) {
  assert(0 <= op && op < kNumMatrixOperationTypes);
  return kMatrixOpNames[op];
}

RigInit::RigInit(const RigAnim& defining_anim, const BoneIndex* bone_parents,
                 BoneIndex num_bones)
    : MotivatorInit(kType), defining_anim_(&defining_anim) {
  // Ensure the animation and the mesh have the same hierarchy.
  // We allow the one exception where there are only popsicle stick animations
  // (i.e. animations that affect a single root bone, like popsicle-stick
  // puppets).
  assert(defining_anim.NumBones() == 1 ||
         MatchesHierarchy(defining_anim, bone_parents, num_bones));
  (void)bone_parents;
  (void)num_bones;
}

// static
bool RigInit::MatchesHierarchy(const BoneIndex* parents_a, BoneIndex len_a,
                               const BoneIndex* parents_b, BoneIndex len_b) {
  // TODO: Implement runtime retargetting by allowing the hiearchy to be
  //       slightly different on bones that aren't animated.
  return len_a == len_b &&
         memcmp(parents_a, parents_b, len_a * sizeof(parents_a[0])) == 0;
}

// static
bool RigInit::MatchesHierarchy(const RigAnim& anim, const BoneIndex* parents_b,
                               BoneIndex len_b) {
  return MatchesHierarchy(anim.bone_parents(), anim.NumBones(), parents_b,
                          len_b);
}

// static
bool RigInit::MatchesHierarchy(const RigAnim& anim_a, const RigAnim& anim_b) {
  return MatchesHierarchy(anim_a.bone_parents(), anim_a.NumBones(),
                          anim_b.bone_parents(), anim_b.NumBones());
}

}  // namespace motive
