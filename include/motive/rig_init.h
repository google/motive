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

#ifndef MOTIVE_RIG_INIT_H_
#define MOTIVE_RIG_INIT_H_

#include "motive/common.h"
#include "mathfu/glsl_mappings.h"

namespace motive {

class RigAnim;

class RigInit : public MotivatorInit {
 public:
  MOTIVE_INTERFACE();

  RigInit(const RigAnim& defining_anim, const BoneIndex* bone_parents,
          BoneIndex num_bones, BoneIndex root_motion_bone = kInvalidBoneIdx);
  const RigAnim& defining_anim() const { return *defining_anim_; }
  const mathfu::AffineTransform* bone_transforms() const {
    return bone_transforms_;
  }
  const BoneIndex root_motion_bone() const { return root_motion_bone_; }

  // Utility functions. Ensure that animations are compatible with rigs.
  static bool MatchesHierarchy(const BoneIndex* parents_a, BoneIndex len_a,
                               const BoneIndex* parents_b, BoneIndex len_b);
  static bool MatchesHierarchy(const RigAnim& anim, const BoneIndex* parents_b,
                               BoneIndex len_b);
  static bool MatchesHierarchy(const RigAnim& anim_a, const RigAnim& anim_b);

 private:
  /// Animation defining hierarchy and the union of matrix ops (across all
  /// animations).
  const RigAnim* defining_anim_;

  /// Array defining default pose. That is, the transformation from a bone to
  /// its parent. With just these, you can reconstruct the model in the pose
  /// it was exported in (i.e. its default pose).
  /// These transforms are used as the `start_transform_`s of the underlying
  /// `MatrixInit`s. All the matrix operations are applied from the origin of
  /// the bone they're animating.
  const mathfu::AffineTransform* bone_transforms_;

  /// The index of the bone that root motion should be extracted from. This
  /// bone's transform will be stored separately when updating each bone's
  /// transform each frame instead of being applied to the hierarchy and can
  /// be accessed using RigMotivator::RootMotionTransform(). If kInvalidBoneIdx,
  /// root motion will not be extracted.
  BoneIndex root_motion_bone_;
};

}  // namespace motive

#endif  // MOTIVE_RIG_INIT_H_
