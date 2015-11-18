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

#ifndef MOTIVE_ANIM_TABLE_H_
#define MOTIVE_ANIM_TABLE_H_

#include <string>
#include <unordered_map>
#include <vector>

#include "motive/anim.h"

namespace motive {

struct AnimTableFb;
struct RigAnimFb;

/// @class AnimTable
/// @brief Hold animation lists for several object types.
class AnimTable {
 public:
  /// Callback that returns a pointer to the `RigAnimFb` FlatBuffer table for
  /// `anim_name`. Your implementation may vary, but one possibility is to
  /// load the file with `anim_name` into `scratch_buf`, and then return a
  /// pointer to `scratch_buf`.
  ///
  /// Note: Motive does not make assumptions on file io, so the caller must
  ///       provide this function.
  ///
  /// Example:
  ///   const RigAnimFb* MyLoadRigAnimFn(const char* anim_name,
  ///                                    std::string* scratch_buf) {
  ///     const bool load_ok = MyLoadFile(anim_name, scratch_buf);
  ///     assert(load_ok);
  ///     return motive::GetRigAnimFb(scratch_buf->c_str());
  ///   }
  typedef const RigAnimFb* LoadRigAnimFn(const char* anim_name,
                                         std::string* scratch_buf);

  /// Load the AnimTable specified in the FlatBuffer `params`.
  /// For each animation in the AnimTable, `load_fn` is called to get the
  /// `RigAnimFb` associated with the animation.
  bool InitFromFlatBuffers(const AnimTableFb& params, LoadRigAnimFn* load_fn);

  /// Get an animation by index. This is fast and is the preferred way to
  /// look up an animation.
  /// @param object An enum defined by the caller specifying the object type.
  ///               For example, if you want separate animations lists for
  ///               cats and dogs, then object=0 could be cats, and object=1
  ///               could be dogs. The enum should run consecutively from
  ///               0 to number of object types -1.
  /// @param anim_idx An enum defined by the caller specifying the index into
  ///                 the animation list for `object`. If your cat can run
  ///                 and sleep, then you might make run=0 and sleep=1. Enum
  ///                 should run consecutively from 0 to the number of
  ///                 animations - 1. No need for the number of animations to
  ///                 match between objects.
  const RigAnim* Query(int object, int anim_idx) const {
    const AnimIndex idx = CalculateIndex(object, anim_idx);
    return idx == kInvalidAnimIndex ? nullptr : &anims_[idx];
  }

  /// Get an animation by name. This is slow and should be avoided when
  /// possible.
  const RigAnim* QueryByName(const char* anim_name) const {
    return const_cast<AnimTable*>(this)->QueryByName(anim_name);
  }

  /// Return animation that defines the complete rig of this object.
  const RigAnim& DefiningAnim(int object) const {
    return defining_anims_[object];
  }

  /// Return size of the top-most vector. Recall that AnimTable is a vector
  /// of vectors.
  int NumObjects() const { return static_cast<int>(indices_.size()); }

 private:
  typedef uint16_t AnimIndex;
  typedef std::vector<AnimIndex> AnimList;
  typedef std::pair<std::string, AnimIndex> NameToIndex;
  static const AnimIndex kInvalidAnimIndex = static_cast<AnimIndex>(-1);

  AnimIndex AddAnimName(const char* anim_name);
  void InitNameMapFromFlatBuffers(const AnimTableFb& params);
  bool LoadAnimations(LoadRigAnimFn* load_fn);
  void AnimNames(std::vector<const char*>* anim_names) const;
  size_t MaxAnimIndex() const;
  size_t GatherObjectAnims(int object, const RigAnim** anims) const;
  void CalculateDefinineAnims();

  RigAnim* QueryByName(const char* anim_name) {
    auto map_entry = name_map_.find(anim_name);
    if (map_entry == name_map_.end()) return nullptr;
    return &anims_[map_entry->second];
  }

  AnimIndex CalculateIndex(int object, int anim_idx) const {
    assert(0 <= object && object < static_cast<int>(indices_.size()));
    const AnimList& list = indices_[object];

    assert(0 <= anim_idx && anim_idx < static_cast<int>(list.size()));
    return list[anim_idx];
  }

  /// Vector of vectors. Top vector is indexed by `object`. Bottom vector is
  /// indexed by `anim_idx`. Holds an index into the `anims_` array.
  std::vector<AnimList> indices_;

  /// For each `object` type, holds the index to the animation that defines
  /// the complete rig. Animations may animate only a sub-rig, but we need to
  /// have a complete rig for initialization.
  std::vector<RigAnim> defining_anims_;

  /// Map animation name to an animation index.
  /// Used for by-name lookups, and to avoid duplicate copies of the same
  /// animation.
  std::unordered_map<std::string, AnimIndex> name_map_;

  /// Animation data. Contains no duplicate entries, thanks to name_map_.
  std::vector<RigAnim> anims_;
};

}  // namespace motive

#endif  // MOTIVE_ANIM_TABLE_H_
