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

/// @class AnimTable
/// @brief Hold animation lists for several object types.
class AnimTable {
 public:
  /// Load the AnimTable specified in the FlatBuffer `params`.
  /// Enumerates all animations that need to be initialized, but doesn't
  /// initialize any of them. Those animations might be in many formats.
  /// If your animations are in FlatBuffer format, too, then call `AnimNames()`
  /// to get all the files that must be loaded. Then for each file, call
  /// RigAnimFromFlatBuffers() with the loaded FlatBuffer and the RigAnim
  /// returned by QueryByName().
  void InitFromFlatBuffers(const AnimTableFb& params);

  /// Enumerate all animations that are held in this table. Animations are
  /// listed only once. Multiple indices may map to one animation.
  /// Valid after InitFromFlatBuffers() is called.
  void AnimNames(std::vector<const char*>* anim_names) const {
    anim_names->clear();
    anim_names->reserve(name_map_.size());
    for (auto it = name_map_.begin(); it != name_map_.end(); ++it) {
      anim_names->push_back(it->first);
    }
  }

  /// Get an animation by name. Initialize the animation with this function.
  /// First get the list of `anim_names` with AnimNames(), then initialize
  /// each one, in turn, by calling this function.
  RigAnim* QueryByName(const char* anim_name) {
    auto map_entry = name_map_.find(anim_name);
    if (map_entry == name_map_.end()) return nullptr;
    return &anims_[map_entry->second];
  }

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
  const RigAnim& Query(int object, int anim_idx) const {
    const AnimIndex idx = CalculateIndex(object, anim_idx);
    return anims_[idx];
  }

  /// Get an animation by name. This is slow and should be avoided when
  /// possible.
  const RigAnim* QueryByName(const char* anim_name) const {
    return const_cast<AnimTable*>(this)->QueryByName(anim_name);
  }

 private:
  typedef uint16_t AnimIndex;
  typedef typename std::vector<AnimIndex> AnimList;
  typedef typename std::pair<const char*, AnimIndex> NameToIndex;

  AnimIndex CalculateIndex(int object, int anim_idx) const {
    assert(0 <= object && object < static_cast<int>(indices_.size()));
    const AnimList& list = indices_[object];

    assert(0 <= anim_idx && anim_idx < static_cast<int>(list.size()));
    return list[anim_idx];
  }

  /// Map (`object`, `anim_idx`) tuples to an index into the `anims_` array.
  /// Vector of vectors.
  std::vector<AnimList> indices_;

  /// Map animation name to an animation index.
  /// Used for by-name lookups, and to avoid duplicate copies of the same
  /// animation.
  std::unordered_map<const char*, AnimIndex> name_map_;

  /// Animation data. Contains no duplicate entries, thanks to name_map_.
  std::vector<RigAnim> anims_;
};

}  // namespace motive

#endif  // MOTIVE_ANIM_TABLE_H_
