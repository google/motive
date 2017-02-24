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
struct AnimListFb;
struct AnimTableEmbeddedFb;
struct AnimListEmbeddedFb;
struct RigAnimFb;
class TableDescriberInterface;

/// @class AnimTable
/// @brief Hold animation lists for several object types.
///
/// A list of lists of animations.
///
/// The outer list is indexed by `object`, the meaning of which is set by
/// the user, but is most likely an enum for the character being animated
/// (e.g. turtle, zombie, happy person, tall person, etc.).
///
/// The inner list is indexed by `anim_idx`, the meaning of which is also set
/// by the user, but is most likely an enum for the action being animated
/// (e.g. idle, walk forward, jump, etc.).
///
/// Duplicate animations are only loaded once. This allows different objects
/// to use the same animations without any memory overhead.
class AnimTable {
 public:
  // Array of animations corresponding to `anim_idx`.
  // The meaning of `anim_idx` is set by the user, but is most likely an enum
  // for the action being animated (e.g. idle, walk forward, jump, etc.).
  typedef std::vector<std::string> ListFileNames;

  // Array of arrays of animations corresponding to `object`.
  // The meaning of `object` is set by the user, but is most likely an enum
  // for the character being animated (e.g. turtle, zombie, happy person,
  // tall person, etc.).
  typedef std::vector<ListFileNames> TableFileNames;

  /// Callback that loads `file_name` and returns a pointer to the raw data.
  /// Optionally, the function can load the file into `scratch_buf`,
  /// and then return `scratch_buf.c_str()`.
  /// If the load fails, should return nullptr.
  ///
  /// Note: Motive does not make assumptions on file io, so the caller must
  ///       provide this function.
  ///
  typedef const char* LoadFn(const char* file_name, std::string* scratch_buf);

  ~AnimTable();

  /// Load the AnimTable specified in the FlatBuffer `params`.
  /// For each animation in the AnimTable, `load_fn` is called to get the
  /// to load the individual animation files, if they're not embedded in
  /// table_fb. If they are embedded then `load_fn` can be nullptr.
  /// `table_fb` can be discarded after this call.
  bool InitFromFlatBuffers(const AnimTableFb& table_fb, LoadFn* load_fn);

  /// Load the animations specified in the FlatBuffer `list_fb`.
  /// All of the loaded animations are for object 0.
  bool InitFromFlatBuffers(const AnimListFb& list_fb, LoadFn* load_fn);

  /// Load the AnimTable specified in the vector of vectors.
  /// The top level vector represents the `object` index.
  /// The bottom level vector represents the `anim_idx`.
  bool InitFromAnimFileNames(const TableFileNames& table_names,
                             LoadFn* load_fn);

  /// Load the AnimTable for only one `object`.
  bool InitFromAnimFileNames(const ListFileNames& list_names, LoadFn* load_fn);

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
    return idx == kInvalidAnimIndex ? nullptr : anims_[idx];
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

  /// Return size of the bottom-most vector. Recall that AnimTable is a vector
  /// of vectors.
  int NumAnims(int object) const {
    return static_cast<int>(indices_[object].size());
  }

  /// Return the number of animations for which we've allocated memory.
  /// Internally, we avoid duplicating animations.
  int NumUniqueAnims() const { return static_cast<int>(anims_.size()); }

 private:
  typedef uint16_t AnimIndex;
  typedef std::vector<AnimIndex> AnimList;
  typedef std::pair<std::string, AnimIndex> NameToIndex;
  static const AnimIndex kInvalidAnimIndex = static_cast<AnimIndex>(-1);

  bool Load(TableDescriberInterface* describer, LoadFn* load_fn);
  void AnimNames(std::vector<const char*>* anim_names) const;
  size_t MaxAnimIndex() const;
  size_t GatherObjectAnims(int object, const RigAnim** anims) const;
  void CalculateDefiningAnims();

  RigAnim* QueryByName(const char* anim_name) {
    auto map_entry = name_map_.find(anim_name);
    if (map_entry == name_map_.end()) return nullptr;
    return anims_[map_entry->second];
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
  std::vector<RigAnim*> anims_;
};

}  // namespace motive

#endif  // MOTIVE_ANIM_TABLE_H_
