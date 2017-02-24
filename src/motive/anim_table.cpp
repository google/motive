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
#include <map>

#include "anim_generated.h"
#include "anim_table_generated.h"
#include "motive/anim_table.h"
#include "motive/io/flatbuffers.h"

using motive::Range;

namespace motive {

// Wrap the source data for an AnimTable.
// Allows AnimTables to be loaded from many different data sources.
class TableDescriberInterface {
 public:
  virtual ~TableDescriberInterface() {}
  virtual int NumObjects() const = 0;
  virtual int NumAnims(int object) const = 0;
  virtual const char* SourceFileName(int object, int anim_idx) const = 0;
  virtual const RigAnimFb* SourceRigAnimFb(int object, int anim_idx) const = 0;
};

static int AnimListLen(const AnimListFb* list) {
  if (list == nullptr) return 0;

  // Handle `anims` interface (with union).
  if (list->anims() != nullptr) return static_cast<int>(list->anims()->size());

  // Handle `anim_files` interface.
  if (list->anim_files() != nullptr)
    return static_cast<int>(list->anim_files()->size());

  return 0;
}

static const char* AnimListFileName(const AnimListFb* list, int anim_idx) {
  if (list == nullptr) return nullptr;

  // Handle `anims` interface (with union).
  if (list->anims() != nullptr) {
    const AnimSource* source = list->anims()->Get(anim_idx);
    if (source->u_type() != AnimSourceUnion_AnimSourceFileName) return nullptr;
    return reinterpret_cast<const AnimSourceFileName*>(source->u())
        ->file_name()
        ->c_str();
  }

  // Handle `anim_files` interface.
  if (list->anim_files() != nullptr)
    return list->anim_files()->Get(anim_idx)->c_str();

  return nullptr;
}

static const RigAnimFb* AnimListRigAnimFb(const AnimListFb* list,
                                          int anim_idx) {
  if (list == nullptr) return nullptr;

  // Handle `anims` interface (with union).
  if (list->anims() != nullptr) {
    const AnimSource* source = list->anims()->Get(anim_idx);
    if (source->u_type() != AnimSourceUnion_AnimSourceEmbedded) return nullptr;
    return reinterpret_cast<const AnimSourceEmbedded*>(source->u())->embedded();
  }

  return nullptr;
}

class AnimTableFbDescriber : public TableDescriberInterface {
 public:
  explicit AnimTableFbDescriber(const AnimTableFb& table_fb)
      : table_fb_(&table_fb) {}
  virtual int NumObjects() const {
    return table_fb_->lists() == nullptr
               ? 0
               : static_cast<int>(table_fb_->lists()->size());
  }
  virtual int NumAnims(int object) const { return AnimListLen(List(object)); }
  virtual const char* SourceFileName(int object, int anim_idx) const {
    return AnimListFileName(List(object), anim_idx);
  }
  virtual const RigAnimFb* SourceRigAnimFb(int object, int anim_idx) const {
    return AnimListRigAnimFb(List(object), anim_idx);
  }

 protected:
  const AnimListFb* List(int object) const {
    return table_fb_->lists() == nullptr ? nullptr
                                         : table_fb_->lists()->Get(object);
  }

  const AnimTableFb* table_fb_;
};

class AnimListFbDescriber : public TableDescriberInterface {
 public:
  explicit AnimListFbDescriber(const AnimListFb& list_fb)
      : list_fb_(&list_fb) {}
  virtual int NumObjects() const { return AnimListLen(list_fb_) == 0 ? 0 : 1; }
  virtual int NumAnims(int /*object*/) const { return AnimListLen(list_fb_); }
  virtual const char* SourceFileName(int /*object*/, int anim_idx) const {
    return AnimListFileName(list_fb_, anim_idx);
  }
  virtual const RigAnimFb* SourceRigAnimFb(int /*object*/, int anim_idx) const {
    return AnimListRigAnimFb(list_fb_, anim_idx);
  }

 protected:
  const AnimListFb* list_fb_;
};

class TableFileNamesDescriber : public TableDescriberInterface {
 public:
  explicit TableFileNamesDescriber(const AnimTable::TableFileNames& table_names)
      : table_names_(&table_names) {}
  virtual int NumObjects() const {
    return static_cast<int>(table_names_->size());
  }
  virtual int NumAnims(int object) const {
    return static_cast<int>((*table_names_)[object].size());
  }
  virtual const char* SourceFileName(int object, int anim_idx) const {
    return (*table_names_)[object][anim_idx].c_str();
  }
  virtual const RigAnimFb* SourceRigAnimFb(int /*object*/,
                                           int /*anim_idx*/) const {
    return nullptr;
  }

 protected:
  const AnimTable::TableFileNames* table_names_;
};

class ListFileNamesDescriber : public TableDescriberInterface {
 public:
  explicit ListFileNamesDescriber(const AnimTable::ListFileNames& list_names)
      : list_names_(&list_names) {}
  virtual int NumObjects() const { return list_names_->size() == 0 ? 0 : 1; }
  virtual int NumAnims(int /*object*/) const {
    return static_cast<int>(list_names_->size());
  }
  virtual const char* SourceFileName(int /*object*/, int anim_idx) const {
    return (*list_names_)[anim_idx].c_str();
  }
  virtual const RigAnimFb* SourceRigAnimFb(int /*object*/,
                                           int /*anim_idx*/) const {
    return nullptr;
  }

 protected:
  const AnimTable::ListFileNames* list_names_;
};

AnimTable::~AnimTable() {
  for (size_t i = 0; i < anims_.size(); ++i) {
    delete anims_[i];
    anims_[i] = nullptr;
  }
}

bool AnimTable::InitFromFlatBuffers(const AnimTableFb& table_fb,
                                    LoadFn* load_fn) {
  AnimTableFbDescriber describer(table_fb);
  return Load(&describer, load_fn);
}

bool AnimTable::InitFromFlatBuffers(const AnimListFb& list_fb,
                                    LoadFn* load_fn) {
  AnimListFbDescriber describer(list_fb);
  return Load(&describer, load_fn);
}

bool AnimTable::InitFromAnimFileNames(const TableFileNames& table_names,
                                      LoadFn* load_fn) {
  TableFileNamesDescriber describer(table_names);
  return Load(&describer, load_fn);
}

bool AnimTable::InitFromAnimFileNames(const ListFileNames& list_names,
                                      LoadFn* load_fn) {
  ListFileNamesDescriber describer(list_names);
  return Load(&describer, load_fn);
}

bool AnimTable::Load(TableDescriberInterface* describer, LoadFn* load_fn) {
  std::string scratch_buf;
  bool success = true;

  // An AnimTable is a list-of-lists. The outside list is indexed by object.
  // Loop through each object (e.g. character type).
  const int num_objects = describer->NumObjects();
  indices_.resize(num_objects);
  defining_anims_.resize(num_objects);
  for (int object = 0; object < num_objects; ++object) {
    const int num_anims = describer->NumAnims(object);
    indices_[object].resize(num_anims);

    // The inside list is animations for the given object.
    // Loop through all animations, loading animations or referencing animations
    // that have already been loaded.
    AnimList& list = indices_[object];
    for (int anim_idx = 0; anim_idx < num_anims; ++anim_idx) {
      // Initialize this anim_idx to point to no data.
      list[anim_idx] = kInvalidAnimIndex;

      const RigAnimFb* anim_fb = describer->SourceRigAnimFb(object, anim_idx);
      const char* anim_name = anim_fb != nullptr
                                  ? anim_fb->name()->c_str()
                                  : describer->SourceFileName(object, anim_idx);

      // Case 1: source data is empty.
      if (anim_name == nullptr || anim_name[0] == '\0') continue;

      // Case 2: source data has already been processed.
      auto existing = name_map_.find(anim_name);
      if (existing != name_map_.end()) {
        list[anim_idx] = existing->second;
        continue;
      }

      // Case 3: load source data.
      if (anim_fb == nullptr) {
        const char* anim_buf = load_fn(anim_name, &scratch_buf);
        anim_fb = anim_buf == nullptr ? nullptr : GetRigAnimFb(anim_buf);

        // Error loading file. Keep loading but return false.
        if (anim_fb == nullptr) {
          success = false;
          continue;
        }
      }

      // Create RigAnim from FlatBuffer.
      const AnimIndex new_idx = static_cast<AnimIndex>(anims_.size());
      RigAnim* anim = new RigAnim();
      RigAnimFromFlatBuffers(*anim_fb, anim);
      anims_.push_back(anim);

      // Insert index into name map so that we only load this anim once.
      name_map_.insert(NameToIndex(anim_name, new_idx));
      list[anim_idx] = new_idx;
    }
  }

  // Now that all animations have been loaded, calculate defining animation,
  // which is the union of all the animations on an object.
  CalculateDefiningAnims();
  return success;
}

static const RigAnim* FindCompleteRig(const RigAnim** anims, size_t num_anims) {
  // We assume that that animation with the most bones has all the bones.
  // Not necessarily true, since all animations could animate a subset of the
  // rig. We should really do a deeper union of the bone hierarchy.
  assert(num_anims > 0);
  BoneIndex max_bone_index = 0;
  for (size_t i = 1; i < num_anims; ++i) {
    if (anims[i]->NumBones() > anims[max_bone_index]->NumBones()) {
      max_bone_index = static_cast<BoneIndex>(i);
    }
  }
  return anims[max_bone_index];
}

static void CreateDefiningAnim(const RigAnim** anims, size_t num_anims,
                               RigAnim* defining_anim) {
  // Get the bone hierarchy that covers all the hierarchies in `anims`.
  const RigAnim* complete_rig = FindCompleteRig(anims, num_anims);
  const BoneIndex num_bones = complete_rig->NumBones();
  const BoneIndex* parents = complete_rig->bone_parents();
  defining_anim->Init("defining_anim", num_bones, true);

  // For each bone, consider adding each operation in the canonical operations.
  for (BoneIndex j = 0; j < num_bones; ++j) {
    // Start initializing this bone.
    MatrixAnim& matrix_anim =
        defining_anim->InitMatrixAnim(j, parents[j], complete_rig->BoneName(j));
    MatrixOpArray& ops = matrix_anim.ops();

    // Create a sorted map from operation id to the range of values that it
    // takes.
    struct OpRange {
      MatrixOperationType op;
      Range range;
    };
    std::map<int, OpRange> id_to_range;

    for (size_t i = 0; i < num_anims; ++i) {
      if (j >= anims[i]->NumBones()) continue;
      const MatrixOpArray::OpVector& opv = anims[i]->Anim(j).ops().ops();
      for (auto op_it = opv.begin(); op_it != opv.end(); ++op_it) {
        const MatrixOperationInit& op_init = *op_it;
        auto& range_it = id_to_range[op_init.id];

        // Bone `j` with matrix op id `op_init.id` has two animations with
        // mismatched operations. Things will go strangely when a kTranslateX
        // is blended with a kScaleZ, for example.
        assert(range_it.op == kInvalidMatrixOperation ||
               range_it.op == op_init.type);

        range_it.op = op_init.type;
        Range& range = range_it.range;
        if (op_init.init == nullptr) {
          range = range.Include(op_init.initial_value);
        } else {
          range = Range::Full();
        }
      }
    }

    // Count the number of ranges that need init parameters.
    int num_ops_with_init = 0;
    for (auto it = id_to_range.begin(); it != id_to_range.end(); ++it) {
      if (it->second.range.Length() > 0.0f) num_ops_with_init++;
    }
    MatrixAnim::Spline* splines = matrix_anim.Construct(num_ops_with_init);

    // Create the array of matrix operations.
    int num_ops_inited = 0;
    for (auto it = id_to_range.begin(); it != id_to_range.end(); ++it) {
      // If this operation exists, add it to the `defining_anim`.
      const MatrixOpId id = static_cast<MatrixOpId>(it->first);
      const MatrixOperationType op = it->second.op;
      const Range& range = it->second.range;
      if (range.Length() == 0.0f) {
        // If there is only one value for an operation, add it as a const.
        const float const_value = range.start();
        ops.AddOp(id, op, const_value);
      } else {
        // Otherwise, add it as an animated parameter.
        // The range is modular for rotate operations, but not for scale or
        // translate operations.
        const Range& op_range = RangeOfOp(op);
        splines[num_ops_inited].init = SplineInit(op_range);
        ops.AddOp(id, op, splines[num_ops_inited].init);
        num_ops_inited++;
      }
    }
  }
}

size_t AnimTable::MaxAnimIndex() const {
  size_t max_anim_idx = 0;
  for (size_t i = 0; i < indices_.size(); ++i) {
    max_anim_idx = std::max(max_anim_idx, indices_[i].size());
  }
  return max_anim_idx;
}

size_t AnimTable::GatherObjectAnims(int object, const RigAnim** anims) const {
  const AnimList& list = indices_[object];
  size_t num_anims = 0;
  for (size_t j = 0; j < list.size(); ++j) {
    if (list[j] == kInvalidAnimIndex) continue;
    anims[num_anims++] = anims_[list[j]];
  }
  return num_anims;
}

void AnimTable::CalculateDefiningAnims() {
  std::vector<const RigAnim*> anims(MaxAnimIndex());
  for (int object = 0; object < NumObjects(); ++object) {
    const size_t num_anims = GatherObjectAnims(object, &anims[0]);
    if (num_anims == 0) continue;

    CreateDefiningAnim(&anims[0], num_anims, &defining_anims_[object]);
  }
}

}  // namespace motive
