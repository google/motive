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

static const MatrixOperationInit* FindOpInit(const MatrixOpArray::OpVector& ops,
                                             MatrixOperationType op) {
  for (size_t i = 0; i < ops.size(); ++i) {
    if (ops[i].type == op) return &ops[i];
  }
  return nullptr;
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
      const MatrixOpId id = it->first;
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

AnimTable::AnimIndex AnimTable::AddAnimName(const char* anim_name) {
  // Allow empty indices in the animation table.
  if (anim_name == nullptr || *anim_name == '\0') return kInvalidAnimIndex;

  // If this anim already exists, re-use it.
  auto existing = name_map_.find(anim_name);
  if (existing != name_map_.end()) return existing->second;

  // Allocate a new index and insert into name map.
  const AnimIndex new_idx = static_cast<AnimIndex>(name_map_.size());
  name_map_.insert(NameToIndex(anim_name, new_idx));
  return new_idx;
}

void AnimTable::InitNameMap(const AnimFileNames& anim_file_names) {
  // Allocate the index arrays.
  indices_.resize(anim_file_names.size());
  defining_anims_.resize(anim_file_names.size());
  for (size_t i = 0; i < anim_file_names.size(); ++i) {
    indices_[i].resize(anim_file_names[i].size());
  }

  // Create the name map and allocate indices into anims_.
  for (size_t i = 0; i < indices_.size(); ++i) {
    // Record the indices of each animation for this object.
    const std::vector<std::string>& anims = anim_file_names[i];
    AnimList& list = indices_[i];
    for (size_t j = 0; j < anims.size(); ++j) {
      list[j] = AddAnimName(anims[j].c_str());
    }
  }

  // Initialize the array of animations. These must be loaded separately.
  anims_.resize(name_map_.size());
}

bool AnimTable::LoadAnimations(LoadRigAnimFn* load_fn) {
  std::vector<const char*> anim_names;
  AnimNames(&anim_names);

  // Loop through each name.
  std::string scratch_buf;
  for (auto it = anim_names.begin(); it != anim_names.end(); ++it) {
    const char* anim_name = *it;

    // Load the animation file.
    const RigAnimFb* anim_fb = load_fn(anim_name, &scratch_buf);
    if (anim_fb == nullptr) return false;

    // Initialize the animation file into `anim_table`.
    RigAnim* anim = QueryByName(anim_name);
    RigAnimFromFlatBuffers(*anim_fb, anim_name, anim);
  }
  return true;
}

bool AnimTable::InitFromFlatBuffers(const AnimTableFb& params,
                                    LoadRigAnimFn* load_fn) {
  AnimFileNames anim_file_names;
  anim_file_names.resize(params.lists()->size());

  // Convert the vector of vectors-of-strings from Flatbuffers to C++.
  for (flatbuffers::uoffset_t i = 0; i < anim_file_names.size(); ++i) {
    auto files_fb = params.lists()->Get(i)->anim_files();
    anim_file_names[i].resize(files_fb->size());

    for (flatbuffers::uoffset_t j = 0; j < files_fb->size(); ++j) {
      anim_file_names[i][j] = files_fb->Get(j)->str();
    }
  }

  return InitFromAnimFileNames(anim_file_names, load_fn);
}

bool AnimTable::InitFromAnimFileNames(const AnimFileNames& anim_file_names,
                                      LoadRigAnimFn* load_fn) {
  // Deserialize `params` into the vector-of-vectors of animations.
  InitNameMap(anim_file_names);

  // Load each FlatBuffer animation file in turn using the callback `load_fn`.
  const bool load_ok = LoadAnimations(load_fn);
  if (!load_ok) return false;

  // Now that all animations have been loaded, calculate defining animation,
  // which is the union of all the animations on an object.
  CalculateDefinineAnims();
  return true;
}

bool AnimTable::InitFromAnimFileNames(
    const std::vector<std::string>& anim_file_names_one_object,
    LoadRigAnimFn* load_fn) {
  AnimFileNames anim_file_names(1);
  anim_file_names[0] = anim_file_names_one_object;
  return InitFromAnimFileNames(anim_file_names, load_fn);
}

/// Enumerate all animations that are held in this table. Animations are
/// listed only once. Multiple indices may map to one animation.
/// Valid after InitFromFlatBuffers() is called.
void AnimTable::AnimNames(std::vector<const char*>* anim_names) const {
  anim_names->clear();
  anim_names->reserve(name_map_.size());
  for (auto it = name_map_.begin(); it != name_map_.end(); ++it) {
    anim_names->push_back(it->first.c_str());
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
    anims[num_anims++] = &anims_[list[j]];
  }
  return num_anims;
}

void AnimTable::CalculateDefinineAnims() {
  std::vector<const RigAnim*> anims(MaxAnimIndex());
  for (int object = 0; object < NumObjects(); ++object) {
    const size_t num_anims = GatherObjectAnims(object, &anims[0]);
    CreateDefiningAnim(&anims[0], num_anims, &defining_anims_[object]);
  }
}

}  // namespace motive
