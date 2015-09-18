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
#include "anim_generated.h"
#include "anim_table_generated.h"
#include "motive/anim_table.h"
#include "motive/io/flatbuffers.h"

using fpl::Range;

namespace motive {

static const MatrixOperationType kCanonicalRigAnimOps[] = {
  kTranslateX,
  kTranslateY,
  kTranslateZ,
  kRotateAboutZ,
  kRotateAboutY,
  kRotateAboutX,
  kScaleX,
  kScaleY,
  kScaleZ,
  kScaleUniformly,
};

#if !defined(NDEBUG)  // Since only called in assert statement.
static bool IsCanonicalAnim(const RigAnim& anim) {
  for (BoneIndex i = 0; i < anim.NumBones(); ++i) {
    const MatrixOpArray::OpVector& ops = anim.Anim(i).ops().ops();
    size_t canon_idx = 0;
    for (size_t j = 0; j < ops.size(); ++j) {
      const MatrixOperationInit& op_init = ops[j];
      while (kCanonicalRigAnimOps[canon_idx] != op_init.type) {
        canon_idx++;
        if (canon_idx == MOTIVE_ARRAY_SIZE(kCanonicalRigAnimOps))
          return false;
      }
      canon_idx++;
    }
  }
  return true;
}
#endif

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
  // This function only works for animations that follow the canonical order.
  for (size_t i = 0; i < num_anims; ++i) {
    assert(IsCanonicalAnim(*anims[i]));
  }

  // Get the bone hierarchy that covers all the hierarchies in `anims`.
  const RigAnim* complete_rig = FindCompleteRig(anims, num_anims);
  const BoneIndex num_bones = complete_rig->NumBones();
  const BoneIndex* parents = complete_rig->bone_parents();
  defining_anim->Init("defining_anim", num_bones, true);

  // For each bone, consider adding each operation in the canonical operations.
  for (BoneIndex j = 0; j < num_bones; ++j) {
    // Start initializing this bone.
    MatrixAnim& matrix_anim = defining_anim->InitMatrixAnim(
        j, parents[j], complete_rig->BoneName(j));
    MatrixOpArray& ops = matrix_anim.ops();

    // Loop through all possible operations for this bone.
    Range ranges[MOTIVE_ARRAY_SIZE(kCanonicalRigAnimOps)];
    for (size_t k = 0; k < MOTIVE_ARRAY_SIZE(kCanonicalRigAnimOps); ++k) {
      const MatrixOperationType op = kCanonicalRigAnimOps[k];

      // Loop over all anims, checking to see if they have this operation.
      for (size_t i = 0; i < num_anims; ++i) {
        if (j >= anims[i]->NumBones()) continue;

        const MatrixOpArray::OpVector& ops = anims[i]->Anim(j).ops().ops();
        const MatrixOperationInit* op_init = FindOpInit(ops, op);
        if (op_init == nullptr) continue;

        if (op_init->init == nullptr) {
          ranges[k] = ranges[k].Include(op_init->initial_value);
        } else {
          const SmoothInit* smooth_init =
              static_cast<const SmoothInit*>(op_init->init);
          ranges[k] = Range::Union(ranges[k], smooth_init->range());
        }
      }
    }

    // Count the number of ranges that need init parameters.
    int num_ops_with_init = 0;
    for (size_t k = 0; k < MOTIVE_ARRAY_SIZE(kCanonicalRigAnimOps); ++k) {
      if (ranges[k].Valid() && ranges[k].Length() > 0.0f) num_ops_with_init++;
    }
    MatrixAnim::Spline* splines = matrix_anim.Construct(num_ops_with_init);

    // Create the array of matrix operations.
    int num_ops_inited = 0;
    for (size_t k = 0; k < MOTIVE_ARRAY_SIZE(kCanonicalRigAnimOps); ++k) {
      if (!ranges[k].Valid()) continue;

      // If this operation exists, add it to the `defining_anim`.
      const MatrixOperationType op = kCanonicalRigAnimOps[k];
      if (ranges[k].Length() == 0.0f) {
        // If there is only one value for an operation, add it as a const.
        const float const_value = ranges[k].start();
        ops.AddOp(op, const_value);
      } else {
        // Otherwise, add it as an animated parameter.
        // The range is modular for rotate operations, but not for scale or
        // translate operations.
        const bool modular = ModularOp(op);
        const Range& op_range = RangeOfOp(op, ranges[k]);
        splines[num_ops_inited].init = SmoothInit(op_range, modular);
        ops.AddOp(op, splines[num_ops_inited].init);
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

void AnimTable::InitNameMapFromFlatBuffers(const AnimTableFb& params) {
  // Allocate the index arrays.
  indices_.resize(params.lists()->size());
  defining_anims_.resize(params.lists()->size());
  for (flatbuffers::uoffset_t i = 0; i < indices_.size(); ++i) {
    indices_[i].resize(params.lists()->Get(i)->anim_files()->Length());
  }

  // Create the name map and allocate indices into anims_.
  for (flatbuffers::uoffset_t i = 0; i < indices_.size(); ++i) {
    const AnimListFb* anim_list = params.lists()->Get(i);

    // Record the indices of each animation for this object.
    auto files_fb = anim_list->anim_files();
    AnimList& list = indices_[i];
    for (flatbuffers::uoffset_t j = 0; j < list.size(); ++j) {
      const char* anim_name = files_fb->Get(j)->c_str();
      list[j] = AddAnimName(anim_name);
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
  // Deserialize `params` into the vector-of-vectors of animations.
  InitNameMapFromFlatBuffers(params);

  // Load each FlatBuffer animation file in turn using the callback `load_fn`.
  const bool load_ok = LoadAnimations(load_fn);
  if (!load_ok) return false;

  // Now that all animations have been loaded, calculate defining animation,
  // which is the union of all the animations on an object.
  CalculateDefinineAnims();
  return true;
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
