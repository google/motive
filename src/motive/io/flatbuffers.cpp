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
#include "anim_generated.h"
#include "motive_generated.h"
#include "anim_table_generated.h"
#include "motive/io/flatbuffers.h"
#include "motive/init.h"
#include "motive/anim.h"
#include "motive/anim_table.h"

namespace motive {

// Verify that MatrixOperationType and MatrixOperationTypeFb are identical
// enumerations. Since FlatBuffer support is optional, we must duplicate the
// MatrixOperationType from init.h in matrix_anim.fbx.
#define MOTIVE_VERIFY_MATRIX_OP_ENUM(name)                                    \
  static_assert(                                                              \
      name == static_cast<MatrixOperationType>(MatrixOperationTypeFb_##name), \
      "FlatBuffer and init.h enum do not match")
MOTIVE_VERIFY_MATRIX_OP_ENUM(kInvalidMatrixOperation);
MOTIVE_VERIFY_MATRIX_OP_ENUM(kRotateAboutX);
MOTIVE_VERIFY_MATRIX_OP_ENUM(kRotateAboutY);
MOTIVE_VERIFY_MATRIX_OP_ENUM(kRotateAboutZ);
MOTIVE_VERIFY_MATRIX_OP_ENUM(kTranslateX);
MOTIVE_VERIFY_MATRIX_OP_ENUM(kTranslateY);
MOTIVE_VERIFY_MATRIX_OP_ENUM(kTranslateZ);
MOTIVE_VERIFY_MATRIX_OP_ENUM(kScaleX);
MOTIVE_VERIFY_MATRIX_OP_ENUM(kScaleY);
MOTIVE_VERIFY_MATRIX_OP_ENUM(kScaleZ);
MOTIVE_VERIFY_MATRIX_OP_ENUM(kScaleUniformly);
MOTIVE_VERIFY_MATRIX_OP_ENUM(kNumMatrixOperationTypes);
#undef MOTIVE_VERIFY_OP_ENUM

static void ModularInitFromFlatBuffers(const ModularParameters& params,
                                       ModularInit* init) {
  init->set_modular(params.modular() != 0);
  init->set_range(fpl::Range(params.min(), params.max()));
}

void OvershootInitFromFlatBuffers(const OvershootParameters& params,
                                  OvershootInit* init) {
  ModularInitFromFlatBuffers(*params.base(), init);
  init->set_max_velocity(params.max_velocity());
  init->set_max_delta(params.max_delta());
  Settled1fFromFlatBuffers(*params.at_target(), &init->at_target());
  init->set_accel_per_difference(params.acceleration_per_difference());
  init->set_wrong_direction_multiplier(
      params.wrong_direction_acceleration_multiplier());
  init->set_max_delta_time(params.max_delta_time());
}

void SmoothInitFromFlatBuffers(const SmoothParameters& params,
                               SmoothInit* init) {
  ModularInitFromFlatBuffers(*params.base(), init);
}

void Settled1fFromFlatBuffers(const Settled1fParameters& params,
                              Settled1f* settled) {
  settled->max_velocity = params.max_velocity();
  settled->max_difference = params.max_difference();
}

void MatrixAnimFromFlatBuffers(const MatrixAnimFb& params, MatrixAnim* anim) {
  MatrixOpArray& ops = anim->ops();
  ops.Clear(params.ops()->size());

  // Count the number of splines.
  int num_splines = 0;
  for (auto op = params.ops()->begin(); op != params.ops()->end(); ++op) {
    if (op->value_type() == MatrixOpValueFb_CompactSplineFb) num_splines++;
  }

  // Initialize the output structure with the correct number of splines.
  MatrixAnim::Spline* splines = anim->Construct(num_splines);

  // Loop through each op, adding to the MatrixAnim ops.
  int spline_idx = 0;
  for (auto op = params.ops()->begin(); op != params.ops()->end(); ++op) {
    const MatrixOperationType op_type =
        static_cast<MatrixOperationType>(op->type());

    switch (op->value_type()) {
      case MatrixOpValueFb_CompactSplineFb: {
        const CompactSplineFb* spline_fb =
            reinterpret_cast<const CompactSplineFb*>(op->value());
        MatrixAnim::Spline& s = splines[spline_idx++];

        // Copy the spline data into s.spline.
        // TODO: modify CompactSpline so we can just point at spline data
        //       instead of copying it.
        const fpl::Range y_range(spline_fb->y_range_start(),
                                 spline_fb->y_range_end());
        s.spline.Init(y_range, spline_fb->x_granularity(),
                      spline_fb->nodes()->size());
        for (auto n = spline_fb->nodes()->begin();
             n != spline_fb->nodes()->end(); ++n) {
          s.spline.AddNodeVerbatim(n->x(), n->y(), n->angle());
        }

        // Hold `init` and `playback` data in structures that won't disappear,
        // since these are referenced by pointer.
        s.init = SmoothInit(y_range, spline_fb->modular_arithmetic());
        s.playback = fpl::SplinePlayback(s.spline, 0.0f);
        ops.AddOp(op_type, s.init, s.playback);
        break;
      }

      case MatrixOpValueFb_ConstantOpFb: {
        const ConstantOpFb* const_fb =
            reinterpret_cast<const ConstantOpFb*>(op->value());
        ops.AddOp(op_type, const_fb->y_const());
        break;
      }

      default:
        assert(false);  // Invalid FlatBuffer data.
    }
  }
}

void RigAnimFromFlatBuffers(const RigAnimFb& params, RigAnim* anim) {
  const size_t num_bones = params.matrix_anims()->Length();
  const auto names = params.bone_names();
  const auto parents = params.bone_parents();
  const bool record_names = names != nullptr && names->Length() == num_bones;
  assert(parents != nullptr && parents->Length() == num_bones);

  anim->Init(num_bones, record_names);

  MotiveTime end_time = 0;
  for (size_t i = 0; i < num_bones; ++i) {
    const BoneIndex parent = parents->Get(i);
    const char* name = record_names ? names->Get(i)->c_str() : "";
    MatrixAnim& m = anim->InitMatrixAnim(i, parent, name);
    MatrixAnimFromFlatBuffers(*params.matrix_anims()->Get(i), &m);

    end_time = std::max(end_time, m.ops().end_time());
  }
  anim->set_end_time(end_time);
}

void AnimTable::InitFromFlatBuffers(const AnimTableFb& params) {
  // Allocate the index arrays.
  indices_.resize(params.lists()->size());
  for (flatbuffers::uoffset_t i = 0; i < indices_.size(); ++i) {
    indices_[i].resize(params.lists()->Get(i)->anim_files()->Length());
  }

  // Create the name map and allocate indices into anims_.
  for (flatbuffers::uoffset_t i = 0; i < indices_.size(); ++i) {
    auto files_fb = params.lists()->Get(i)->anim_files();
    AnimList& list = indices_[i];

    for (flatbuffers::uoffset_t j = 0; j < list.size(); ++j) {
      const char* anim_name = files_fb->Get(j)->c_str();

      // If this anim already exists, re-use it.
      auto existing = name_map_.find(anim_name);
      if (existing != name_map_.end()) {
        list[j] = existing->second;
        continue;
      }

      // Allocate a new index and insert into name map.
      const AnimIndex new_idx = static_cast<AnimIndex>(name_map_.size());
      list[j] = new_idx;
      name_map_.insert(NameToIndex(anim_name, new_idx));
    }
  }

  // Initialize the array of animations. These must be loaded separately.
  // See LoadAnimTableAnimations() for an example.
  anims_.resize(name_map_.size());
}

const char* LoadAnimTableAnimations(AnimTable* anim_table,
                                    LoadFileFn* load_fn) {
  std::vector<const char*> anim_names;
  anim_table->AnimNames(&anim_names);

  // Loop through each name
  for (auto it = anim_names.begin(); it != anim_names.end(); ++it) {
    const char* anim_name = *it;

    // Load the animation file.
    std::string anim_buf;
    const bool load_success = load_fn(anim_name, &anim_buf);
    if (!load_success) return anim_name;

    // Initialize the animation file into `anim_table`.
    const RigAnimFb* anim_fb = GetRigAnimFb(anim_buf.c_str());
    RigAnim* anim = anim_table->QueryByName(anim_name);
    RigAnimFromFlatBuffers(*anim_fb, anim);
  }

  return nullptr;
}

}  // namespace motive
