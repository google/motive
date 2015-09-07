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

namespace motive {

// TODO: Make these configurable per call, instead of const in the anim.
static const float kAnimStartTime = 0.0f;
static const float kAnimPlaybackRate = 1.0f;
static const float kAnimBlendTime = 200.0f;

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

void MatrixAnimFromFlatBuffers(const MatrixAnimFb& params, bool repeat,
                               MatrixAnim* anim) {
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
        s.init =
            SmoothInit(y_range, spline_fb->modular_arithmetic() ? true : false);
        s.playback = fpl::SplinePlayback(s.spline, kAnimStartTime, repeat,
                                         kAnimPlaybackRate, kAnimBlendTime);
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

  anim->Init(static_cast<motive::BoneIndex>(num_bones), record_names);

  MotiveTime end_time = 0;
  for (size_t i = 0; i < num_bones; ++i) {
    const BoneIndex parent = parents->Get(i);
    const char* name = record_names ? names->Get(i)->c_str() : "";
    MatrixAnim& m =
        anim->InitMatrixAnim(static_cast<motive::BoneIndex>(i), parent, name);
    MatrixAnimFromFlatBuffers(*params.matrix_anims()->Get(i), params.repeat(),
                              &m);

    end_time = std::max(end_time, m.ops().end_time());
  }
  anim->set_end_time(end_time);
}

}  // namespace motive
