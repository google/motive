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
#include "matrix_anim_generated.h"
#include "motive_generated.h"
#include "motive/io/flatbuffers.h"
#include "motive/init.h"
#include "motive/anim.h"

namespace motive {

// Verify that MatrixOperationType and MatrixOperationTypeFb are identical
// enumerations. Since FlatBuffer support is optional, we must duplicate the
// MatrixOperationType from init.h in matrix_anim.fbx.
#define MOTIVE_VERIFY_MATRIX_OP_ENUM(name) \
    static_assert(name == static_cast<MatrixOperationType>( \
                  MatrixOperationTypeFb_##name), \
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
  MatrixInit& init = anim->init();
  init.Clear(params.ops()->size());

  for (auto op = params.ops()->begin(); op != params.ops()->end(); ++op) {
    const MatrixOperationType op_type =
        static_cast<MatrixOperationType>(op->type());

    switch (op->value_type()) {
      case MatrixOpValueFb_CompactSplineFb: {
        const CompactSplineFb* spline_fb =
            reinterpret_cast<const CompactSplineFb*>(op->value());
        fpl::CompactSpline& spline = anim->AllocSpline();

        fpl::Range y_range(spline_fb->y_range_start(),
                           spline_fb->y_range_end());
        spline.Init(y_range, spline_fb->x_granularity(),
                    spline_fb->nodes()->size());
        for (auto n = spline_fb->nodes()->begin();
             n != spline_fb->nodes()->end(); ++n) {
          spline.AddNodeVerbatim(n->x(), n->y(), n->angle());
        }

        init.AddOp(op_type,
                   SmoothInit(y_range, spline_fb->modular_arithmetic()),
                   fpl::SplinePlayback(spline));
        break;
      }

      case MatrixOpValueFb_ConstantOpFb: {
        const ConstantOpFb* const_fb =
            reinterpret_cast<const ConstantOpFb*>(op->value());
        init.AddOp(op_type, const_fb->y_const());
        break;
      }

      default:
        assert(false);  // Invalid FlatBuffer data.
    }
  }
}

}  // namespace motive
