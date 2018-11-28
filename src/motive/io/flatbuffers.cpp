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
#include "motive/io/flatbuffers.h"

#include "anim_generated.h"
#include "anim_table_generated.h"
#include "motive/overshoot_init.h"
#include "motive/matrix_anim.h"
#include "motive/rig_anim.h"
#include "motive_generated.h"

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

void OvershootInitFromFlatBuffers(const OvershootParameters& params,
                                  OvershootInit* init) {
  init->set_modular(params.base()->modular() != 0);
  init->set_range(Range(params.base()->min(), params.base()->max()));
  init->set_max_velocity(params.max_velocity());
  init->set_max_delta(params.max_delta());
  Settled1fFromFlatBuffers(*params.at_target(), &init->at_target());
  init->set_accel_per_difference(params.acceleration_per_difference());
  init->set_wrong_direction_multiplier(
      params.wrong_direction_acceleration_multiplier());
  init->set_max_delta_time(params.max_delta_time());
}

void SplineInitFromFlatBuffers(const SplineParameters& params,
                               SplineInit* init) {
  init->set_range(params.base()->modular() != 0
                      ? Range(params.base()->min(), params.base()->max())
                      : Range());
}

void Settled1fFromFlatBuffers(const Settled1fParameters& params,
                              Settled1f* settled) {
  settled->max_velocity = params.max_velocity();
  settled->max_difference = params.max_difference();
}

void MatrixAnimFromFlatBuffers(const MatrixAnimFb& params, MatrixAnim* anim) {
  std::vector<MatrixOperationInit>& ops = anim->ops();
  ops.clear();
  ops.reserve(params.ops()->size());

  // Count the number of splines.
  int num_splines = 0;
  for (auto op = params.ops()->begin(); op != params.ops()->end(); ++op) {
    if (op->value_type() == MatrixOpValueFb_CompactSplineFb ||
        op->value_type() == MatrixOpValueFb_CompactSplineFloatFb) {
      num_splines++;
    }
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

        // Hold `init` data in structures that won't disappear, since these are
        // referenced by pointer.
        const Range& op_range = RangeOfOp(op_type);
        s.init = SplineInit(op_range);

        if (spline_fb) {
          const CompactSplineIndex num_spline_nodes =
              static_cast<CompactSplineIndex>(spline_fb->nodes()->size());
          s.spline = CompactSpline::Create(num_spline_nodes);

          // Copy the spline data into s.spline.
          // TODO: modify CompactSpline so we can just point at spline data
          //       instead of copying it.
          const Range y_range(spline_fb->y_range_start(),
                              spline_fb->y_range_end());
          s.spline->Init(y_range, spline_fb->x_granularity());
          for (auto n = spline_fb->nodes()->begin();
               n != spline_fb->nodes()->end(); ++n) {
            s.spline->AddNodeVerbatim(n->x(), n->y(), n->angle());
          }
          assert(s.spline->num_nodes() == s.spline->max_nodes());
          ops.emplace_back(op->id(), op_type, s.init, *s.spline);
        } else {
          ops.emplace_back(op->id(), op_type, s.init);
        }
        break;
      }

      case MatrixOpValueFb_CompactSplineFloatFb: {
        const CompactSplineFloatFb* spline_fb =
            reinterpret_cast<const CompactSplineFloatFb*>(op->value());
        MatrixAnim::Spline& s = splines[spline_idx++];

        // Hold `init` data in structures that won't disappear, since these are
        // referenced by pointer.
        const Range& op_range = RangeOfOp(op_type);
        s.init = SplineInit(op_range);
        if (spline_fb) {
          const CompactSplineIndex num_spline_nodes =
              static_cast<CompactSplineIndex>(spline_fb->nodes()->size());
          s.spline = CompactSpline::Create(num_spline_nodes);

          // n->time() is in seconds, but CompactSplines work in milliseconds.
          const float end_x = spline_fb->nodes()->end()->time() * 1000.f;
          const float x_granularity =
              CompactSpline::RecommendXGranularity(end_x);

          Range y_range = Range::Empty();
          for (auto n = spline_fb->nodes()->begin();
               n != spline_fb->nodes()->end(); ++n) {
            y_range = y_range.Include(n->value());
          }

          // Initialize the spline before adding nodes.
          s.spline->Init(y_range, x_granularity);

          // Create the CompactSpline in the memory buffer, then copy the spline
          // data into s.spline.
          // TODO: modify CompactSpline so we can just point at spline data
          //       instead of copying it.
          for (auto n = spline_fb->nodes()->begin();
               n != spline_fb->nodes()->end(); ++n) {
            // n->time() is in seconds, but CompactSplines work in milliseconds.
            // Derivatives need to be scaled to the milliseconds as well.
            s.spline->AddNode(n->time() * 1000.f, n->value(),
                              n->derivative() / 1000.f,
                              kAddWithoutModification);
          }
          assert(s.spline->num_nodes() == s.spline->max_nodes());
          ops.emplace_back(op->id(), op_type, s.init, *s.spline);
        } else {
          ops.emplace_back(op->id(), op_type, s.init);
        }
        break;
      }

      case MatrixOpValueFb_ConstantOpFb: {
        const ConstantOpFb* const_fb =
            reinterpret_cast<const ConstantOpFb*>(op->value());
        ops.emplace_back(op->id(), op_type, const_fb->y_const());
        break;
      }

      default:
        assert(false);  // Invalid FlatBuffer data.
    }
  }
}

// Maximum duration of any of the splines.
static MotiveTime EndTime(const std::vector<MatrixOperationInit>& ops) {
  MotiveTime end_time = 0;
  for (size_t i = 0; i < ops.size(); ++i) {
    const MatrixOperationInit& op = ops[i];
    if (op.union_type == MatrixOperationInit::kUnionSpline) {
      end_time = std::max(end_time, static_cast<MotiveTime>(op.spline->EndX()));
    }
  }
  return end_time;
}

void RigAnimFromFlatBuffers(const RigAnimFb& params, RigAnim* anim) {
  const size_t num_bones = flatbuffers::VectorLength(params.matrix_anims());
  const auto names = params.bone_names();
  const auto parents = params.bone_parents();
  const bool record_names = names != nullptr && names->Length() == num_bones;
  assert(flatbuffers::VectorLength(parents) == num_bones);

  const char* anim_name =
      params.name() == nullptr ? "Unknown" : params.name()->c_str();
  anim->Init(anim_name, static_cast<motive::BoneIndex>(num_bones),
             record_names);

  MotiveTime end_time = 0;
  for (BoneIndex i = 0; i < num_bones; ++i) {
    const BoneIndex parent = parents->Get(i);
    const char* name = record_names ? names->Get(i)->c_str() : "";
    MatrixAnim& m = anim->InitMatrixAnim(i, parent, name);
    MatrixAnimFromFlatBuffers(*params.matrix_anims()->Get(i), &m);
    end_time = std::max(end_time, EndTime(m.ops()));
  }

  // Set animation-wide values.
  anim->set_end_time(params.repeat() ? std::numeric_limits<MotiveTime>::max()
                                     : end_time);
  anim->set_repeat(params.repeat() != 0);
}

}  // namespace motive
