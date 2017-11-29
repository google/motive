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

#include "anim_pipeline.h"

#include <unordered_map>

#include "anim_data.h"
#include "fbx_common/fbx_common.h"
#include "flat_anim.h"
#include "fplutil/file_utils.h"

namespace motive {

using fplutil::kLogError;
using fplutil::kLogImportant;
using fplutil::kLogInfo;
using fplutil::kLogVerbose;
using fplutil::kLogWarning;
using fplutil::Logger;
using fplutil::LogLevel;
using fplutil::LogOptions;

static const int kDefaultChannelOrder[] = {0, 1, 2};
static const int kRotationOrderToChannelOrder[][3] = {
    {2, 1, 0},  // eOrderXYZ,
    {2, 0, 1},  // eOrderXZY,
    {1, 0, 2},  // eOrderYZX,
    {1, 2, 0},  // eOrderYXZ,
    {0, 2, 1},  // eOrderZXY,
    {0, 1, 2},  // eOrderZYX,
    {2, 1, 0},  // eOrderSphericXYZ
};
static const int kRotationOrderToChannelOrderInverted[][3] = {
    {0, 1, 2},  // eOrderXYZ,
    {0, 2, 1},  // eOrderXZY,
    {1, 2, 0},  // eOrderYZX,
    {1, 0, 2},  // eOrderYXZ,
    {2, 0, 1},  // eOrderZXY,
    {2, 1, 0},  // eOrderZYX,
    {0, 1, 2},  // eOrderSphericXYZ
};

// Half a percent.
static const float kDefaultScaleTolerance = 0.005f;

// 0.5 degrees in radians.
static const float kDefaultRotateTolerance = 0.00873f;

// Totally arbitrary. TODO: make a percentage of the model size.
static const float kDefaultTranslateTolerance = 0.01f;

// 0.5 degrees in radians.
static const float kDefaultDerivativeAngleTolerance = 0.00873f;

// 10 degrees in radians.
static const float kDefaultRepeatDerivativeAngleTolerance = 0.1745f;

Tolerances::Tolerances()
    : scale(kDefaultScaleTolerance),
      rotate(kDefaultRotateTolerance),
      translate(kDefaultTranslateTolerance),
      derivative_angle(kDefaultDerivativeAngleTolerance),
      repeat_derivative_angle(kDefaultRepeatDerivativeAngleTolerance) {}

/// @class FbxParser
/// @brief Load FBX files and save their geometry and animations in our
///        FlatBuffer format.
class FbxAnimParser {
 public:
  explicit FbxAnimParser(Logger& log)
      : manager_(nullptr), scene_(nullptr), log_(log) {
    // The FbxManager is the gateway to the FBX API.
    manager_ = FbxManager::Create();
    if (manager_ == nullptr) {
      log_.Log(kLogError, "Unable to create FBX manager.\n");
      return;
    }

    // Initialize with standard IO settings.
    FbxIOSettings* ios = FbxIOSettings::Create(manager_, IOSROOT);
    manager_->SetIOSettings(ios);

    // Create an FBX scene. This object holds most objects imported/exported
    // from/to files.
    scene_ = FbxScene::Create(manager_, "My Scene");
    if (scene_ == nullptr) {
      log_.Log(kLogError, "Unable to create FBX scene.\n");
      return;
    }
  }

  ~FbxAnimParser() {
    // Delete the FBX Manager and all objects that it created.
    if (manager_ != nullptr) manager_->Destroy();
  }

  bool Valid() const { return manager_ != nullptr && scene_ != nullptr; }

  bool Load(const char* file_name, fplutil::AxisSystem axis_system,
            float distance_unit_scale) {
    if (!Valid()) return false;

    log_.Log(
        kLogInfo,
        "---- anim_pipeline: %s ------------------------------------------\n",
        fplutil::BaseFileName(file_name).c_str());

    // Create the importer and initialize with the file.
    FbxImporter* importer = FbxImporter::Create(manager_, "");
    const bool init_status =
        importer->Initialize(file_name, -1, manager_->GetIOSettings());

    // Check the SDK and pipeline versions.
    int sdk_major = 0, sdk_minor = 0, sdk_revision = 0;
    int file_major = 0, file_minor = 0, file_revision = 0;
    FbxManager::GetFileFormatVersion(sdk_major, sdk_minor, sdk_revision);
    importer->GetFileVersion(file_major, file_minor, file_revision);

    // Exit on load error.
    if (!init_status) {
      FbxString error = importer->GetStatus().GetErrorString();
      log_.Log(kLogError, "%s\n\n", error.Buffer());
      importer->Destroy();
      return false;
    }

    // Import the scene.
    const bool import_status = importer->Import(scene_);

    // Report version information.
    const LogLevel version_log_level = import_status ? kLogVerbose : kLogError;
    log_.Log(version_log_level, "File version %d.%d.%d, SDK version %d.%d.%d\n",
             file_major, file_minor, file_revision, sdk_major, sdk_minor,
             sdk_revision);

    // Exit on import error.
    if (!import_status) {
      FbxString error = importer->GetStatus().GetErrorString();
      log_.Log(kLogError, "%s\n\n", error.Buffer());
      importer->Destroy();
      return false;
    }

    // Clean-up temporaries.
    importer->Destroy();

    // Exit if the import failed.
    if (!import_status) return false;

    // Ensure the correct distance unit and axis system are being used.
    fplutil::ConvertFbxScale(distance_unit_scale, scene_, &log_);
    fplutil::ConvertFbxAxes(axis_system, scene_, &log_);

    // Log nodes after we've processed them.
    log_.Log(kLogVerbose, "Converted scene nodes\n");
    fplutil::LogFbxScene(scene_, 0, kLogVerbose, &log_);

    // Remember the source file name so we can search for textures nearby.
    anim_file_name_ = std::string(file_name);
    return true;
  }

  // Map FBX nodes to bone indices, used to create bone index references.
  typedef std::unordered_map<const FbxNode*, unsigned int> NodeToBoneMap;

  static int AddBoneForNode(NodeToBoneMap* node_to_bone_map,
                            const FbxNode* node, int parent_bone_index,
                            FlatAnim* out) {
    // The node is a bone if it was marked as one by MarkBoneNodesRecursive.
    const auto found_it = node_to_bone_map->find(node);
    if (found_it == node_to_bone_map->end()) {
      return -1;
    }

    // Add the bone entry.
    const char* const name = node->GetName();
    const unsigned int bone_index = out->AllocBone(name, parent_bone_index);
    found_it->second = bone_index;
    return bone_index;
  }

  bool MarkBoneNodesRecursive(NodeToBoneMap* node_to_bone_map,
                              FbxNode* node) const {
    // We need a bone for this node if it has a skeleton attribute or a mesh.
    bool need_bone = (node->GetSkeleton() || node->GetMesh());

    // We also need a bone for this node if it has any such child bones.
    const int child_count = node->GetChildCount();
    for (int child_index = 0; child_index != child_count; ++child_index) {
      FbxNode* const child_node = node->GetChild(child_index);
      if (MarkBoneNodesRecursive(node_to_bone_map, child_node)) {
        need_bone = true;
      }
    }

    // Flag the node as a bone.
    if (need_bone) {
      node_to_bone_map->insert(NodeToBoneMap::value_type(node, -1));
    }
    return need_bone;
  }

  void GatherBonesRecursive(NodeToBoneMap* node_to_bone_map,
                            const FbxNode* node, int parent_bone_index,
                            FlatAnim* out) const {
    const int bone_index =
        AddBoneForNode(node_to_bone_map, node, parent_bone_index, out);
    if (bone_index >= 0) {
      const int child_count = node->GetChildCount();
      for (int child_index = 0; child_index != child_count; ++child_index) {
        const FbxNode* const child_node = node->GetChild(child_index);
        GatherBonesRecursive(node_to_bone_map, child_node, bone_index, out);
      }
    }
  }

  void GatherFlatAnim(bool no_uniform_scale, FlatAnim* out) const {
    FbxNode* const root_node = scene_->GetRootNode();
    const int child_count = root_node->GetChildCount();
    NodeToBoneMap node_to_bone_map;

    // First pass: determine which nodes are to be treated as bones.
    // We skip the root node so it's not included in the bone hierarchy.
    for (int child_index = 0; child_index != child_count; ++child_index) {
      FbxNode* const child_node = root_node->GetChild(child_index);
      MarkBoneNodesRecursive(&node_to_bone_map, child_node);
    }

    // Second pass: add bones.
    // We skip the root node so it's not included in the bone hierarchy.
    for (int child_index = 0; child_index != child_count; ++child_index) {
      FbxNode* const child_node = root_node->GetChild(child_index);
      GatherBonesRecursive(&node_to_bone_map, child_node, -1, out);
    }

    // Final pass: extract animation data for bones.
    GatherFlatAnimRecursive(&node_to_bone_map, root_node, no_uniform_scale,
                            out);
  }

  void LogAnimStateAtTime(int time_in_ms) const {
    fplutil::LogFbxScene(scene_, time_in_ms, kLogInfo, &log_);
  }

 private:
  MOTIVE_DISALLOW_COPY_AND_ASSIGN(FbxAnimParser);

  struct AnimOp {
    MatrixOperationType op;
    bool invert;
  };

  struct AnimProperty {
    FbxPropertyT<FbxDouble3>* property;
    MatrixOpId id;
    AnimOp op;
  };

  static FlatTime FbxToFlatTime(const FbxTime& t) {
    const FbxLongLong milliseconds = t.GetMilliSeconds();
    assert(milliseconds <= std::numeric_limits<FlatTime>::max());
    return static_cast<FlatTime>(milliseconds);
  }

  static FlatVal InvertValue(FlatVal val, const AnimOp& op) {
    return !op.invert ? val : motive::ScaleOp(op.op) ? 1.0f / val : -val;
  }

  static FlatVal FbxToFlatValue(const double x, const AnimOp& op) {
    const FlatVal val = motive::RotateOp(op.op)
                            ? static_cast<FlatVal>(FBXSDK_DEG_TO_RAD * x)
                            : static_cast<FlatVal>(x);
    return InvertValue(val, op);
  }

  static FlatDerivative FbxToFlatDerivative(const float d, const AnimOp& op) {
    // The FBX derivative appears to be in units of seconds.
    // The FlatBuffer file format is in units of milliseconds.
    const float d_time_scaled = d / 1000.0f;
    return FbxToFlatValue(d_time_scaled, op);
  }

  void GatherFlatAnimRecursive(const NodeToBoneMap* node_to_bone_map,
                               FbxNode* node, bool no_uniform_scale,
                               FlatAnim* out) const {
    if (node == nullptr) return;
    log_.Log(kLogVerbose, "Node: %s\n", node->GetName());

    // The root node cannot have a transform applied to it, so we do not
    // export it as a bone.
    int bone_index = -1;
    if (node != scene_->GetRootNode()) {
      // We're only interested in nodes that contain meshes or are part of a
      // skeleton. If a node and all nodes under it have neither, we early out.
      const auto found_it = node_to_bone_map->find(node);
      if (found_it == node_to_bone_map->end()) return;
      bone_index = found_it->second;

      // Gather the animation data that drives the bone.
      out->SetCurBoneIndex(bone_index);
      GatherFlatAnimForNode(node, no_uniform_scale, out);
      out->ResetCurBoneIndex();
    }

    // Recursively traverse each node in the scene
    if (bone_index < 0 || out->ShouldRecurse(bone_index)) {
      for (int i = 0; i < node->GetChildCount(); i++) {
        GatherFlatAnimRecursive(node_to_bone_map, node->GetChild(i),
                                no_uniform_scale, out);
      }
    }
  }

  static FbxAnimCurveNode* AnimCurveNodeDrivingProperty(FbxProperty& property) {
    const int count = property.GetSrcObjectCount();
    for (int i = 0; i < count; ++i) {
      FbxObject* obj = property.GetSrcObject(i);
      if (obj->GetClassId() == FbxAnimCurveNode::ClassId)
        return static_cast<FbxAnimCurveNode*>(obj);
    }
    return nullptr;
  }

  bool AnimConst(const AnimProperty& p, int channel, float tolerance,
                 float derivative_tolerance, FbxAnimCurveNode* anim_node,
                 float* const_value) const {
    // If anim_node can provide no data, return the value from the property.
    if (anim_node == nullptr ||
        channel >= static_cast<int>(anim_node->GetChannelsCount())) {
      *const_value = FbxToFlatValue(p.property->Get()[channel], p.op);
      return true;
    }

    // Grab the start value from the anim_node. If const, this will be the
    // constant value.
    *const_value =
        FbxToFlatValue(anim_node->GetChannelValue(channel, 0.0f), p.op);

    // If there is no animation curve, or the curve has no keys, then must be
    // const.
    FbxAnimCurve* curve = anim_node->GetCurve(channel);
    if (curve == nullptr || curve->KeyGetCount() <= 0) return true;

    // The first value may be different from the value at time 0.
    // The value at time 0 may actually be the end value, if the first key
    // doesn't start at time 0 and the channel cycles.
    const float first_value = FbxToFlatValue(curve->KeyGetValue(0), p.op);

    // If any keys has a different value, or non-0 slope, then not const.
    const int num_keys = curve->KeyGetCount();
    for (int i = 0; i < num_keys - 1; ++i) {
      const float left_derivative =
          FbxToFlatDerivative(curve->KeyGetLeftDerivative(i), p.op);
      const float right_derivative =
          FbxToFlatDerivative(curve->KeyGetRightDerivative(i), p.op);
      const float value = FbxToFlatValue(curve->KeyGetValue(i + 1), p.op);
      if (fabs(DerivativeAngle(left_derivative)) > derivative_tolerance ||
          fabs(DerivativeAngle(right_derivative)) > derivative_tolerance ||
          fabs(value - first_value) > tolerance)
        return false;
    }
    return true;
  }

  static const int* ChannelOrder(const FbxNode* node, const AnimOp& op) {
    // X,y,z order is significant only for rotations.
    if (!motive::RotateOp(op.op)) return kDefaultChannelOrder;

    // We output the last channel first, since they're applied in reverse
    // order.
    FbxEuler::EOrder rotation_order;
    node->GetRotationOrder(FbxNode::eSourcePivot, rotation_order);
    assert(0 <= rotation_order &&
           rotation_order < MOTIVE_ARRAY_SIZE(kRotationOrderToChannelOrder));
    return op.invert ? kRotationOrderToChannelOrderInverted[rotation_order]
                     : kRotationOrderToChannelOrder[rotation_order];
  }

  void GatherFlatAnimForNode(FbxNode* node, bool no_uniform_scale,
                             FlatAnim* out) const {
    // The FBX tranform format is defined as below (see
    // http://help.autodesk.com/view/FBX/2016/ENU/?guid=__files_GUID_10CDD63C_79C1_4F2D_BB28_AD2BE65A02ED_htm):
    //
    // WorldTransform = ParentWorldTransform * T * Roff * Rp * Rpre * R *
    //                  Rpost_inv * Rp_inv * Soff * Sp * S * Sp_inv
    //
    const AnimProperty properties[] = {
        {&node->LclTranslation, 0, {motive::kTranslateX, false}},
        {&node->RotationOffset, 0, {motive::kTranslateX, false}},
        {&node->RotationPivot, 0, {motive::kTranslateX, false}},
        {&node->PreRotation, 3, {motive::kRotateAboutX, false}},
        {&node->LclRotation, 6, {motive::kRotateAboutX, false}},
        {&node->PostRotation, 9, {motive::kRotateAboutX, true}},
        {&node->RotationPivot, 12, {motive::kTranslateX, true}},
        {&node->ScalingOffset, 12, {motive::kTranslateX, false}},
        {&node->ScalingPivot, 12, {motive::kTranslateX, false}},
        {&node->LclScaling, 15, {motive::kScaleX, false}},
        {&node->ScalingPivot, 19, {motive::kTranslateX, true}}};

    for (size_t i = 0; i < MOTIVE_ARRAY_SIZE(properties); ++i) {
      const AnimProperty& p = properties[i];

      // Get the curve attached to the property that's animated.
      FbxAnimCurveNode* anim_node = AnimCurveNodeDrivingProperty(*p.property);

      // Ensure we have three channels (x, y, z).
      if (anim_node != nullptr && anim_node->GetChannelsCount() != 3) {
        log_.Log(kLogError,
                 "Animation property %s has %d channels instead of 3\n",
                 p.property->GetNameAsCStr(), anim_node->GetChannelsCount());
        continue;
      }

      // Rotations must be applied in the correct order.
      const int* channel_order = ChannelOrder(node, p.op);
      for (int channel_idx = 0; channel_idx < 3; ++channel_idx) {
        // Proceed through each channel: x, y, z.
        const int channel = channel_order[channel_idx];
        const motive::MatrixOperationType op =
            static_cast<motive::MatrixOperationType>(p.op.op + channel);

        // If the channel is const, only output if it's not the default value.
        float const_value = 0.0f;
        const bool anim_const = AnimConst(p, channel, out->ToleranceForOp(op),
                                          out->ToleranceForDerivativeAngle(),
                                          anim_node, &const_value);
        if (anim_const && out->IsDefaultValue(op, const_value)) continue;

        // Allocate a channel_id for the output data.
        const FlatChannelId channel_id =
            out->AllocChannel(op, static_cast<MatrixOpId>(p.id + channel_idx));

        // Record constant value for this channel.
        if (anim_const) {
          out->AddConstant(channel_id, const_value);
          log_.Log(kLogVerbose, "  [channel %d] %s, %s: constant %f\n",
                   channel_id, p.property->GetNameAsCStr(), MatrixOpName(op),
                   const_value);
          assert(out->NumNodes(channel_id) > 0);
          continue;
        }
        assert(anim_node != nullptr);

        // We process only the first curve, for simplicity.
        // If we run into animations with multiple curves, we should add
        // extra logic here.
        const int num_curves = anim_node->GetCurveCount(channel);
        if (num_curves > 1) {
          log_.Log(kLogWarning,
                   "%s, %s has %d curves. Only using the first one.\n",
                   p.property->GetNameAsCStr(), MatrixOpName(op), num_curves);
        }

        // For every key in the curve, log data to `out`.
        log_.Log(kLogVerbose, "  [channel %d] %s, %s: curve\n", channel_id,
                 p.property->GetNameAsCStr(), MatrixOpName(op));
        FbxAnimCurve* curve = anim_node->GetCurve(channel);
        GatherFlatAnimCurve(channel_id, curve, p.op, out);
        assert(out->NumNodes(channel_id) > 0);
      }
    }

    // Collapse unnecesary channels, when possible.
    out->PruneChannels(no_uniform_scale);
  }

  void GatherFlatAnimCurve(const FlatChannelId channel_id, FbxAnimCurve* curve,
                           const AnimOp& op, FlatAnim* out) const {
    log_.Log(kLogVerbose, "    source, key, x, y, slope\n");
    const int num_keys = curve->KeyGetCount();
    assert(num_keys > 1);  // Since we checked for constant values earlier.

    // If there are multiple keys, then add at least one cubic for each
    // key interval.
    for (int k = 0; k < num_keys - 1; ++k) {
      const FbxTime start_time = curve->KeyGetTime(k);
      const FbxTime end_time = curve->KeyGetTime(k + 1);

      // Gather indermediate values. We use these to check validity of cubic.
      static const int kNumIntermediateValues = 16;
      const FbxTime t_inc =
          (end_time - start_time) / (kNumIntermediateValues - 1);
      FbxTime t = start_time;
      int last_index = 0;
      float values[kNumIntermediateValues];
      float derivatives[kNumIntermediateValues];
      for (int i = 0; i < kNumIntermediateValues; ++i) {
        values[i] = FbxToFlatValue(curve->Evaluate(t, &last_index), op);
        derivatives[i] = FbxToFlatDerivative(
            curve->EvaluateLeftDerivative(t, &last_index), op);
        t += t_inc;
      }
      derivatives[0] = FbxToFlatDerivative(
          curve->EvaluateRightDerivative(start_time, &last_index), op);

      // Send to FlatAnim for conversion into cubic curves.
      const FlatTime start_time_flat = FbxToFlatTime(start_time);
      const FlatTime end_time_flat = FbxToFlatTime(end_time);
      out->AddCurve(channel_id, start_time_flat, end_time_flat, values,
                    derivatives, kNumIntermediateValues);

      // Log the input key points.
      log_.Log(kLogVerbose, "    fbx, %d, %d, %f, %f\n", k, start_time_flat,
               values[0], derivatives[0]);
      if (k == num_keys - 2) {
        log_.Log(kLogVerbose, "    fbx, %d, %d, %f, %f\n", k + 1, end_time_flat,
                 values[kNumIntermediateValues - 1],
                 derivatives[kNumIntermediateValues - 1]);
      }
    }

    // Remove duplicates.
    out->PruneNodes(channel_id);

    // Log the output key points.
    out->LogChannel(channel_id);
  }

  // Entry point to the FBX SDK.
  FbxManager* manager_;

  // Hold the FBX file data.
  FbxScene* scene_;

  // Name of source mesh file. Used to search for textures, when the textures
  // are not found in their referenced location.
  std::string anim_file_name_;

  // Information and warnings.
  Logger& log_;
};

AnimPipelineArgs::AnimPipelineArgs()
    : fbx_file(""),
      output_file(""),
      log_level(kLogWarning),
      repeat_preference(kRepeatIfRepeatable),
      stagger_end_times(false),
      preserve_start_time(false),
      root_bones_only(false),
      no_uniform_scale(false),
      axis_system(fplutil::kUnspecifiedAxisSystem),
      distance_unit_scale(-1.0f),
      debug_time(-1) {}

int RunAnimPipeline(const AnimPipelineArgs& args, fplutil::Logger& log) {
  // Update the amount of information we're dumping.
  log.set_level(args.log_level);

  // Load the FBX file.
  motive::FbxAnimParser pipe(log);
  const bool load_status = pipe.Load(args.fbx_file.c_str(), args.axis_system,
                                     args.distance_unit_scale);
  if (!load_status) return 1;

  // Output debug information for the specific time of the animation.
  if (args.debug_time >= 0) {
    pipe.LogAnimStateAtTime(args.debug_time);
    return 0;
  }

  // Gather data into a format conducive to our FlatBuffer format.
  motive::FlatAnim anim(args.tolerances, args.root_bones_only, log);
  pipe.GatherFlatAnim(args.no_uniform_scale, &anim);

  // We want the animation to start from tick 0.
  if (!args.preserve_start_time) {
    anim.ShiftTime(-anim.MinAnimatedTime());
  }

  // We want all of our animation channels to end at the same time.
  if (!args.stagger_end_times) {
    anim.ExtendChannelsToTime(anim.MaxAnimatedTime());
  }

  // Output gathered data to a binary FlatBuffer.
  anim.LogAllChannels();
  const bool output_status =
      anim.OutputFlatBuffer(args.output_file, args.repeat_preference);

  if (!output_status) return 1;

  // Success.
  return 0;
}

}  // namespace motive
