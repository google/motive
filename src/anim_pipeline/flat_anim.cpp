#include "flat_anim.h"

#include <sstream>
#include "anim_generated.h"
#include "anim_list_generated.h"
#include "fplutil/file_utils.h"

namespace motive {

void FlatAnim::LogChannel(FlatChannelId channel_id) const {
  const Channels& channels = CurChannels();
  const Nodes& n = channels[channel_id].nodes;
  for (size_t i = 0; i < n.size(); ++i) {
    const SplineNode& node = n[i];
    log_.Log(fplutil::kLogVerbose, "    flat, %d, %d, %f, %f\n", i, node.time,
             node.val, node.derivative);
  }
}

void FlatAnim::LogAllChannels() const {
  log_.Log(fplutil::kLogInfo, "  %30s %16s  %9s   %s\n", "bone name",
           "operation", "time range", "values");
  for (size_t bone_idx = 0; bone_idx < bones_.size(); ++bone_idx) {
    const Bone& bone = bones_[bone_idx];
    const Channels& channels = bone.channels;
    if (channels.size() == 0) continue;

    for (auto c = channels.begin(); c != channels.end(); ++c) {
      log_.Log(fplutil::kLogInfo, "  %30s %16s   ", BoneBaseName(bone.name),
               MatrixOpName(c->op));
      const char* format = motive::RotateOp(c->op)
                               ? "%.0f "
                               : motive::TranslateOp(c->op) ? "%.1f " : "%.2f ";
      const float factor =
          motive::RotateOp(c->op) ? motive::kRadiansToDegrees : 1.0f;

      const Nodes& n = c->nodes;
      if (n.size() <= 1) {
        log_.Log(fplutil::kLogInfo, " constant   ");
      } else {
        log_.Log(fplutil::kLogInfo, "%4d~%4d   ", n[0].time,
                 n[n.size() - 1].time);
      }

      for (size_t i = 0; i < n.size(); ++i) {
        log_.Log(fplutil::kLogInfo, format, factor * n[i].val);
      }
      log_.Log(fplutil::kLogInfo, "\n");
    }
  }
}

bool FlatAnim::OutputFlatBuffer(const std::string& suggested_output_file,
                                RepeatPreference repeat_preference) const {
  const std::string anim_name = fplutil::RemoveDirectoryFromName(
      fplutil::RemoveExtensionFromName(suggested_output_file));

  // Build the flatbuffer into `fbb`.
  flatbuffers::FlatBufferBuilder fbb;
  const int num_rig_anims = CreateFlatBuffer(fbb, repeat_preference, anim_name);
  if (num_rig_anims == 0) return false;

  // Set the extension appropriately.
  const std::string output_file =
      fplutil::RemoveExtensionFromName(suggested_output_file) + "." +
      (num_rig_anims == 1 ? motive::RigAnimFbExtension()
                          : motive::AnimListFbExtension());

  // Ensure output directory exists.
  const std::string output_dir = fplutil::DirectoryName(output_file);
  if (!fplutil::CreateDirectory(output_dir.c_str())) {
    log_.Log(fplutil::kLogError, "Could not create output directory %s\n",
             output_dir.c_str());
    return false;
  }

  // Create the output file.
  FILE* file = fopen(output_file.c_str(), "wb");
  if (file == nullptr) {
    log_.Log(fplutil::kLogError, "Could not open %s for writing\n",
             output_file.c_str());
    return false;
  }

  // Write the binary data to the file and close it.
  log_.Log(fplutil::kLogVerbose, "Writing %s", output_file.c_str());
  fwrite(fbb.GetBufferPointer(), 1, fbb.GetSize(), file);
  fclose(file);

  // Log summary.
  log_.Log(fplutil::kLogImportant, "  %s (%d bytes)\n",
           fplutil::RemoveDirectoryFromName(output_file).c_str(), NumBytes());
  return true;
}

int FlatAnim::CreateFlatBuffer(flatbuffers::FlatBufferBuilder& fbb,
                               RepeatPreference repeat_preference,
                               const std::string& anim_name) const {
  if (bones_.size() > kMaxNumBones) {
    log_.Log(fplutil::kLogError, "Too many bones: %d. Limit of %d.\n",
             bones_.size(), kMaxNumBones);
    return 0;
  }

  const BoneIndex num_bones = static_cast<BoneIndex>(bones_.size());

  // Output entire bone range into one RigAnim.
  if (!root_bones_only_) {
    const flatbuffers::Offset<RigAnimFb> rig_anim_offset =
        CreateRigAnimFbFromBoneRange(fbb, repeat_preference,
                                     BoneRange(0, num_bones), anim_name);
    motive::FinishRigAnimFbBuffer(fbb, rig_anim_offset);
    return 1;
  }

  // Output each bone into a separate RigAnim.
  std::vector<flatbuffers::Offset<RigAnimFb>> rig_anim_offsets;
  rig_anim_offsets.reserve(num_bones);
  for (BoneIndex bone_idx = 0; bone_idx < num_bones; ++bone_idx) {
    // Skip bones that have no animation data.
    const Bone& bone = bones_[bone_idx];
    if (bone.channels.size() == 0) continue;

    // Use the bone index to ensure that the anim name is unique in the
    // AnimTable. Note that the bone name may be the same for multiple bones.
    std::stringstream bone_anim_name;
    bone_anim_name << anim_name << "_" << static_cast<int>(bone_idx);

    // Create a RigAnim with only `bone_idx`.
    rig_anim_offsets.push_back(CreateRigAnimFbFromBoneRange(
        fbb, repeat_preference, BoneRange(bone_idx, bone_idx + 1),
        bone_anim_name.str()));
  }

  // No bones had any animation data, so do nothing.
  if (rig_anim_offsets.size() == 0) {
    log_.Log(fplutil::kLogWarning, "No animation found.\n");
    return 0;
  }

  // If only one bone with animation data exists, just output a RigAnim.
  if (rig_anim_offsets.size() == 1) {
    motive::FinishRigAnimFbBuffer(fbb, rig_anim_offsets[0]);
    return 1;
  }

  // Multiple animations, so output an AnimList of RigAnims.
  std::vector<flatbuffers::Offset<AnimSource>> anims;
  anims.reserve(rig_anim_offsets.size());
  for (auto it = rig_anim_offsets.begin(); it != rig_anim_offsets.end(); ++it) {
    anims.push_back(motive::CreateAnimSource(
        fbb, motive::AnimSourceUnion_AnimSourceEmbedded,
        motive::CreateAnimSourceEmbedded(fbb, *it).Union()));
  }
  auto list_offset = motive::CreateAnimListFb(fbb, 0, fbb.CreateVector(anims));
  motive::FinishAnimListFbBuffer(fbb, list_offset);
  return static_cast<int>(rig_anim_offsets.size());
}

flatbuffers::Offset<RigAnimFb> FlatAnim::CreateRigAnimFbFromBoneRange(
    flatbuffers::FlatBufferBuilder& fbb, RepeatPreference repeat_preference,
    const BoneRange& bone_range, const std::string& anim_name) const {
  std::vector<flatbuffers::Offset<motive::MatrixAnimFb>> matrix_anims;
  std::vector<flatbuffers::Offset<flatbuffers::String>> bone_names;
  std::vector<BoneIndex> bone_parents;
  const size_t num_bones = bone_range.Length();
  matrix_anims.reserve(num_bones);
  bone_names.reserve(num_bones);
  bone_parents.reserve(num_bones);
  for (BoneIndex bone_idx = bone_range.start(); bone_idx < bone_range.end();
       ++bone_idx) {
    const Bone& bone = bones_[bone_idx];
    const Channels& channels = bone.channels;

    // Output each channel as a MatrixOp, and gather in the `ops` vector.
    std::vector<flatbuffers::Offset<motive::MatrixOpFb>> ops;
    for (auto c = channels.begin(); c != channels.end(); ++c) {
      const Nodes& n = c->nodes;
      assert(n.size() > 0);

      flatbuffers::Offset<void> value;
      motive::MatrixOpValueFb value_type;
      if (n.size() <= 1) {
        // Output constant value MatrixOp.
        value = motive::CreateConstantOpFb(fbb, n[0].val).Union();
        value_type = motive::MatrixOpValueFb_ConstantOpFb;

      } else {
        // We clamp negative times to 0, but it's going to look strange for
        // non-constant channels.
        if (n[0].time < 0) {
          log_.Log(fplutil::kLogWarning, "%s (%s) starts at negative time %d\n",
                   BoneBaseName(bone.name), MatrixOpName(c->op), n[0].time);
        }

        // Output spline MatrixOp.
        CompactSpline* s = CreateCompactSpline(*c);
        value = CreateSplineFlatBuffer(fbb, *s).Union();
        value_type = motive::MatrixOpValueFb_CompactSplineFb;
        CompactSpline::Destroy(s);
      }

      ops.push_back(motive::CreateMatrixOpFb(
          fbb, c->id, static_cast<motive::MatrixOperationTypeFb>(c->op),
          value_type, value));
    }

    // Convert vector into a FlatBuffers vector, and create the
    // MatrixAnimation.
    auto ops_fb = fbb.CreateVector(ops);
    auto matrix_anim_fb = CreateMatrixAnimFb(fbb, ops_fb);
    matrix_anims.push_back(matrix_anim_fb);
    bone_names.push_back(fbb.CreateString(BoneBaseName(bone.name)));
    bone_parents.push_back(BoneParent(bone_idx));
  }

  // Finish off the FlatBuffer by creating the root RigAnimFb table.
  auto bone_names_fb = fbb.CreateVector(bone_names);
  auto bone_parents_fb = fbb.CreateVector(bone_parents);
  auto matrix_anims_fb = fbb.CreateVector(matrix_anims);
  const bool repeat = Repeat(repeat_preference);
  auto anim_name_fb = fbb.CreateString(anim_name);
  auto rig_anim_fb = CreateRigAnimFb(fbb, matrix_anims_fb, bone_parents_fb,
                                     bone_names_fb, repeat, anim_name_fb);
  return rig_anim_fb;
}

flatbuffers::Offset<motive::CompactSplineFb> FlatAnim::CreateSplineFlatBuffer(
    flatbuffers::FlatBufferBuilder& fbb, const CompactSpline& s) {
  auto nodes_fb = fbb.CreateVectorOfStructs(
      reinterpret_cast<const motive::CompactSplineNodeFb*>(s.nodes()),
      s.num_nodes());

  auto spline_fb = motive::CreateCompactSplineFb(
      fbb, s.y_range().start(), s.y_range().end(), s.x_granularity(), nodes_fb);

  return spline_fb;
}

const char* FlatAnim::BoneBaseName(const std::string& name) {
  const size_t colon = name.find_last_of(':');
  const size_t base_idx = colon == std::string::npos ? 0 : colon + 1;
  return &name[base_idx];
}

Range FlatAnim::SplineYRange(const Channel& ch) {
  // Find extreme values for nodes.
  Range y_range(Range::Empty());
  for (auto n = ch.nodes.begin(); n != ch.nodes.end(); ++n) {
    y_range = y_range.Include(n->val);
  }
  return y_range;
}

CompactSpline* FlatAnim::CreateCompactSpline(const Channel& ch) {
  const Nodes& nodes = ch.nodes;
  assert(nodes.size() > 1);

  // Maximize the bits we get for x by making the last time the maximum
  // x-value.
  const float x_granularity = CompactSpline::RecommendXGranularity(
      static_cast<float>(nodes.back().time));
  const Range y_range = SplineYRange(ch);

  // Construct the Spline from the node data directly.
  CompactSpline* s =
      CompactSpline::Create(static_cast<CompactSplineIndex>(nodes.size()));
  s->Init(y_range, x_granularity);
  float last_time = -std::numeric_limits<float>::max();
  for (auto n = nodes.begin(); n != nodes.end(); ++n) {
    const float n_time = static_cast<float>(std::max(0, n->time));
    // Exclude any decreasing time values, as these may produce inconsistent
    // spans at evaluation time and lead to errors.
    if (n_time >= last_time) {
      s->AddNode(n_time, n->val, n->derivative, kAddWithoutModification);
      last_time = n_time;
    }
  }
  return s;
}

}  // namespace motive
