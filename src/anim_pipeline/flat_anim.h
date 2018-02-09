#ifndef MOTIVE_FLAT_ANIM_H_
#define MOTIVE_FLAT_ANIM_H_

#include "anim_data.h"
#include "flatbuffers/flatbuffers.h"

namespace motive {

/// @class FlatAnim
/// @brief Extends AnimData with functionality for writing to a FlatBuffer.
class FlatAnim : public AnimData {
 public:
  FlatAnim(const Tolerances& tolerances, bool root_bones_only,
           fplutil::Logger& log)
      : AnimData(tolerances, root_bones_only, log) {}

  bool OutputFlatBuffer(const std::string& suggested_output_file,
                        RepeatPreference repeat_preference) const;

  void LogChannel(FlatChannelId channel_id) const;
  void LogAllChannels() const;

 private:
  // Build the FlatBuffer to be output into `fbb` and return the number of
  // `RigAnimFb` tables output to `fbb`. If the number is >1, then aggregate
  // them all into one `AnimListFb`.
  int CreateFlatBuffer(flatbuffers::FlatBufferBuilder& fbb,
                       RepeatPreference repeat_preference,
                       const std::string& anim_name) const;
  flatbuffers::Offset<RigAnimFb> CreateRigAnimFbFromBoneRange(
      flatbuffers::FlatBufferBuilder& fbb, RepeatPreference repeat_preference,
      const BoneRange& bone_range, const std::string& anim_name) const;
  // Remove the namespacing from the bone name.
  static const char* BoneBaseName(const std::string& name);
  static Range SplineYRange(const Channel& ch);
  static CompactSpline* CreateCompactSpline(const Channel& ch);
  static flatbuffers::Offset<motive::CompactSplineFb> CreateSplineFlatBuffer(
      flatbuffers::FlatBufferBuilder& fbb, const CompactSpline& s);
};

}  // namespace motive

#endif  // MOTIVE_FLAT_ANIM_H_
