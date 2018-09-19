#ifndef MOTIVE_ANIM_DATA_H_
#define MOTIVE_ANIM_DATA_H_

#include "anim_generated.h"
#include "anim_list_generated.h"
#include "anim_pipeline.h"
#include "motive/common.h"
#include "motive/math/angle.h"
#include "motive/math/range.h"
#include "motive/matrix_op.h"

namespace motive {

// Unique id identifying a single float curve being animated.
typedef int FlatChannelId;

// Time used for animation curves. Use an integer type for time so that we
// don't lose precision at the end of long animations.
typedef int FlatTime;

// Value output from animation curves.
typedef float FlatVal;

// Slope of animation curves.
typedef float FlatDerivative;

// Start and end range of bone indices.
typedef RangeT<BoneIndex> BoneRange;

/// @brief Convert derivative to its angle in x/y space.
///  derivative 0 ==> angle 0
///  derivative 1 ==> angle 45 degrees
///  derivative +inf ==> angle 90 degrees
///  derivative -2 ==> angle -63.4 degrees
/// @returns Angle, in radians, >= -pi and <= pi
inline float DerivativeAngle(float derivative) { return atan(derivative); }

/// @class AnimData
/// @brief Hold animation data to be written to FlatBuffer animation format.
class AnimData {
 public:
  AnimData(const Tolerances& tolerances, bool root_bones_only,
           fplutil::Logger& log);

  unsigned int AllocBone(const char* bone_name, int parent_bone_index);

  // Set/Reset the current bone index, used to access the current channels via
  // CurChannels.
  void SetCurBoneIndex(unsigned int cur_bone_index);
  void ResetCurBoneIndex();

  struct CurveSegment {
    FlatTime time_start;
    FlatTime time_end;
    const FlatVal* vals;
    const FlatDerivative* derivatives;
    size_t count;

    CurveSegment(FlatTime time_start, FlatTime time_end, const FlatVal* vals,
                 const FlatDerivative* derivatives, size_t count)
        : time_start(time_start),
          time_end(time_end),
          vals(vals),
          derivatives(derivatives),
          count(count) {}

    inline bool operator<(const CurveSegment& rhs) const {
      return time_start < rhs.time_start;
    }

    inline bool operator>(const CurveSegment& rhs) const {
      return rhs < *this;
    }
  };

  FlatChannelId AllocChannel(MatrixOperationType op, MatrixOpId id);
  void AddConstant(FlatChannelId channel_id, FlatVal const_val);
  void AddCurve(FlatChannelId channel_id, FlatTime time_start,
                FlatTime time_end, const FlatVal* vals,
                const FlatDerivative* derivatives, size_t count);
  size_t NumNodes(FlatChannelId channel_id) const;

  // Return true if we should keep decending down the mesh tree looking for
  // more animation.
  bool ShouldRecurse(unsigned int cur_bone_index) const;

  /// @brief Remove redundant nodes from `channel_id`.
  void PruneNodes(FlatChannelId channel_id);

  /// @brief Collapse multiple channels into one, when possible.
  void PruneChannels(bool no_uniform_scale);

  /// @brief Shift all times in all channels by `time_offset`.
  void ShiftTime(FlatTime time_offset);

  /// @brief For each channel that ends before `end_time`, extend it at its
  ///        current value to `end_time`. If already longer, or has no nodes
  ///        to begin with, do nothing.
  void ExtendChannelsToTime(FlatTime end_time);

  float ToleranceForOp(MatrixOperationType op) const;

  float ToleranceForDerivativeAngle() const;

  bool IsDefaultValue(MatrixOperationType op, float value) const;

  /// @brief Return the time of the channel that requires the most time.
  FlatTime MaxAnimatedTime() const;

  /// @brief Return the time of the channel that starts the earliest.
  ///
  /// Could be a negative time.
  FlatTime MinAnimatedTime() const;

 protected:
  MOTIVE_DISALLOW_COPY_AND_ASSIGN(AnimData);

  struct SplineNode {
    FlatTime time;
    FlatVal val;
    FlatDerivative derivative;

    SplineNode() : time(0), val(0.0f), derivative(0.0f) {}
    SplineNode(FlatTime time, FlatVal val, FlatDerivative derivative)
        : time(time), val(val), derivative(derivative) {}
    bool operator==(const SplineNode& rhs) const {
      return time == rhs.time && val == rhs.val && derivative == rhs.derivative;
    }
    bool operator!=(const SplineNode& rhs) const { return !operator==(rhs); }
  };
  typedef std::vector<SplineNode> Nodes;

  struct Channel {
    MatrixOperationType op;
    MatrixOpId id;
    Nodes nodes;

    Channel() : op(kInvalidMatrixOperation), id(kInvalidMatrixOpId) {}
    Channel(MatrixOperationType op, MatrixOpId id) : op(op), id(id) {}
    bool operator<(const Channel& rhs) const { return id < rhs.id; }
    bool operator>=(const Channel& rhs) const { return !operator<(rhs); }
  };
  typedef std::vector<Channel> Channels;

  struct Bone {
    // Unique name for this bone. Taken from mesh hierarchy.
    std::string name;

    // Parent bone index.  -1 for no parent.
    int parent_bone_index;

    // Hold animation data. One curve per channel.
    Channels channels;

    Bone(const char* name, int parent_bone_index)
        : name(name), parent_bone_index(parent_bone_index) {
      // There probably won't be more than one of each op type.
      channels.reserve(kNumMatrixOperationTypes);
    }
  };

  Channels& CurChannels();
  const Channels& CurChannels() const;

  int NumBytes() const;

  float Tolerance(FlatChannelId channel_id) const;

  /// Return the first channel of the first bone that isn't repeatable.
  /// If all channels are repeatable, return kInvalidBoneIdx.
  /// A channel is repeatable if its start and end values and derivatives
  /// are within `tolerances_`.
  BoneIndex FirstNonRepeatingBone(FlatChannelId* first_channel_id) const;

  // Determine if the animation should repeat back to start after it reaches
  // the end.
  bool Repeat(RepeatPreference repeat_preference) const;

  /// @brief Return true if the three channels starting at `channel_id`
  ///        can be replaced with a single kScaleUniformly channel.
  bool UniformScaleChannels(const Channels& channels,
                            FlatChannelId channel_id) const;

  FlatChannelId SummableChannel(const Channels& channels,
                                FlatChannelId ch) const;

  static FlatVal EvaluateNodes(const Nodes& nodes, FlatTime time,
                               FlatDerivative* derivative);

  // Gets the value from the channel either at the exact time specified, or
  // interpolated from the surrounding nodes. This is intended to be called
  // with consecutively increasing time values.  Returns true if a node was
  // sampled directly, so caller can move to next node.
  bool GetValueAtTime(const Nodes& nodes, const Nodes::const_iterator& node,
                      FlatTime time, FlatVal* value,
                      FlatDerivative* derivative) const;

  // Sum curves in ch_a and ch_b and put the result in ch_a.
  void SumChannels(Channels& channels, FlatChannelId ch_a,
                   FlatChannelId ch_b) const;

  BoneIndex BoneParent(int bone_idx) const;

  /// @brief Returns true if all nodes between the first and last in `n`
  ///        can be deleted without noticable difference to the curve.
  bool IntermediateNodesRedundant(const SplineNode* n, size_t len,
                                  float tolerance) const;

  static bool EqualNodes(const SplineNode& a, const SplineNode& b,
                         float tolerance, float derivative_tolerance);
  static FlatVal DefaultOpValue(MatrixOperationType op);

  // Hold animation data for each bone that's animated.
  std::vector<Bone> bones_;
  int cur_bone_index_;

  // Amount output curves are allowed to deviate from input.
  Tolerances tolerances_;

  // Only record animations for first bones in the skeleton to have animation.
  // Each such bone gets its own animation file.
  bool root_bones_only_;

  // Information and warnings.
  fplutil::Logger& log_;
};

}  // namespace motive

#endif  // MOTIVE_ANIM_DATA_H_
