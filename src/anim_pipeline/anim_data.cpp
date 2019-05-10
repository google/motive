#include "anim_data.h"

#include <queue>

#include "motive/matrix_anim.h"
#include "motive/rig_anim.h"

namespace motive {

// Use these bitfields to find situations where scale x, y, and z occur, in
// any order, in a row.
static const uint32_t kScaleXBitfield = 1 << motive::kScaleX;
static const uint32_t kScaleYBitfield = 1 << motive::kScaleY;
static const uint32_t kScaleZBitfield = 1 << motive::kScaleZ;
static const uint32_t kScaleXyzBitfield =
    kScaleXBitfield | kScaleYBitfield | kScaleZBitfield;

AnimData::AnimData(const Tolerances& tolerances, bool root_bones_only,
                   fplutil::Logger& log)
    : cur_bone_index_(-1),
      tolerances_(tolerances),
      root_bones_only_(root_bones_only),
      log_(log) {}

unsigned int AnimData::AllocBone(const char* bone_name, int parent_bone_index) {
  const unsigned int bone_index = static_cast<unsigned int>(bones_.size());
  bones_.push_back(Bone(bone_name, parent_bone_index));
  return bone_index;
}

void AnimData::SetCurBoneIndex(unsigned int cur_bone_index) {
  assert(cur_bone_index < bones_.size());
  assert(cur_bone_index_ == -1);
  cur_bone_index_ = cur_bone_index;
}

void AnimData::ResetCurBoneIndex() { cur_bone_index_ = -1; }

FlatChannelId AnimData::AllocChannel(MatrixOperationType op, MatrixOpId id) {
  Channels& channels = CurChannels();
  channels.push_back(Channel(op, id));
  return static_cast<FlatChannelId>(channels.size() - 1);
}

bool AnimData::ShouldRecurse(unsigned int cur_bone_index) const {
  // When searching for just the root bones, keep recursing until we find
  // a bone that has animation data.
  return !root_bones_only_ || bones_[cur_bone_index].channels.empty();
}

void AnimData::AddConstant(FlatChannelId channel_id, FlatVal const_val) {
  Channels& channels = CurChannels();
  Nodes& n = channels[channel_id].nodes;
  n.resize(0);
  n.push_back(SplineNode(0, const_val, 0.0f));
}

size_t AnimData::NumNodes(FlatChannelId channel_id) const {
  const Channels& channels = CurChannels();
  const Nodes& n = channels[channel_id].nodes;
  return n.size();
}

void AnimData::AddCurve(FlatChannelId channel_id, FlatTime time_start,
                        FlatTime time_end, const FlatVal* vals,
                        const FlatDerivative* derivatives, size_t count) {
  // Break the curve down into segments and process them depth-first so that the
  // resulting nodes are in chronological order.
  std::vector<CurveSegment> segments;
  segments.emplace_back(time_start, time_end, vals, derivatives, count);

  while (!segments.empty()) {
    CurveSegment s = segments.back();
    segments.pop_back();

    // Create cubic that covers the entire range from s.time_start ~ s.time_end.
    // The cubic `c` is shifted to the left, to start at 0 instead of
    // s.time_start. This is to maintain floating-point precision.
    const float time_width = static_cast<float>(s.time_end - s.time_start);
    const CubicCurve c(CubicInit(s.vals[0], s.derivatives[0],
                                 s.vals[s.count - 1],
                                 s.derivatives[s.count - 1], time_width));

    // Find the worst intermediate val in for this cubic.
    // That is, the index into `s.vals` where the cubic evaluation is most
    // inaccurate.
    const float time_inc = time_width / (s.count - 1);
    float time = time_inc;
    float worst_diff = 0.0f;
    float worst_time = 0.0f;
    size_t worst_idx = 0;
    for (size_t i = 1; i < s.count - 1; ++i) {
      const float cubic_val = c.Evaluate(time);
      const float curve_val = s.vals[i];
      const float diff_val = fabs(cubic_val - curve_val);
      if (diff_val > worst_diff) {
        worst_idx = i;
        worst_diff = diff_val;
        worst_time = time;
      }
      time += time_inc;
    }

    // If the cubic is off by a lot, divide the curve into two curves at the
    // worst time. Note that the recursion will end, at worst, when
    // s.count ==> 2.
    const float tolerance = Tolerance(channel_id);
    if (worst_idx > 0 && worst_diff > tolerance) {
      const FlatTime time_mid =
          s.time_start + static_cast<FlatTime>(worst_time);
      // Push the "end" segment on first so that the "start" segment is
      // processed first, resulting in a depth-first search.
      segments.emplace_back(time_mid, s.time_end, &s.vals[worst_idx],
                            &s.derivatives[worst_idx], s.count - worst_idx);
      segments.emplace_back(s.time_start, time_mid, s.vals, s.derivatives,
                            worst_idx + 1);
      continue;
    }

    // Otherwise, the generated cubic is good enough, so record it.
    const SplineNode start_node(s.time_start, s.vals[0], s.derivatives[0]);
    const SplineNode end_node(s.time_end, s.vals[s.count - 1],
                              s.derivatives[s.count - 1]);

    // Only push the start node if it differs from the previously pushed end
    // node. Most of the time it will be the same.
    Channels& channels = CurChannels();
    Nodes& n = channels[channel_id].nodes;
    const bool start_matches_prev = !n.empty() && n.back() == start_node;
    if (!start_matches_prev) {
      n.push_back(start_node);
    }
    n.push_back(end_node);
  }
}

void AnimData::PruneNodes(FlatChannelId channel_id) {
  const float tolerance = Tolerance(channel_id);

  // For every node try to prune as many redunant nodes that come after it.
  // A node is redundant if the spline evaluates to the same value even if
  // it doesn't exists (note: here "same value" means within `tolerances_`).
  Channels& channels = CurChannels();
  Nodes& n = channels[channel_id].nodes;
  std::vector<bool> prune(n.size(), false);
  for (size_t i = 0; i < n.size();) {
    size_t next_i = i + 1;
    for (size_t j = i + 2; j < n.size(); ++j) {
      const bool redundant =
          IntermediateNodesRedundant(&n[i], j - i + 1, tolerance);
      if (redundant) {
        prune[j - 1] = true;
        next_i = j;
      }
    }
    i = next_i;
  }

  // Compact to remove all pruned nodes.
  size_t write = 0;
  for (size_t read = 0; read < n.size(); ++read) {
    if (prune[read]) continue;
    if (write < read) {
      n[write] = n[read];
    }
    write++;
  }
  n.resize(write);

  // If value is constant for the entire time, remove the second node so that
  // we know to output a constant value in `OutputFlatBuffer()`.
  const bool is_const =
      n.size() == 2 && fabs(n[0].val - n[1].val) < tolerance &&
      fabs(DerivativeAngle(n[0].derivative)) < tolerances_.derivative_angle &&
      fabs(DerivativeAngle(n[1].derivative)) < tolerances_.derivative_angle;
  if (is_const) {
    n.resize(1);
  }
}

void AnimData::PruneChannels(bool no_uniform_scale) {
  for (auto bone = bones_.begin(); bone != bones_.end(); ++bone) {
    // Iterate from the end to minimize the cost of the erase operations.
    Channels& channels = bone->channels;
    for (FlatChannelId ch = static_cast<FlatChannelId>(channels.size() - 1);
         ch >= 0; ch--) {
      // Collapse kScaleX,Y,Z into kScaleUniformly.
      const bool uniform_scale =
          (!no_uniform_scale && UniformScaleChannels(channels, ch));
      if (uniform_scale) {
        log_.Log(fplutil::kLogVerbose,
                 "  Collapsing scale x, y, z channels %d~%d into"
                 " one scale-uniformly channel\n",
                 ch, ch + 2);

        // Ids values are in consecutive order
        //   scale-X id, scale-Y id, scale-Z id, scale-uniformly id
        // the same as op values are in consecutive order
        //   kScaleX, kScaleY, kScaleZ, kScaleUniformly
        // but with a different initial value.
        //
        // So to convert from scale-? id to scale-uniformly id, we add on
        // the difference kScaleUniformly - kScale?.
        channels[ch].id +=
            motive::kScaleUniformly - static_cast<MatrixOpId>(channels[ch].op);
        channels[ch].op = motive::kScaleUniformly;
        channels.erase(channels.begin() + (ch + 1),
                       channels.begin() + (ch + 3));
      }

      // Sum together channels that are adjacent, or separated only by
      // independent ops.
      const FlatChannelId summable_ch = SummableChannel(channels, ch);
      if (summable_ch >= 0) {
        log_.Log(fplutil::kLogVerbose, "  Summing %s channels %d and %d\n",
                 MatrixOpName(channels[ch].op), ch, summable_ch);

        SumChannels(channels, ch, summable_ch);
        channels.erase(channels.begin() + summable_ch);
      }

      // Remove constant channels that have the default value.
      // Most of the time these won't be created, but it's possible that
      // of the collapse operations above (especially summing) will create
      // this situation.
      if (channels[ch].nodes.size() == 1 &&
          IsDefaultValue(channels[ch].op, channels[ch].nodes[0].val)) {
        log_.Log(fplutil::kLogVerbose, "  Omitting constant %s channel %d\n",
                 MatrixOpName(channels[ch].op), ch);
        channels.erase(channels.begin() + ch);
      }
    }

    // Ensure that the channels remain in accending order of id.
    std::sort(channels.begin(), channels.end());
  }
}

void AnimData::ShiftTime(FlatTime time_offset) {
  if (time_offset == 0) return;
  log_.Log(fplutil::kLogImportant, "Shifting animation by %d ticks.\n",
           time_offset);

  for (auto bone = bones_.begin(); bone != bones_.end(); ++bone) {
    for (auto ch = bone->channels.begin(); ch != bone->channels.end(); ++ch) {
      for (auto n = ch->nodes.begin(); n != ch->nodes.end(); ++n) {
        n->time += time_offset;
      }
    }
  }
}

void AnimData::ExtendChannelsToTime(FlatTime end_time) {
  for (auto bone = bones_.begin(); bone != bones_.end(); ++bone) {
    Channels& channels = bone->channels;
    for (auto ch = channels.begin(); ch != channels.end(); ++ch) {
      Nodes& n = ch->nodes;

      // Ignore empty or constant channels.
      if (n.size() <= 1) continue;

      // Ignore channels that are already long enough.
      const SplineNode back = n.back();
      if (back.time >= end_time) continue;

      // Append a point with 0 derivative at the back, if required.
      // This ensures that the extra segment is a flat line.
      if (back.derivative != 0) {
        n.push_back(SplineNode(back.time, back.val, 0.0f));
      }

      // Append a point at the end time, also with 0 derivative.
      n.push_back(SplineNode(end_time, back.val, 0.0f));
    }
  }
}

float AnimData::ToleranceForOp(MatrixOperationType op) const {
  return motive::RotateOp(op)
             ? tolerances_.rotate
             : motive::TranslateOp(op)
                   ? tolerances_.translate
                   : motive::ScaleOp(op) ? tolerances_.scale : 0.1f;
}

float AnimData::ToleranceForDerivativeAngle() const {
  return tolerances_.derivative_angle;
}

bool AnimData::IsDefaultValue(MatrixOperationType op, float value) const {
  return fabs(value - DefaultOpValue(op)) < ToleranceForOp(op);
}

int AnimData::NumBytes() const {
  static const size_t kBytesPerSplineNode = 6;
  size_t num_bytes =
      sizeof(motive::RigAnim) + bones_.size() * sizeof(motive::MatrixAnim);

  for (size_t i = 0; i < bones_.size(); ++i) {
    auto channels = bones_[i].channels;

    num_bytes += channels.size() * sizeof(motive::MatrixOperationInit);
    for (size_t j = 0; j < channels.size(); ++j) {
      Nodes nodes = channels[j].nodes;
      num_bytes += sizeof(CompactSpline) + nodes.size() * kBytesPerSplineNode;
    }
  }
  return static_cast<int>(num_bytes);
}

FlatTime AnimData::MaxAnimatedTime() const {
  FlatTime max_time = std::numeric_limits<FlatTime>::min();
  for (auto bone = bones_.begin(); bone != bones_.end(); ++bone) {
    for (auto ch = bone->channels.begin(); ch != bone->channels.end(); ++ch) {
      // Only consider channels with more than one keyframe (non-constant).
      if (ch->nodes.size() > 1) {
        max_time = std::max(max_time, ch->nodes.back().time);
      }
    }
  }
  return max_time == std::numeric_limits<FlatTime>::min() ? 0 : max_time;
}

FlatTime AnimData::MinAnimatedTime() const {
  FlatTime min_time = std::numeric_limits<FlatTime>::max();
  for (auto bone = bones_.begin(); bone != bones_.end(); ++bone) {
    for (auto ch = bone->channels.begin(); ch != bone->channels.end(); ++ch) {
      // Only consider channels with more than one keyframe (non-constant).
      if (ch->nodes.size() > 1) {
        min_time = std::min(min_time, ch->nodes[0].time);
      }
    }
  }
  return min_time == std::numeric_limits<FlatTime>::max() ? 0 : min_time;
}

AnimData::Channels& AnimData::CurChannels() {
  assert(static_cast<unsigned int>(cur_bone_index_) < bones_.size());
  return bones_[cur_bone_index_].channels;
}

const AnimData::Channels& AnimData::CurChannels() const {
  assert(static_cast<unsigned int>(cur_bone_index_) < bones_.size());
  return bones_[cur_bone_index_].channels;
}

float AnimData::Tolerance(FlatChannelId channel_id) const {
  const Channels& channels = CurChannels();
  return ToleranceForOp(channels[channel_id].op);
}

BoneIndex AnimData::FirstNonRepeatingBone(
    FlatChannelId* first_channel_id) const {
  for (BoneIndex bone_idx = 0; bone_idx < bones_.size(); ++bone_idx) {
    const Bone& bone = bones_[bone_idx];
    const Channels& channels = bone.channels;

    for (FlatChannelId channel_id = 0;
         channel_id < static_cast<FlatChannelId>(channels.size());
         ++channel_id) {
      const Channel& channel = channels[channel_id];

      // Get deltas for the start and end of the channel.
      const SplineNode& start = channel.nodes.front();
      const SplineNode& end = channel.nodes.back();
      const float diff_val = fabs(start.val - end.val);
      const float diff_derivative_angle =
          fabs(DerivativeAngle(start.derivative - end.derivative));

      // Return false unless the start and end of the channel are the same.
      const float tolerance = ToleranceForOp(channel.op);
      const bool same =
          diff_val < tolerance &&
          diff_derivative_angle < tolerances_.repeat_derivative_angle;
      if (!same) {
        *first_channel_id = channel_id;
        return bone_idx;
      }
    }
  }
  return kInvalidBoneIdx;
};

bool AnimData::Repeat(RepeatPreference repeat_preference) const {
  if (repeat_preference == kNeverRepeat) return false;

  // Check to see if the animation is repeatable.
  FlatChannelId channel_id = 0;
  const BoneIndex bone_idx = FirstNonRepeatingBone(&channel_id);
  const bool repeat =
      repeat_preference == kAlwaysRepeat ||
      (repeat_preference == kRepeatIfRepeatable && bone_idx == kInvalidBoneIdx);

  // Log repeat information.
  if (repeat_preference == kAlwaysRepeat) {
    if (bone_idx != kInvalidBoneIdx) {
      const Bone& bone = bones_[bone_idx];
      const Channel& channel = bone.channels[channel_id];
      log_.Log(fplutil::kLogWarning,
               "Animation marked as repeating (as requested),"
               " but it does not repeat on bone %s's"
               " `%s` channel\n",
               bone.name.c_str(), MatrixOpName(channel.op));
    }
  } else if (repeat_preference == kRepeatIfRepeatable) {
    log_.Log(fplutil::kLogVerbose,
             repeat ? "Animation repeats.\n" : "Animation does not repeat.\n");
  }

  return repeat;
}

bool AnimData::UniformScaleChannels(const Channels& channels,
                                    FlatChannelId channel_id) const {
  if (channel_id + 2 >= static_cast<FlatChannelId>(channels.size()))
    return false;

  // Consider the three channels starting at `channel_id`.
  const Channel& c0 = channels[channel_id];
  const Channel& c1 = channels[channel_id + 1];
  const Channel& c2 = channels[channel_id + 2];

  // The order is not important, but we need kScaleX, Y, and Z.
  const uint32_t op_bits = (1 << c0.op) | (1 << c1.op) | (1 << c2.op);
  if (op_bits != kScaleXyzBitfield) return false;

  // The sequence of values must also be identical.
  const Nodes& n0 = c0.nodes;
  const Nodes& n1 = c1.nodes;
  const Nodes& n2 = c2.nodes;
  const bool same_length = n0.size() == n1.size() && n0.size() == n2.size() &&
                           n1.size() == n2.size();
  if (!same_length) return false;

  // The splines must be equal.
  const float tolerance = tolerances_.scale;
  for (size_t i = 0; i < n0.size(); ++i) {
    const SplineNode v0 = n0[i];
    const SplineNode v1 = n1[i];
    const SplineNode v2 = n2[i];
    const bool are_equal =
        EqualNodes(v0, v1, tolerance, tolerances_.derivative_angle) &&
        EqualNodes(v0, v2, tolerance, tolerances_.derivative_angle) &&
        EqualNodes(v1, v2, tolerance, tolerances_.derivative_angle);
    if (!are_equal) return false;
  }

  return true;
}

FlatChannelId AnimData::SummableChannel(const Channels& channels,
                                        FlatChannelId ch) const {
  const MatrixOperationType ch_op = channels[ch].op;
  for (FlatChannelId id = ch + 1;
       id < static_cast<FlatChannelId>(channels.size()); ++id) {
    const MatrixOperationType id_op = channels[id].op;

    // If we're adjacent to a similar op, we can combine by summing.
    if (id_op == ch_op) return id;

    // Rotate ops cannot have other ops inbetween them and still be combined.
    if (RotateOp(ch_op)) return -1;

    // Translate and scale ops can only have, respectively, other translate
    // and scale ops in between them.
    if (TranslateOp(ch_op) && !TranslateOp(id_op)) return -1;
    if (ScaleOp(ch_op) && !ScaleOp(id_op)) return -1;
  }
  return -1;
}

FlatVal AnimData::EvaluateNodes(const Nodes& nodes, FlatTime time,
                                FlatDerivative* derivative) {
  assert(nodes.size() > 0);

  // Handle before and after curve cases.
  *derivative = 0.0f;
  if (time < nodes.front().time) return nodes.front().val;
  if (time >= nodes.back().time) return nodes.back().val;

  // Find first node after `time`.
  size_t i = 1;
  for (;; ++i) {
    assert(i < nodes.size());
    if (nodes[i].time >= time) break;
  }
  const SplineNode& pre = nodes[i - 1];
  const SplineNode& post = nodes[i];
  assert(pre.time <= time && time <= post.time);

  // Create a cubic from before time to after time, and interpolate values
  // with it.
  const float cubic_total_time = static_cast<float>(post.time - pre.time);
  const float cubic_time = static_cast<float>(time - pre.time);
  const CubicCurve cubic(CubicInit(pre.val, pre.derivative, post.val,
                                   post.derivative, cubic_total_time));
  *derivative = cubic.Derivative(cubic_time);
  return cubic.Evaluate(cubic_time);
}

bool AnimData::GetValueAtTime(const Nodes& nodes,
                              const Nodes::const_iterator& node, FlatTime time,
                              FlatVal* value,
                              FlatDerivative* derivative) const {
  if (node != nodes.end() && node->time == time) {
    *value = node->val;
    *derivative = node->derivative;
    return true;
  } else {
    *value = EvaluateNodes(nodes, time, derivative);
    return false;
  }
}

void AnimData::SumChannels(Channels& channels, FlatChannelId ch_a,
                           FlatChannelId ch_b) const {
  const Nodes& nodes_a = channels[ch_a].nodes;
  const Nodes& nodes_b = channels[ch_b].nodes;
  Nodes sum;

  // TODO(b/66226797): The following assumes that the key on constant channels
  // is not significant to its evaluation. With pre/post infinities, single
  // key curves might not necessarily be "constant" curves. We should validate
  // if elsewhere that assumption is also made.
  //
  // If there is only one key, we ignore it because we can sample the curve
  // at any time, and don't want its key time to affect the resulting curve.
  auto node_iter_a = (nodes_a.size() == 1) ? nodes_a.end() : nodes_a.begin();
  auto node_iter_b = (nodes_b.size() == 1) ? nodes_b.end() : nodes_b.begin();

  // If both channels are constant, the curve should just contain a single
  // key with the sum.  Time and derivative are ignored in constant channels.
  if (nodes_a.size() == 1 && nodes_b.size() == 1) {
    sum.push_back(SplineNode(0, nodes_a[0].val + nodes_b[0].val, 0.0f));
  }

  while (node_iter_a != nodes_a.end() || node_iter_b != nodes_b.end()) {
    FlatTime time = std::numeric_limits<FlatTime>::max();
    if (node_iter_a != nodes_a.end()) {
      time = node_iter_a->time;
    }
    if (node_iter_b != nodes_b.end()) {
      time = std::min(time, node_iter_b->time);
    }

    FlatVal a, b;
    FlatDerivative da, db;
    if (GetValueAtTime(nodes_a, node_iter_a, time, &a, &da)) {
      ++node_iter_a;
    }
    if (GetValueAtTime(nodes_b, node_iter_b, time, &b, &db)) {
      ++node_iter_b;
    }
    sum.push_back(SplineNode(time, a + b, da + db));
  }

  channels[ch_a].nodes = sum;
}

BoneIndex AnimData::BoneParent(int bone_idx) const {
  const int parent_bone_index = bones_[bone_idx].parent_bone_index;
  return parent_bone_index < 0 ? kInvalidBoneIdx
                               : static_cast<BoneIndex>(parent_bone_index);
}

bool AnimData::IntermediateNodesRedundant(const SplineNode* n, size_t len,
                                          float tolerance) const {
  // If the start and end nodes occur at the same time and are equal,
  // then ignore everything inbetween them.
  const SplineNode& start = n[0];
  const SplineNode& end = n[len - 1];
  if (EqualNodes(start, end, tolerance, tolerances_.derivative_angle))
    return true;

  // Construct cubic curve `c` that skips all the intermediate nodes.
  const float cubic_width = static_cast<float>(end.time - start.time);
  const CubicCurve c(CubicInit(start.val, start.derivative, end.val,
                               end.derivative, cubic_width));

  // For each intermediate node, check if the cubic `c` is close.
  for (size_t i = 1; i < len - 1; ++i) {
    // Evaluate `c` at the time of `mid`.
    const SplineNode& mid = n[i];
    const float mid_time = static_cast<float>(mid.time - start.time);
    const float mid_val = c.Evaluate(mid_time);
    const float mid_derivative = c.Derivative(mid_time);

    // If the mid point is on the curve, it's redundant.
    const float derivative_angle_error =
        DerivativeAngle(mid_derivative - mid.derivative);
    const bool mid_on_c =
        fabs(mid_val - mid.val) < tolerance &&
        fabs(derivative_angle_error) < tolerances_.derivative_angle;
    if (!mid_on_c) return false;
  }

  // All mid points are redundant.
  return true;
}

bool AnimData::EqualNodes(const SplineNode& a, const SplineNode& b,
                          float tolerance, float derivative_tolerance) {
  return a.time == b.time && fabs(a.val - b.val) < tolerance &&
         fabs(DerivativeAngle(a.derivative - b.derivative)) <
             derivative_tolerance;
}

FlatVal AnimData::DefaultOpValue(MatrixOperationType op) {
  // Translate and rotate operations are 0 by default.
  // Scale operations are 1 by default.
  return motive::ScaleOp(op) ? 1.0f : 0.0f;
}

}  // namespace motive
