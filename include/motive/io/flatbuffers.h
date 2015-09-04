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

#ifndef MOTIVE_IO_FLATBUFFERS_H_
#define MOTIVE_IO_FLATBUFFERS_H_

namespace motive {

class AnimTable;
class MatrixAnim;
struct MatrixAnimFb;
class OvershootInit;
struct OvershootParameters;
class RigAnim;
struct RigAnimFb;
class SmoothInit;
struct SmoothParameters;
struct Settled1f;
struct Settled1fParameters;

/// Load `file_name` into `buf` and return true on success, false on failure.
/// Motive does not make assumptions on file io, so the caller must provide
/// this function. Files should loaded in binary format.
typedef bool LoadFileFn(const char* file_name, std::string* buf);

/// Convert from FlatBuffer params to Motive init, for Overshoot.
void OvershootInitFromFlatBuffers(const OvershootParameters& params,
                                  OvershootInit* init);

/// Convert from FlatBuffer params to Motive init, for Smooth.
void SmoothInitFromFlatBuffers(const SmoothParameters& params,
                               SmoothInit* init);

/// Convert from FlatBuffer params to Motive Settled1f.
void Settled1fFromFlatBuffers(const Settled1fParameters& params,
                              Settled1f* settled);

/// Convert from FlatBuffer params to Motive MatrixAnim.
void MatrixAnimFromFlatBuffers(const MatrixAnimFb& params, bool repeat,
                               MatrixAnim* anim);

/// Convert from FlatBuffer params to Motive MatrixAnim.
void RigAnimFromFlatBuffers(const RigAnimFb& params, RigAnim* anim);

/// Initialize all animations in `anim_table`. Assume that each `anim_name` is
/// a file name, and load it with `load_fn`. Then call
/// MatrixAnimFromFlatBuffers for each file to initialize all the animations.
/// Returns the `anim_name` that couldn't be loaded, on failure, or nullptr,
/// on success.
/// Note: Every platform will have its own way of handling binary data. This
///       utility function is provided mostly as an example for how to
///       initialize an AnimTable.
const char* LoadAnimTableAnimations(AnimTable* anim_table, LoadFileFn* load_fn);

}  // namespace motive

#endif  // MOTIVE_IO_FLATBUFFERS_H_
