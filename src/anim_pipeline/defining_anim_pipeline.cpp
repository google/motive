// Copyright 2017 Google Inc. All rights reserved.
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

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "anim_generated.h"
#include "anim_list_generated.h"
#include "motive/anim_table.h"
#include "motive/rig_anim.h"

const char* LoadFile(const char* filename, std::string* scratch_buf) {
  std::ifstream fin(filename, std::ios::in | std::ios::binary);
  std::string data((std::istreambuf_iterator<char>(fin)),
                   std::istreambuf_iterator<char>());
  fin.close();
  if (scratch_buf) {
    *scratch_buf = std::move(data);
    return scratch_buf->c_str();
  }
  return nullptr;
}

bool SaveFile(const char* filename, const uint8_t* bytes, size_t num_bytes) {
  std::fstream fout(filename, std::ios::out | std::ios::binary);
  fout.write(reinterpret_cast<const char*>(bytes), num_bytes);
  fout.close();
  return true;
}

// Creates a RigAnimFb object using the FlatBufferBuilder from the provided
// RigAnim data.
bool CreateRigAnimFbFromRigAnim(flatbuffers::FlatBufferBuilder& fbb,
                                const motive::RigAnim& anim) {
  const uint8_t num_bones = anim.NumBones();

  std::vector<std::string> bone_names;
  for (uint8_t bone_idx = 0; bone_idx < num_bones; ++bone_idx) {
    bone_names.push_back(anim.BoneName(bone_idx));
  }

  const std::vector<uint8_t> bone_parents(anim.bone_parents(),
                                          anim.bone_parents() + num_bones);

  std::vector<flatbuffers::Offset<motive::MatrixAnimFb>> matrix_anims;
  for (uint8_t bone_idx = 0; bone_idx < num_bones; ++bone_idx) {
    const motive::MatrixAnim& matrix_anim = anim.Anim(bone_idx);

    // Convert the individual MatrixOps from the matrix animation into their
    // equivalent MatrixOpFb objects.
    //
    std::vector<flatbuffers::Offset<motive::MatrixOpFb>> ops;
    for (auto& op : matrix_anim.ops().ops()) {
      const motive::MatrixOperationTypeFb type =
          static_cast<motive::MatrixOperationTypeFb>(op.type);

      switch (op.union_type) {
        // "Empty" ops are spline operations without spline data.
        case motive::MatrixOperationInit::kUnionEmpty: {
          const motive::MatrixOpValueFb value_type =
              motive::MatrixOpValueFb_CompactSplineFb;
          ops.push_back(CreateMatrixOpFb(fbb, op.id, type, value_type));
          break;
        }
        // "InitialValue" ops are constant operations with a single value.
        case motive::MatrixOperationInit::kUnionInitialValue: {
          const motive::MatrixOpValueFb value_type =
              motive::MatrixOpValueFb_ConstantOpFb;
          const auto value =
              motive::CreateConstantOpFb(fbb, op.initial_value).Union();
          ops.push_back(CreateMatrixOpFb(fbb, op.id, type, value_type, value));
          break;
        }
        // We assume a "defining animation" operation should be either a
        // constant value operation or an "empty" spline operation as this is
        // how the CreateDefiningAnim in anim_table.cpp is implemented.
        default: {
          std::cerr << "Unsupported matrix operation type: " << op.union_type;
          return false;
        }
      }
    }

    matrix_anims.push_back(CreateMatrixAnimFbDirect(fbb, &ops));
  }

  auto root = CreateRigAnimFb(
      fbb, fbb.CreateVector(matrix_anims), fbb.CreateVector(bone_parents),
      fbb.CreateVectorOfStrings(bone_names), anim.repeat(),
      fbb.CreateString(anim.anim_name()));
  FinishRigAnimFbBuffer(fbb, root);
  return true;
}


int main(int argc, char** argv) {
  motive::AnimTable table;

  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " [output] [inputs...]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Pipeline to extract a defining motive animation [output] "
        "from a set of related motive animations [inputs]." << std::endl;
    return -1;
  }

  const std::string output_file = argv[1];
  std::vector<std::string> input_files;
  for (int i = 2; i < argc; ++i) {
    input_files.push_back(argv[i]);
  }

  if (!table.InitFromAnimFileNames(input_files, LoadFile)) {
    std::cerr << "Could not initialize anim table." << std::endl;
    return -1;
  }

  flatbuffers::FlatBufferBuilder fbb;
  const motive::RigAnim& defining_anim = table.DefiningAnim(0);

  if (!CreateRigAnimFbFromRigAnim(fbb, defining_anim)) {
    std::cerr << "Error processing defining anim." << std::endl;
    return -1;
  }

  if (!SaveFile(output_file.c_str(), fbb.GetBufferPointer(), fbb.GetSize())) {
    std::cerr << "Error saving file: " << output_file << std::endl;
    return -1;
  }
  return 0;
}
