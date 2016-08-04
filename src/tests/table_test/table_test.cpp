// Copyright 2016 Google Inc. All rights reserved.
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
#include "anim_table_generated.h"
#include "gtest/gtest.h"
#include "motive/anim_table.h"

using motive::AnimTable;
using motive::AnimListFb;
using motive::AnimTableFb;
using motive::AnimSource;

enum AnimTableInitMethod {
  kInitFromNames,
  kInitFromFlatBufferNames,
  kInitFromFlatBufferEmbedded,
};

class TableTests : public ::testing::Test {
 protected:
  virtual void SetUp() {}
  virtual void TearDown() {}
};

#define TEST_ALL_INIT_METHODS(TestFn)                           \
  TEST_F(TableTests, TestFn##Names) { TestFn(kInitFromNames); } \
  TEST_F(TableTests, TestFn##FlatBufferNames) {                 \
    TestFn(kInitFromFlatBufferNames);                           \
  }                                                             \
  TEST_F(TableTests, TestFn##FlatBufferEmbedded) {              \
    TestFn(kInitFromFlatBufferEmbedded);                        \
  }

// For internal mock data, only say files that start with "valid" are valid.
static bool ValidTestName(const char* name) {
  return strncmp(name, "valid", 5) == 0;
}

static flatbuffers::Offset<motive::RigAnimFb> CreateRigAnimFbOffset(
    flatbuffers::FlatBufferBuilder& fbb, const std::string& name) {
  return ValidTestName(name.c_str())
             ? motive::CreateRigAnimFb(fbb, 0, 0, 0, false,
                                       fbb.CreateString(name))
             : 0;
}

static const char* RigAnimFbLoadFn(const char* file_name,
                                   std::string* scratch_buf) {
  // Construct a RigAnimFb FlatBuffer.
  flatbuffers::FlatBufferBuilder fbb;
  auto rig_anim_fb = CreateRigAnimFbOffset(fbb, file_name);
  if (rig_anim_fb.o == 0) return nullptr;
  FinishRigAnimFbBuffer(fbb, rig_anim_fb);

  // Copy it into the scratch buffer and return a pointer to it.
  scratch_buf->assign(reinterpret_cast<const char*>(fbb.GetBufferPointer()),
                      fbb.GetSize());
  return scratch_buf->c_str();
}

static flatbuffers::Offset<AnimSource> CreateAnimSourceOffset(
    flatbuffers::FlatBufferBuilder& fbb, const std::string& name,
    motive::AnimSourceUnion source) {
  flatbuffers::Offset<void> u;
  switch (source) {
    case motive::AnimSourceUnion_AnimSourceFileName:
      u = motive::CreateAnimSourceFileName(fbb, fbb.CreateString(name)).Union();
      break;

    case motive::AnimSourceUnion_AnimSourceEmbedded:
      u = motive::CreateAnimSourceEmbedded(fbb,
                                           CreateRigAnimFbOffset(fbb, name))
              .Union();
      break;

    default:
      assert(false);
  }
  return motive::CreateAnimSource(fbb, source, u);
}

static flatbuffers::Offset<AnimListFb> CreateAnimListFbOffset(
    flatbuffers::FlatBufferBuilder& fbb, const AnimTable::ListFileNames& names,
    motive::AnimSourceUnion source) {
  std::vector<flatbuffers::Offset<AnimSource>> anims(names.size());
  anims.clear();
  for (auto it = names.begin(); it != names.end(); ++it) {
    anims.push_back(CreateAnimSourceOffset(fbb, *it, source));
  }
  auto anims_offset = fbb.CreateVector(anims);
  return motive::CreateAnimListFb(fbb, 0, anims_offset);
}

static const AnimListFb& CreateAnimListFb(flatbuffers::FlatBufferBuilder& fbb,
                                          const AnimTable::ListFileNames& names,
                                          motive::AnimSourceUnion source) {
  auto list_offset = CreateAnimListFbOffset(fbb, names, source);
  motive::FinishAnimListFbBuffer(fbb, list_offset);
  return *motive::GetAnimListFb(fbb.GetBufferPointer());
}

static const AnimTableFb& CreateAnimTableFb(
    flatbuffers::FlatBufferBuilder& fbb, const AnimTable::TableFileNames& names,
    motive::AnimSourceUnion source) {
  std::vector<flatbuffers::Offset<motive::AnimListFb>> lists(names.size());
  lists.clear();
  for (auto it = names.begin(); it != names.end(); ++it) {
    lists.push_back(CreateAnimListFbOffset(fbb, *it, source));
  }
  auto table_fb_offset = motive::CreateAnimTableFb(fbb, &lists);
  motive::FinishAnimTableFbBuffer(fbb, table_fb_offset);
  return *motive::GetAnimTableFb(fbb.GetBufferPointer());
}

static bool InitFromList(const AnimTable::ListFileNames& names,
                         AnimTableInitMethod method, AnimTable* table) {
  if (method == kInitFromNames)
    return table->InitFromAnimFileNames(names, RigAnimFbLoadFn);

  const motive::AnimSourceUnion source =
      method == kInitFromFlatBufferNames
          ? motive::AnimSourceUnion_AnimSourceFileName
          : motive::AnimSourceUnion_AnimSourceEmbedded;

  flatbuffers::FlatBufferBuilder fbb;
  const AnimListFb& list_fb = CreateAnimListFb(fbb, names, source);
  return table->InitFromFlatBuffers(list_fb, RigAnimFbLoadFn);
}

static bool InitFromTable(const AnimTable::TableFileNames& names,
                          AnimTableInitMethod method, AnimTable* table) {
  if (method == kInitFromNames)
    return table->InitFromAnimFileNames(names, RigAnimFbLoadFn);

  const motive::AnimSourceUnion source =
      method == kInitFromFlatBufferNames
          ? motive::AnimSourceUnion_AnimSourceFileName
          : motive::AnimSourceUnion_AnimSourceEmbedded;

  flatbuffers::FlatBufferBuilder fbb;
  const AnimTableFb& list_fb = CreateAnimTableFb(fbb, names, source);
  return table->InitFromFlatBuffers(list_fb, RigAnimFbLoadFn);
}

void ListEmpty(AnimTableInitMethod method) {
  AnimTable::ListFileNames names;
  AnimTable table;
  EXPECT_TRUE(InitFromList(names, method, &table));
  EXPECT_EQ(table.NumObjects(), 0);
  EXPECT_EQ(table.NumUniqueAnims(), 0);
}
TEST_ALL_INIT_METHODS(ListEmpty)

void ListMultipleValid(AnimTableInitMethod method) {
  AnimTable::ListFileNames names;
  names.push_back("valid1.motiveanim");
  names.push_back("valid2.motiveanim");
  names.push_back("valid3.motiveanim");

  AnimTable table;
  EXPECT_TRUE(InitFromList(names, method, &table));
  EXPECT_EQ(table.NumObjects(), 1);
  EXPECT_EQ(table.NumAnims(0), 3);
  EXPECT_EQ(table.NumUniqueAnims(), 3);
}
TEST_ALL_INIT_METHODS(ListMultipleValid)

void ListDuplicate(AnimTableInitMethod method) {
  AnimTable::ListFileNames names;
  names.push_back("valid1.motiveanim");
  names.push_back("valid2.motiveanim");
  names.push_back("valid1.motiveanim");

  AnimTable table;
  EXPECT_TRUE(InitFromList(names, method, &table));
  EXPECT_EQ(table.NumObjects(), 1);
  EXPECT_EQ(table.NumAnims(0), 3);
  EXPECT_EQ(table.NumUniqueAnims(), 2);
}
TEST_ALL_INIT_METHODS(ListDuplicate)

void ListConsecutiveDuplicates(AnimTableInitMethod method) {
  AnimTable::ListFileNames names;
  names.push_back("valid1.motiveanim");
  names.push_back("valid1.motiveanim");
  names.push_back("valid1.motiveanim");
  names.push_back("valid1.motiveanim");

  AnimTable table;
  EXPECT_TRUE(InitFromList(names, method, &table));
  EXPECT_EQ(table.NumObjects(), 1);
  EXPECT_EQ(table.NumAnims(0), 4);
  EXPECT_EQ(table.NumUniqueAnims(), 1);
}
TEST_ALL_INIT_METHODS(ListConsecutiveDuplicates)

void ListSingleInvalid(AnimTableInitMethod method) {
  AnimTable::ListFileNames names;
  names.push_back("valid1.motiveanim");
  names.push_back("invalid.motiveanim");
  names.push_back("valid2.motiveanim");
  names.push_back("valid3.motiveanim");

  AnimTable table;
  // When embedded, invalid animations don't trigger a file load failure,
  // so the init succeeds. Invalid animations are just empty slots in the table.
  EXPECT_FALSE(InitFromList(names, method, &table) &&
               method != kInitFromFlatBufferEmbedded);
  EXPECT_EQ(table.NumObjects(), 1);
  EXPECT_EQ(table.NumAnims(0), 4);
  EXPECT_EQ(table.NumUniqueAnims(), 3);
  EXPECT_NE(table.Query(0, 0), nullptr);
  EXPECT_EQ(table.Query(0, 1), nullptr);
}
TEST_ALL_INIT_METHODS(ListSingleInvalid)

void TableEmpty(AnimTableInitMethod method) {
  AnimTable::TableFileNames names;

  AnimTable table;
  EXPECT_TRUE(InitFromTable(names, method, &table));
  EXPECT_EQ(table.NumObjects(), 0);
  EXPECT_EQ(table.NumUniqueAnims(), 0);
}
TEST_ALL_INIT_METHODS(TableEmpty)

void TableMultipleValid(AnimTableInitMethod method) {
  AnimTable::TableFileNames names(2);
  names[0].push_back("valid1.motiveanim");
  names[0].push_back("valid2.motiveanim");
  names[1].push_back("valid3.motiveanim");
  names[1].push_back("valid4.motiveanim");
  names[1].push_back("valid5.motiveanim");

  AnimTable table;
  EXPECT_TRUE(InitFromTable(names, method, &table));
  EXPECT_EQ(table.NumObjects(), 2);
  EXPECT_EQ(table.NumAnims(0), 2);
  EXPECT_EQ(table.NumAnims(1), 3);
  EXPECT_EQ(table.NumUniqueAnims(), 5);
  EXPECT_NE(table.Query(0, 0), nullptr);
  EXPECT_NE(table.Query(0, 1), nullptr);
  EXPECT_NE(table.Query(1, 0), nullptr);
  EXPECT_NE(table.Query(1, 1), nullptr);
  EXPECT_NE(table.Query(1, 2), nullptr);
}
TEST_ALL_INIT_METHODS(TableMultipleValid)

void TableDuplicates(AnimTableInitMethod method) {
  AnimTable::TableFileNames names(2);
  names[0].push_back("valid1.motiveanim");
  names[0].push_back("valid1.motiveanim");
  names[1].push_back("valid1.motiveanim");
  names[1].push_back("valid1.motiveanim");
  names[1].push_back("valid2.motiveanim");

  AnimTable table;
  EXPECT_TRUE(InitFromTable(names, method, &table));
  EXPECT_EQ(table.NumObjects(), 2);
  EXPECT_EQ(table.NumAnims(0), 2);
  EXPECT_EQ(table.NumAnims(1), 3);
  EXPECT_EQ(table.NumUniqueAnims(), 2);
  EXPECT_NE(table.Query(0, 0), nullptr);
  EXPECT_NE(table.Query(0, 1), nullptr);
  EXPECT_NE(table.Query(1, 0), nullptr);
  EXPECT_NE(table.Query(1, 1), nullptr);
  EXPECT_NE(table.Query(1, 2), nullptr);
}
TEST_ALL_INIT_METHODS(TableDuplicates)

void TableInvalids(AnimTableInitMethod method) {
  AnimTable::TableFileNames names(2);
  names[0].push_back("valid1.motiveanim");
  names[0].push_back("invalid.motiveanim");
  names[1].push_back("invalid.motiveanim");
  names[1].push_back("invalid.motiveanim");
  names[1].push_back("invalid.motiveanim");

  AnimTable table;
  // When embedded, invalid animations don't trigger a file load failure,
  // so the init succeeds. Invalid animations are just empty slots in the table.
  EXPECT_FALSE(InitFromTable(names, method, &table) &&
               method != kInitFromFlatBufferEmbedded);
  EXPECT_EQ(table.NumObjects(), 2);
  EXPECT_EQ(table.NumAnims(0), 2);
  EXPECT_EQ(table.NumAnims(1), 3);
  EXPECT_EQ(table.NumUniqueAnims(), 1);
  EXPECT_NE(table.Query(0, 0), nullptr);
  EXPECT_EQ(table.Query(0, 1), nullptr);
  EXPECT_EQ(table.Query(1, 0), nullptr);
  EXPECT_EQ(table.Query(1, 1), nullptr);
  EXPECT_EQ(table.Query(1, 2), nullptr);
}
TEST_ALL_INIT_METHODS(TableInvalids)

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
