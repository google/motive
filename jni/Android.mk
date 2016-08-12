# Copyright 2015 Google Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

LOCAL_PATH := $(call my-dir)

MOTIVE_RELATIVE_DIR := ..
MOTIVE_DIR := $(LOCAL_PATH)/$(MOTIVE_RELATIVE_DIR)

include $(MOTIVE_DIR)/jni/android_config.mk
include $(DEPENDENCIES_FLATBUFFERS_DIR)/android/jni/include.mk

# realpath-portable From flatbuffers/android/jni/include.mk
LOCAL_PATH := $(call realpath-portable,$(LOCAL_PATH))
MOTIVE_DIR := $(call realpath-portable,$(MOTIVE_DIR))

MOTIVE_STATIC_LIBRARIES := libmathfu cpufeatures

MOTIVE_EXPORT_C_INCLUDES := \
  $(MOTIVE_DIR)/include \
  $(MOTIVE_GENERATED_OUTPUT_DIR) \
  $(DEPENDENCIES_MATHFU_DIR)/benchmarks

MOTIVE_C_INCLUDES := \
  $(MOTIVE_EXPORT_C_INCLUDES) \
  $(MOTIVE_DIR)/src/motive \
  $(DEPENDENCIES_FLATBUFFERS_DIR)/include \
  $(DEPENDENCIES_FPLUTIL_DIR)/libfplutil/include

MOTIVE_SRC_FILES := \
  $(MOTIVE_RELATIVE_DIR)/src/motive/anim.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/motive/anim_table.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/motive/engine.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/motive/init.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/motive/io/flatbuffers.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/motive/math/angle.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/motive/math/bulk_spline_evaluator.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/motive/math/compact_spline.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/motive/math/curve.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/motive/math/dual_cubic.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/motive/motivator.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/motive/processor/matrix_processor.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/motive/processor/overshoot_processor.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/motive/processor/rig_processor.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/motive/processor/spline_processor.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/motive/processor.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/motive/util/benchmark.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/motive/util/optimizations.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/motive/version.cpp

MOTIVE_CFLAGS:=

MOTIVE_ENABLE_ASSEMBLY ?= 1
MOTIVE_TEST_ASSEMBLY ?= 0
ifneq ($(MOTIVE_ENABLE_ASSEMBLY),0)
  # Presently, we only have assembly functions for 'armeabi-v7a' and
  # 'armeabi-v7a-hard'.
  ifneq (,$(findstring armeabi-v7a,$(TARGET_ARCH_ABI)))
    # Use the .neon extension to compile with NEON support.
    MOTIVE_SRC_FILES += \
      $(MOTIVE_RELATIVE_DIR)/src/motive/math/bulk_spline_evaluator_neon.s.neon
    MOTIVE_CFLAGS += -DMOTIVE_NEON

    # Run both NEON and C++ code and compare results.
    ifneq ($(MOTIVE_TEST_ASSEMBLY),0)
      MOTIVE_CFLAGS += -DMOTIVE_ASSEMBLY_TEST=Neon
    endif
  endif
endif

include $(CLEAR_VARS)
LOCAL_MODULE := motive
LOCAL_ARM_MODE := arm
LOCAL_STATIC_LIBRARIES := $(MOTIVE_STATIC_LIBRARIES)
LOCAL_EXPORT_C_INCLUDES := $(MOTIVE_EXPORT_C_INCLUDES)
LOCAL_C_INCLUDES := $(MOTIVE_C_INCLUDES)
LOCAL_CFLAGS += $(MOTIVE_CFLAGS)
LOCAL_CPPFLAGS += $(MOTIVE_CFLAGS)
LOCAL_SRC_FILES := $(MOTIVE_SRC_FILES)
include $(BUILD_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := motive_benchmark
LOCAL_ARM_MODE := arm
LOCAL_STATIC_LIBRARIES := $(MOTIVE_STATIC_LIBRARIES)
LOCAL_EXPORT_C_INCLUDES := $(MOTIVE_EXPORT_C_INCLUDES)
LOCAL_C_INCLUDES := $(MOTIVE_C_INCLUDES)
LOCAL_CFLAGS += $(MOTIVE_CFLAGS) -DBENCHMARK_MOTIVE
LOCAL_SRC_FILES := $(MOTIVE_SRC_FILES)
LOCAL_EXPORT_CFLAGS := -DBENCHMARK_MOTIVE
include $(BUILD_STATIC_LIBRARY)


MOTIVE_SCHEMA_DIR := $(MOTIVE_DIR)/schemas
MOTIVE_SCHEMA_INCLUDE_DIRS :=

MOTIVE_SCHEMA_FILES := \
  $(MOTIVE_SCHEMA_DIR)/anim.fbs \
  $(MOTIVE_SCHEMA_DIR)/anim_table.fbs \
  $(MOTIVE_SCHEMA_DIR)/anim_list.fbs \
  $(MOTIVE_SCHEMA_DIR)/compact_spline.fbs \
  $(MOTIVE_SCHEMA_DIR)/motive.fbs \
  $(MOTIVE_SCHEMA_DIR)/spline_anim.fbs

ifeq (,$(MOTIVE_RUN_ONCE))
MOTIVE_RUN_ONCE := 1
$(call flatbuffers_header_build_rules, \
  $(MOTIVE_SCHEMA_FILES), \
  $(MOTIVE_SCHEMA_DIR), \
  $(MOTIVE_GENERATED_OUTPUT_DIR), \
  $(MOTIVE_SCHEMA_INCLUDE_DIRS), \
  $(MOTIVE_SRC_FILES), \
  motive_generated_includes)
endif


$(call import-add-path,$(DEPENDENCIES_FLATBUFFERS_DIR)/..)
$(call import-add-path,$(DEPENDENCIES_MATHFU_DIR)/..)

$(call import-module,flatbuffers/android/jni)
$(call import-module,mathfu/jni)
$(call import-module,android/cpufeatures)
