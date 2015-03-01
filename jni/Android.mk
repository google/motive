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

include $(CLEAR_VARS)

LOCAL_MODULE := motive
LOCAL_ARM_MODE := arm
LOCAL_STATIC_LIBRARIES := libmathfu
LOCAL_SHARED_LIBRARIES := SDL2

MOTIVE_RELATIVE_DIR := ..
MOTIVE_DIR := $(LOCAL_PATH)/$(MOTIVE_RELATIVE_DIR)

include $(MOTIVE_DIR)/jni/android_config.mk
include $(DEPENDENCIES_FLATBUFFERS_DIR)/android/jni/include.mk

MOTIVE_GENERATED_OUTPUT_DIR := $(MOTIVE_DIR)/gen/include

LOCAL_EXPORT_C_INCLUDES := \
  $(MOTIVE_DIR)/include \
  $(MOTIVE_GENERATED_OUTPUT_DIR)

LOCAL_C_INCLUDES := \
  $(LOCAL_EXPORT_C_INCLUDES) \
  $(MOTIVE_DIR)/src \
  $(DEPENDENCIES_FLATBUFFERS_DIR)/include \
  $(DEPENDENCIES_FPLUTIL_DIR)/libfplutil/include \
  $(DEPENDENCIES_SDL_DIR) \
  $(DEPENDENCIES_SDL_DIR)/include

LOCAL_SRC_FILES := \
  $(MOTIVE_RELATIVE_DIR)/src/engine.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/io/flatbuffers.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/math/bulk_spline_evaluator.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/math/compact_spline.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/math/curve.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/math/dual_cubic.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/motivator.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/processor/matrix_processor.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/processor/overshoot_processor.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/processor/smooth_processor.cpp \
  $(MOTIVE_RELATIVE_DIR)/src/processor.cpp

MOTIVE_SCHEMA_DIR := $(MOTIVE_DIR)/schemas
MOTIVE_SCHEMA_INCLUDE_DIRS :=

MOTIVE_SCHEMA_FILES := \
  $(MOTIVE_SCHEMA_DIR)/motive.fbs

ifeq (,$(MOTIVE_RUN_ONCE))
MOTIVE_RUN_ONCE := 1
$(call flatbuffers_header_build_rules, \
  $(MOTIVE_SCHEMA_FILES), \
  $(MOTIVE_SCHEMA_DIR), \
  $(MOTIVE_GENERATED_OUTPUT_DIR), \
  $(MOTIVE_SCHEMA_INCLUDE_DIRS), \
  $(LOCAL_SRC_FILES))
endif

include $(BUILD_STATIC_LIBRARY)

$(call import-add-path,$(DEPENDENCIES_FLATBUFFERS_DIR)/..)
$(call import-add-path,$(DEPENDENCIES_MATHFU_DIR)/..)

$(call import-module,flatbuffers/android/jni)
$(call import-module,mathfu/jni)

