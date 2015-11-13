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

LOCAL_MODULE := motive_benchmarker
LOCAL_ARM_MODE := arm
LOCAL_STATIC_LIBRARIES := libmathfu libmotive_benchmark

MOTIVE_RELATIVE_DIR := ../../..
MOTIVE_DIR := $(LOCAL_PATH)/$(MOTIVE_RELATIVE_DIR)

include $(MOTIVE_DIR)/jni/android_config.mk

LOCAL_EXPORT_C_INCLUDES := \
  $(MOTIVE_DIR)/include \
  $(MOTIVE_GENERATED_OUTPUT_DIR)

LOCAL_C_INCLUDES := \
  $(LOCAL_EXPORT_C_INCLUDES) \
  $(MOTIVE_DIR)/src \
  $(DEPENDENCIES_FLATBUFFERS_DIR)/include \
  $(DEPENDENCIES_FPLUTIL_DIR)/libfplutil/include

# Uncomment this to run both the C++ and assembly versions of functions
# and compare the results.
# MOTIVE_TEST_ASSEMBLY := 1

LOCAL_SRC_FILES := \
  $(MOTIVE_RELATIVE_DIR)/src/benchmarker/benchmarker.cpp

LOCAL_WHOLE_STATIC_LIBRARIES:=android_native_app_glue libfplutil_main \
  libfplutil_print

LOCAL_LDLIBS:=-llog -landroid

include $(BUILD_SHARED_LIBRARY)

$(call import-add-path,$(MOTIVE_DIR)/..)
$(call import-add-path,$(DEPENDENCIES_FLATBUFFERS_DIR)/..)
$(call import-add-path,$(DEPENDENCIES_MATHFU_DIR)/..)
$(call import-add-path,$(DEPENDENCIES_FPLUTIL_DIR))

$(call import-module,flatbuffers/android/jni)
$(call import-module,libfplutil/jni)
$(call import-module,mathfu/jni)
$(call import-module,motive/jni)


