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

MOTIVE_DIR:=$(PROJECT_ROOT)
include $(PROJECT_ROOT)/jni/android_config.mk

include $(CLEAR_VARS)
LOCAL_MODULE:=motive_$(MOTIVE_APP_NAME)
LOCAL_SRC_FILES:=$(MOTIVE_APP_NAME).cpp
LOCAL_WHOLE_STATIC_LIBRARIES:=android_native_app_glue libfplutil_main \
  libfplutil_print libgtest libmathfu motive
LOCAL_LDLIBS:=-llog -landroid
LOCAL_ARM_MODE:=arm
LOCAL_C_INCLUDES:= \
	$(PROJECT_ROOT)/include \
	$(DEPENDENCIES_FLATBUFFERS_DIR)/include
include $(BUILD_SHARED_LIBRARY)

$(call import-add-path,$(PROJECT_ROOT)/..)
$(call import-add-path,$(DEPENDENCIES_GTEST_DIR)/..)
$(call import-add-path,$(DEPENDENCIES_FPLUTIL_DIR)/..)

$(call import-module,android/native_app_glue)
$(call import-module,googletest)
$(call import-module,fplutil/libfplutil/jni)
$(call import-module,motive/jni)
