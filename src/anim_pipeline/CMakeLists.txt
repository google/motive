# Copyright (c) 2015 Google, Inc.
#
# This software is provided 'as-is', without any express or implied
# warranty.  In no event will the authors be held liable for any damages
# arising from the use of this software.
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely, subject to the following restrictions:
# 1. The origin of this software must not be misrepresented; you must not
# claim that you wrote the original software. If you use this software
# in a product, an acknowledgment in the product documentation would be
# appreciated but is not required.
# 2. Altered source versions must be plainly marked as such, and must not be
# misrepresented as being the original software.
# 3. This notice may not be removed or altered from any source distribution.
cmake_minimum_required(VERSION 2.8.12)

project(anim_pipeline)

# This is the directory into which the executables are built.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/../../bin/${CMAKE_SYSTEM_NAME})

set(tmp_dir ${CMAKE_BINARY_DIR}/obj)
get_filename_component(fpl_root "${CMAKE_SOURCE_DIR}/../../.." ABSOLUTE)
set(dependencies_flatbuffers_dir "${fpl_root}/flatbuffers"
    CACHE PATH "Directory containing the Flatbuffers library.")
set(dependencies_fplutil_dir "${fpl_root}/fplutil"
    CACHE PATH "Directory containing the FplUtil library.")
set(dependencies_mathfu_dir "${fpl_root}/mathfu"
    CACHE PATH "Directory containing the MathFu library.")
set(dependencies_motive_dir "${fpl_root}/motive"
    CACHE PATH "Directory containing the motive animation library.")

# Autodesk's FBX SDK must be installed to build the pipeline.
set(fbx_sdk_dir $ENV{FBX_SDK})
if("${fbx_sdk_dir}" STREQUAL "")
  message(FATAL_ERROR "Please install the FBX SDK, and then set the FBX_SDK"
                      " environment variable to the install directory")
endif()

# Search for the FBX library.
# Visual Studio 64 bit option can be specified with
# `cmake -G "Visual Studio 11 Win64"`
if(CMAKE_CL_64)
  set(fbs_sdk_subdir "x64")
else()
  set(fbs_sdk_subdir "x86")
endif()

# Prefer the static library '.a' over the sharedlibrary '.so'.
find_library(fbx_sdk_lib_release
  NAMES libfbxsdk.a fbxsdk libfbxsdk-md.lib
  PATHS ${fbx_sdk_dir}/lib/gcc4/x64/release
        ${fbx_sdk_dir}/lib/clang/release
        ${fbx_sdk_dir}/lib/vs2008/${fbs_sdk_subdir}/release
        ${fbx_sdk_dir}/lib/vs2010/${fbs_sdk_subdir}/release
        ${fbx_sdk_dir}/lib/vs2012/${fbs_sdk_subdir}/release
        ${fbx_sdk_dir}/lib/vs2013/${fbs_sdk_subdir}/release)

find_library(fbx_sdk_lib_debug
  NAMES libfbxsdk.a fbxsdk libfbxsdk-md.lib
  PATHS ${fbx_sdk_dir}/lib/gcc4/x64/debug
        ${fbx_sdk_dir}/lib/clang/debug
        ${fbx_sdk_dir}/lib/vs2008/${fbs_sdk_subdir}/debug
        ${fbx_sdk_dir}/lib/vs2010/${fbs_sdk_subdir}/debug
        ${fbx_sdk_dir}/lib/vs2012/${fbs_sdk_subdir}/debug
        ${fbx_sdk_dir}/lib/vs2013/${fbs_sdk_subdir}/debug)

set(fbx_sdk_lib debug ${fbx_sdk_lib_debug} optimized ${fbx_sdk_lib_release})

# Linux and Mac require these libs too.
if(NOT MSVC)
  set(fbx_sdk_extra_libs pthread dl)
endif()

# Include motive.
set(motive_build_samples OFF CACHE BOOL "")
set(motive_build_tests OFF CACHE BOOL "")
add_subdirectory("${dependencies_motive_dir}" ${tmp_dir}/motive)
get_property(MOTIVE_FLATBUFFERS_GENERATED_INCLUDES_DIR
             TARGET motive_generated_includes PROPERTY MOTIVE_DIR)

# Include MathFu in this project with test and benchmark builds disabled.
set(mathfu_build_benchmarks OFF CACHE BOOL "")
set(mathfu_build_tests OFF CACHE BOOL "")
add_subdirectory(${dependencies_mathfu_dir} ${tmp_dir}/mathfu)

# We use file_util.h from fplutil.
set(fplutil_build_tests OFF)
add_subdirectory("${dependencies_fplutil_dir}/libfplutil" ${tmp_dir}/fplutil)

# Setup include directories.
include_directories(${MOTIVE_FLATBUFFERS_GENERATED_INCLUDES_DIR})
include_directories(${dependencies_flatbuffers_dir}/include)
include_directories(${dependencies_fplutil_dir}/libfplutil/include)
include_directories(${dependencies_motive_dir}/include)
include_directories(${fbx_sdk_dir}/include)

# Detect clang
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(CMAKE_COMPILER_IS_CLANGXX 1)
endif()

# Compiler flags.
set(C_FLAGS_WARNINGS "")
if(MSVC)
  set(C_FLAGS_WARNINGS "/W4 /WX")
  # Suppress deprecation of fopen.
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR
       CMAKE_COMPILER_IS_CLANGXX)
  add_definitions(-g)
  set(CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} -std=c++0x")
  # Disable warnings in fbxsdk.h that can't be disabled with pragmas.
  set(C_FLAGS_WARNINGS
      "${C_FLAGS_WARNINGS} -Wno-literal-suffix -Wno-comment")
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C_FLAGS_WARNINGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${C_FLAGS_WARNINGS}")

# Needed by the FBX SDK when we call some file manipulation functions.
if(APPLE)
  set(CMAKE_CXX_LINK_FLAGS "-framework Carbon")
endif()

# Executable target.
add_executable(anim_pipeline ${CMAKE_CURRENT_SOURCE_DIR}/anim_pipeline.cpp)

# Additional flags for the target.
mathfu_configure_flags(anim_pipeline)

# Dependencies for the executable target.
add_dependencies(anim_pipeline motive fplutil)
target_link_libraries(anim_pipeline
    ${fbx_sdk_lib}
    ${fbx_sdk_extra_libs}
    motive
    fplutil)
