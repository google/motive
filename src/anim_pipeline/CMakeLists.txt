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
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/${CMAKE_SYSTEM_NAME})
set(tmp_dir ${CMAKE_BINARY_DIR}/obj)

# Include functions fbx_compile_options() and fbx_configure_target()
include("../../cmake/find_fplutil.cmake")
include("${fplutil_dir}/fbx_common/cmake_fbx.txt")

# Set compile options for FBX programs.
fbx_compile_options()

# Include motive.
set(motive_build_samples OFF CACHE BOOL "")
set(motive_build_tests OFF CACHE BOOL "")
set(motive_build_viewer OFF CACHE BOOL "")
add_subdirectory("${dependencies_motive_dir}" ${tmp_dir}/motive)
get_property(MOTIVE_FLATBUFFERS_GENERATED_INCLUDES_DIR
             TARGET motive_generated_includes PROPERTY GENERATED_INCLUDES_DIR)

# Include MathFu in this project with test and benchmark builds disabled.
set(mathfu_build_benchmarks OFF CACHE BOOL "")
set(mathfu_build_tests OFF CACHE BOOL "")
add_subdirectory(${dependencies_mathfu_dir} ${tmp_dir}/mathfu)

# We use file_util.h from fplutil.
set(fplutil_build_tests OFF CACHE BOOL "")
add_subdirectory("${dependencies_fplutil_dir}/libfplutil" ${tmp_dir}/fplutil)

# Add the common FBX library.
add_subdirectory(${dependencies_fplutil_dir}/fbx_common ${tmp_dir}/fbx_common)

# Setup include directories.
include_directories(${MOTIVE_FLATBUFFERS_GENERATED_INCLUDES_DIR})
include_directories(${dependencies_flatbuffers_dir}/include)
include_directories(${dependencies_fplutil_dir}/libfplutil/include)
include_directories(${dependencies_motive_dir}/include)

# Executable target.
add_executable(anim_pipeline ${CMAKE_CURRENT_SOURCE_DIR}/anim_pipeline.cpp
                             ${CMAKE_CURRENT_SOURCE_DIR}/anim_pipeline_main.cpp)

# Set further options for FBX programs.
fbx_configure_target(anim_pipeline)
target_link_libraries(anim_pipeline fplutil motive)

# Additional flags for the target.
mathfu_configure_flags(anim_pipeline)
