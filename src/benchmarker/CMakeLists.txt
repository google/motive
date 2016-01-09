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

project(benchmarker)

# This is the motive directory, which is a parent of the current directory.
get_filename_component(motive_dir "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

# This is the directory into which the executables are built.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${motive_dir}/bin)

set(COMMON_LIBS "motive;${CMAKE_THREAD_LIBS_INIT}")
set(tmp_dir ${CMAKE_BINARY_DIR}/obj)

# If the dependencies directory exists, assume this is the root directory for
# all libraries required by this project.
get_filename_component(fpl_root "${motive_dir}/.." ABSOLUTE)
if(NOT DEFINED dependencies_root)
  set(dependencies_root "${motive_dir}/dependencies"
      CACHE PATH "Directory holding the dependencies, when pulled from github.")
  if(EXISTS "${dependencies_root}")
    set(fpl_root "${dependencies_root}")
  endif()
endif()

set(dependencies_motive_dir "${motive_dir}"
    CACHE PATH "Directory containing the motive animation library.")
set(dependencies_mathfu_dir "${fpl_root}/mathfu"
    CACHE PATH "Directory containing the MathFu library.")

# Include MathFu in this project with test and benchmark builds disabled.
set(mathfu_build_benchmarks OFF CACHE BOOL "")
set(mathfu_build_tests OFF CACHE BOOL "")
add_subdirectory(${dependencies_mathfu_dir} ${tmp_dir}/mathfu)

# Include motive.
set(motive_build_samples OFF CACHE BOOL "")
set(motive_build_tests OFF CACHE BOOL "")
set(motive_build_benchmarker OFF CACHE BOOL "")
set(motive_enable_benchmarks ON CACHE BOOL "")
add_subdirectory("${dependencies_motive_dir}" ${tmp_dir}/motive)

include_directories(${MOTIVE_FLATBUFFERS_GENERATED_INCLUDES_DIR})
include_directories(${dependencies_motive_dir}/include)
include_directories(${dependencies_fplutil_dir}/libfplutil/include)

# Detect clang
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(CMAKE_COMPILER_IS_CLANGXX 1)
endif()

# Compiler flags.
set(C_FLAGS_WARNINGS "")
if(MSVC)
  # C4127: conditional expression is constant
  # C4577: 'noexcept' used with no exception handling mode specified.
  set(C_FLAGS_WARNINGS "/W4 /WX /wd4127 /wd4577")
  set(CMAKE_CXX_FLAGS "/MP")
  add_definitions(-DNOMINMAX -D_HAS_EXCEPTIONS=0 -D_CRT_SECURE_NO_WARNINGS)
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR
       CMAKE_COMPILER_IS_CLANGXX)
  add_definitions(-g)
  set(CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -pedantic -Werror -Wextra")
endif()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C_FLAGS_WARNINGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${C_FLAGS_WARNINGS}")
set(CMAKE_BUILD_TYPE Release)
add_definitions(-DBENCHMARK_MOTIVE)

# Executable target.
add_executable(benchmarker ${CMAKE_CURRENT_SOURCE_DIR}/benchmarker.cpp)

# Additional flags for the target.
mathfu_configure_flags(benchmarker)

# Dependencies for the executable target.
add_dependencies(benchmarker motive)
target_link_libraries(benchmarker motive)
