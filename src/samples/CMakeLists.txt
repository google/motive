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


# Include helper functions and macros used by Google Test.
#config_compiler_and_linker()
#string(REPLACE "-W4" "-W3" cxx_default "${cxx_default}")
#string(REPLACE "-Wshadow" "" cxx_default "${cxx_default}")
#string(REPLACE "-Wextra" "" cxx_default "${cxx_default}")

# This is the directory into which the executables are built.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

include_directories(${dependencies_flatbuffers_dir}/include)

# These are support sources that are used for unit tests.
set(SUPPORT_SRCS ${dependencies_flatbuffers_dir}/src/idl_parser.cpp)

# Common libraries for tests.
if(NOT MSVC)
  find_package(Threads)
endif()

set(COMMON_LIBS "gtest;motive;${CMAKE_THREAD_LIBS_INIT}")

add_definitions(-DFPL_CURVE_GRAPH_FUNCTIONS)

# PUT ADDITIONAL SAMPLE BINARIES BELOW!
# The commands should be of the form:
#
# sample_executable(<sample-name>)
#
# Where <sample-name> is the name of the output sample executable and the
# basename of the source file for the sample. For example,
# sample_executable(MySample) generates an executable called MySample_sample
# which is the result of compiling MySample.cpp along with any files specified
# by the SUPPORT_SRCS variable.

function(sample_executable name)
  cxx_executable_with_flags(${name}_sample "${cxx_default}" "${COMMON_LIBS}"
      ${CMAKE_CURRENT_SOURCE_DIR}/${name}.cpp ${SUPPORT_SRCS}
      ${ARGN})
  mathfu_configure_flags(${name}_sample)
endfunction()

sample_executable(linear_processor)
sample_executable(own_vector_types)
sample_executable(smooth1f)

