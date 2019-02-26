# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

cmake_minimum_required(VERSION 3.10)

project(esp)

if(MSVC)
  add_definitions(/DNOMINMAX)
endif()

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

set(BUILD_GUI_VIEWERS OFF CACHE BOOL "BUILD_GUI_VIEWERS")

# set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose build, options are: None Debug Release RelWithDebInfo MinSizeRel"
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)
# avoid ld visibility warnings, should be done by CMAKE_CXX_VISIBILITY_PRESET
# but need cmake_policy(SET CMP0063 NEW) also which seems to not work
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")

# build options
option(BUILD_TEST "Build test binaries" OFF)

# ---[ Dependencies
include(cmake/dependencies.cmake)

# include source dirs
include_directories(${PROJECT_SOURCE_DIR})

# add subdirectories
add_subdirectory(esp/core)
add_subdirectory(esp/geo)
add_subdirectory(esp/gfx/)
add_subdirectory(esp/assets)
add_subdirectory(esp/io)
add_subdirectory(esp/scene)
add_subdirectory(esp/nav)
add_subdirectory(esp/agent)
add_subdirectory(esp/sensor)
add_subdirectory(utils/datatool)
if(${BUILD_GUI_VIEWERS})
  # add_subdirectory(utils/displayObj/)
  add_subdirectory(utils/viewer)
endif()

# build main library
#file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/media DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
# add_library(${CMAKE_PROJECT_NAME}
#             $<TARGET_OBJECTS:assets>
#             $<TARGET_OBJECTS:Camera>
#             $<TARGET_OBJECTS:ShaderProgram>)
# target_link_libraries(${CMAKE_PROJECT_NAME})

# build tests
if (BUILD_TEST)
  message("Building TESTS")
  add_subdirectory(tests)
endif()

# pybind bindings
add_subdirectory(esp/bindings)
