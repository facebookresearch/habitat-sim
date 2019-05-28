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

# build options
option(BUILD_PYTHON_BINDINGS "Whether to build python bindings" ON)
option(BUILD_TEST "Build test binaries" OFF)
option(BUILD_WEBGL "Build for WebGL through emscripten" OFF)
option(BUILD_GUI_VIEWERS "Whether to build GUI viewer utility binary" OFF)
option(BUILD_DATATOOL "Whether to build datatool utility binary" OFF)
option(BUILD_PTEX_SUPPORT "Whether to build ptex mesh support" OFF)
option(USE_SYSTEM_ASSIMP "Use system Assimp instead of a bundled submodule" OFF)
option(USE_SYSTEM_EIGEN "Use system Eigen instead of a bundled submodule" OFF)
option(USE_SYSTEM_MAGNUM "Use system Magnum instead of a bundled submodule" OFF)
option(USE_SYSTEM_PYBIND11 "Use system Pybind11 instead of a bundled submodule" OFF)
option(USE_SYSTEM_RAPIDJSON "Use system RapidJSON instead of a bundled submodule" OFF)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose build, options are: None Debug Release RelWithDebInfo MinSizeRel"
      FORCE)
endif()
# avoid ld visibility warnings, should be done by CMAKE_CXX_VISIBILITY_PRESET
# but need cmake_policy(SET CMP0063 NEW) also which seems to not work
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")

# ---[ Dependencies
include(cmake/dependencies.cmake)

# include source dirs
include_directories(${PROJECT_SOURCE_DIR})

# compiler define for enabling ptex support
if(BUILD_PTEX_SUPPORT)
  add_compile_definitions(BUILD_PTEX_SUPPORT)
endif()

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

if(BUILD_DATATOOL)
  add_subdirectory(utils/datatool)
endif()

if(BUILD_GUI_VIEWERS)
  add_subdirectory(utils/viewer)
endif()

# build tests
if(BUILD_TEST)
  message("Building TESTS")
  add_subdirectory(tests)
endif()

# pybind bindings
if(BUILD_PYTHON_BINDINGS)
  message("Building Python bindings")
  add_subdirectory(esp/bindings)
endif()
