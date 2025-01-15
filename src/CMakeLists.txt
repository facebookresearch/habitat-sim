# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

cmake_minimum_required(VERSION 3.12)

option(BUILD_WITH_CUDA "Build Habitat-Sim with CUDA features enabled -- Requires CUDA"
       OFF
)

if(BUILD_WITH_CUDA)
  project(esp LANGUAGES C CXX CUDA)
  find_library(CUDART_LIBRARY cudart ${CMAKE_CUDA_IMPLICIT_LINK_DIRECTORIES})
else()
  project(esp LANGUAGES C CXX)
endif()

if(MSVC)
  add_definitions(/DNOMINMAX)
endif()

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif()

set(DATA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../data)
set(SCENE_DATASETS ${CMAKE_CURRENT_SOURCE_DIR}/../data/scene_datasets)
set(TEST_ASSETS ${CMAKE_CURRENT_SOURCE_DIR}/../data/test_assets)
# build options
option(BUILD_ASSIMP_SUPPORT "Whether to build assimp import library support" ON)
option(BUILD_PYTHON_BINDINGS "Whether to build python bindings" ON)
option(BUILD_GUI_VIEWERS "Whether to build GUI viewer utility binary" OFF)
option(BUILD_WITH_BULLET
       "Build Habitat-Sim with Bullet physics enabled -- Requires Bullet" OFF
)
option(BUILD_WITH_BACKGROUND_RENDERER "Build Habitat-Sim with async rendering support."
       ON
)
option(BUILD_TEST "Build test binaries" OFF)
option(REL_BUILD_RPATH "Use a relative build rpath" OFF)
option(USE_SYSTEM_ASSIMP "Use system Assimp instead of a bundled submodule" OFF)
option(USE_SYSTEM_OPENEXR "Use system OpenEXR instead of a bundled submodule" OFF)
option(USE_SYSTEM_EIGEN "Use system Eigen instead of a bundled submodule" OFF)
option(USE_SYSTEM_GLFW "Use system GLFW instead of a bundled submodule" OFF)
option(USE_SYSTEM_MAGNUM "Use system Magnum instead of a bundled submodule" OFF)
option(USE_SYSTEM_PYBIND11 "Use system Pybind11 instead of a bundled submodule" OFF)
option(USE_SYSTEM_RAPIDJSON "Use system RapidJSON instead of a bundled submodule" OFF)
option(USE_SYSTEM_BULLET "Use system Bullet instead of a bundled submodule" OFF)
# Zstd is used only by the BasisImporter plugin, if external Magnum is used it
# doesn't need to be built at all.
include(CMakeDependentOption)
cmake_dependent_option(
  USE_SYSTEM_ZSTD "Use system zstd instead of a bundled submodule" OFF
  "NOT USE_SYSTEM_MAGNUM" ON
)
option(
  BUILD_BASIS_COMPRESSOR
  "Wether or not to build the basis compressor.  Loading basis compressed meshes does NOT require this."
  OFF
)
option(BUILD_WITH_AUDIO "Build Habitat-Sim with Audio sensor" OFF)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(
    CMAKE_BUILD_TYPE
    RelWithDebInfo
    CACHE STRING
          "Choose build, options are: None Debug Release RelWithDebInfo MinSizeRel"
          FORCE
  )
endif()
if(CMAKE_INTERPROCEDURAL_OPTIMIZATION)
  # Policy that enables INTERPROCEDURAL_OPTIMIZATION
  set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
  cmake_policy(SET CMP0069 NEW)

  if(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang"
     OR (CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND CMAKE_CXX_COMPILER_VERSION
                                                   VERSION_LESS 13.0)
  )
    message("-- Forcing full LTO for compatibility: ThinLTO requires LLVM Clang>=13.0")
    # Clang has some issues with thinLTO (segfaults on final linking)
    # This enables full LTO through CMake
    # FullLTO is slow, but may yield performance improvements.
    #set(CMAKE_{lang}_COMPILE_OPTIONS_IPO ${CMAKE_CXX_COMPILE_OPTIONS_IPO} -flto)
    add_compile_options("-flto=full") # alternate way of forcing FAT LTO.
  endif()
endif()
# avoid ld visibility warnings, should be done by CMAKE_CXX_VISIBILITY_PRESET
# but need cmake_policy(SET CMP0063 NEW) also which seems to not work
# TODO Figure out how to remove the visibility flags from Bullet
#set(CMAKE_POLICY_DEFAULT_CMP0063 NEW)
#cmake_policy(SET CMP0063 NEW)
#set(CMAKE_CXX_VISIBILITY_PRESET hidden)
#set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")

# Properly set the rpath in a cross-platform way
if(APPLE)
  set(CMAKE_MACOSX_RPATH ON)
  set(_rpath_portable_origin "@loader_path")
else()
  set(_rpath_portable_origin $ORIGIN)
endif(APPLE)
set(CMAKE_SKIP_BUILD_RPATH FALSE)
if(REL_BUILD_RPATH)
  message("Using relative rpath")
  set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
  #_magnum.so needs to look at habitat_sim/_ext for its corrade dependencies
  set(CMAKE_INSTALL_RPATH "${_rpath_portable_origin}"
                          "${_rpath_portable_origin}/habitat_sim/_ext/"
  )
else()
  message("Using absolute rpath")
  # Set to defaults
  set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
  set(CMAKE_INSTALL_RPATH "")
endif()

# ---[ Dependencies
include(cmake/dependencies.cmake)

# build tests
if(BUILD_TEST)
  message("Building TESTS")
  enable_testing()
  find_package(Corrade REQUIRED TestSuite)
  find_package(Magnum REQUIRED OpenGLTester)
endif()

# include source dirs
include_directories(${PROJECT_SOURCE_DIR})

# Physics
add_definitions(-DBT_ENABLE_PROFILE)

# add subdirectories
add_subdirectory(esp)

if(BUILD_PYTHON_BINDINGS)
  message("Building Python bindings")
  add_subdirectory(esp/bindings)
endif()

# Build the following with the build RPath
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

if(BUILD_BASIS_COMPRESSOR)
  add_subdirectory(utils/imageconverter)
endif()

if(BUILD_TEST)
  add_subdirectory(tests)
endif()

# pybind bindings
if(BUILD_GUI_VIEWERS)
  message("Building GUI viewer")
  add_subdirectory(utils/viewer)
  add_subdirectory(utils/replayer)
endif()
