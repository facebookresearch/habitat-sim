# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# OpenMP
set(DEPS_DIR "${CMAKE_CURRENT_LIST_DIR}/../deps")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}")

find_package(OpenMP)
find_package(OpenGL REQUIRED)

include_directories("deps")

# eigen3
include_directories(SYSTEM "${DEPS_DIR}/eigen-git-mirror")

# sophus
include_directories(SYSTEM "${DEPS_DIR}/Sophus")

# glog
add_subdirectory("${DEPS_DIR}/glog")

# tinyobjloader
include_directories(SYSTEM "${DEPS_DIR}/tinyobjloader")

# rapidjson
include_directories(SYSTEM "${DEPS_DIR}/rapidjson/include")

# Why all the weird options in the set() calls below?
# See https://stackoverflow.com/questions/3766740/overriding-a-default-option-value-in-cmake-from-a-parent-cmakelists-txt

# assimp
set(ASSIMP_BUILD_ASSIMP_TOOLS OFF CACHE BOOL "ASSIMP_BUILD_ASSIMP_TOOLS" FORCE)
set(ASSIMP_BUILD_TESTS OFF CACHE BOOL "ASSIMP_BUILD_TESTS" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "ASSIMP_BUILD_TESTS" FORCE)
add_subdirectory("${DEPS_DIR}/assimp")
# find_package(ASSIMP REQUIRED)
include_directories(SYSTEM "${DEPS_DIR}/assimp/include")
# message(STATUS "Found ASSIMP in ${ASSIMP_INCLUDE_DIR}")

# recast
set(RECASTNAVIGATION_DEMO OFF CACHE BOOL "RECASTNAVIGATION_DEMO" FORCE)
set(RECASTNAVIGATION_TESTS OFF CACHE BOOL "RECASTNAVIGATION_TESTS" FORCE)
set(RECASTNAVIGATION_EXAMPLES OFF CACHE BOOL "RECASTNAVIGATION_EXAMPLES" FORCE)
set(RECASTNAVIGATION_STATIC ON CACHE BOOL "RECASTNAVIGATION_STATIC" FORCE)
add_subdirectory("${DEPS_DIR}/recastnavigation/Recast")
add_subdirectory("${DEPS_DIR}/recastnavigation/Detour")
# Needed so that Detour doesn't hide the implementation of the method on dtQueryFilter
target_compile_definitions(Detour
  PUBLIC
  DT_VIRTUAL_QUERYFILTER)


# magnum
set(BUILD_PLUGINS_STATIC ON CACHE BOOL "BUILD_PLUGINS_STATIC" FORCE)
set(BUILD_STATIC ON CACHE BOOL "BUILD_STATIC" FORCE)
set(BUILD_STATIC_PIC ON CACHE BOOL "BUILD_STATIC_PIC" FORCE)
set(WITH_TRADE ON CACHE BOOL "WITH_TRADE" FORCE)
set(WITH_ANYSCENEIMPORTER ON CACHE BOOL "WITH_ANYSCENEIMPORTER" FORCE)
set(WITH_ASSIMPIMPORTER OFF CACHE BOOL "WITH_ASSIMPIMPORTER" FORCE)
set(WITH_TINYGLTFIMPORTER ON CACHE BOOL "WITH_TINYGLTFIMPORTER" FORCE)
set(WITH_ANYIMAGEIMPORTER ON CACHE BOOL "WITH_ANYIMAGEIMPORTER" FORCE)
set(WITH_STBIMAGEIMPORTER ON CACHE BOOL "WITH_STBIMAGEIMPORTER" FORCE)
set(WITH_STBIMAGECONVERTER ON CACHE BOOL "WITH_STBIMAGECONVERTER" FORCE)
set(WITH_SDL2APPLICATION OFF CACHE BOOL "WITH_SDL2APPLICATION" FORCE)
if(${BUILD_GUI_VIEWERS})
  add_subdirectory("${DEPS_DIR}/glfw")
  set(WITH_GLFWAPPLICATION ON CACHE BOOL "WITH_GLFWAPPLICATION" FORCE)
else()
  set(WITH_GLFWAPPLICATION OFF CACHE BOOL "WITH_GLFWAPPLICATION" FORCE)
endif()
if(APPLE)
  set(WITH_WINDOWLESSCGLAPPLICATION ON CACHE BOOL "WITH_WINDOWLESSCGLAPPLICATION" FORCE)
elseif(WIN32)
  set(WITH_WINDOWLESSWGLAPPLICATION ON CACHE BOOL "WITH_WINDOWLESSWGLAPPLICATION" FORCE)
elseif(UNIX)
  if(${BUILD_GUI_VIEWERS})
    set(WITH_WINDOWLESSGLXAPPLICATION ON  CACHE INTERNAL "WITH_WINDOWLESSGLXAPPLICATION" FORCE)
    set(WITH_WINDOWLESSEGLAPPLICATION OFF CACHE INTERNAL "WITH_WINDOWLESSEGLAPPLICATION" FORCE)
  else()
    set(WITH_WINDOWLESSGLXAPPLICATION OFF CACHE INTERNAL "WITH_WINDOWLESSGLXAPPLICATION" FORCE)
    set(WITH_WINDOWLESSEGLAPPLICATION ON  CACHE INTERNAL "WITH_WINDOWLESSEGLAPPLICATION" FORCE)
  endif()
endif()
add_subdirectory("${DEPS_DIR}/corrade")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${DEPS_DIR}/magnum/modules/")
add_subdirectory("${DEPS_DIR}/magnum")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${DEPS_DIR}/magnum-plugins/modules/")
add_subdirectory("${DEPS_DIR}/magnum-plugins")

# python interpreter
find_package(PythonInterp 3.6 REQUIRED)

# Search for python executable to pick up activated virtualenv/conda python
unset(PYTHON_EXECUTABLE CACHE)
find_program(PYTHON_EXECUTABLE
  python
    PATHS ENV PATH   # look in the PATH environment variable
    NO_DEFAULT_PATH  # do not look anywhere else...
)
message(STATUS "Bindings being generated for python at ${PYTHON_EXECUTABLE}")

# pybind11
add_subdirectory("${DEPS_DIR}/pybind11")
# find_package(pybind11 REQUIRED)

# gtest build
if(BUILD_TEST)
  # store build shared libs option
  set(TEMP_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})

  # build gtest static libs and embed into test binaries so no need to install
  set(BUILD_SHARED_LIBS OFF CACHE BOOL "BUILD_SHARED_LIBS" FORCE)
  set(BUILD_GTEST ON CACHE BOOL "BUILD_GTEST" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "INSTALL_GTEST" FORCE)
  set(BUILD_GMOCK OFF CACHE BOOL "BUILD_GMOCK" FORCE)
  add_subdirectory("${DEPS_DIR}/googletest")
  include_directories(SYSTEM "${DEPS_DIR}/googletest/googletest/include")

  # restore build shared libs option
  set(BUILD_SHARED_LIBS ${TEMP_BUILD_SHARED_LIBS})
endif()
