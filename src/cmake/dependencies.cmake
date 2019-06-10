# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# OpenMP
set(DEPS_DIR "${CMAKE_CURRENT_LIST_DIR}/../deps")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}")

find_package(OpenMP)
# We don't find_package(OpenGL REQUIRED) here, but let Magnum do that instead
# as it sets up various things related to GLVND.

include_directories("deps")

# Eigen. Use a system package, if preferred.
if(USE_SYSTEM_EIGEN)
  find_package(Eigen3 REQUIRED)
  include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
else()
  include_directories(SYSTEM "${DEPS_DIR}/eigen-git-mirror")
  set(EIGEN3_INCLUDE_DIR "${DEPS_DIR}/eigen-git-mirror")
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${DEPS_DIR}/eigen-git-mirror/cmake")
endif()

# sophus
include_directories(SYSTEM "${DEPS_DIR}/Sophus")

# glog
add_subdirectory("${DEPS_DIR}/glog")

# tinyobjloader
include_directories(SYSTEM "${DEPS_DIR}/tinyobjloader")

# RapidJSON. Use a system package, if preferred.
if(USE_SYSTEM_RAPIDJSON)
  find_package(RapidJSON CONFIG REQUIRED)
  include_directories(SYSTEM ${RapidJSON_INCLUDE_DIR})
else()
  include_directories(SYSTEM "${DEPS_DIR}/rapidjson/include")
endif()

# Why all the weird options in the set() calls below?
# See https://stackoverflow.com/questions/3766740/overriding-a-default-option-value-in-cmake-from-a-parent-cmakelists-txt

# Assimp. Use a system package, if preferred.
if(NOT USE_SYSTEM_ASSIMP)
  set(ASSIMP_BUILD_ASSIMP_TOOLS OFF CACHE BOOL "ASSIMP_BUILD_ASSIMP_TOOLS" FORCE)
  set(ASSIMP_BUILD_TESTS OFF CACHE BOOL "ASSIMP_BUILD_TESTS" FORCE)
  set(BUILD_SHARED_LIBS OFF CACHE BOOL "ASSIMP_BUILD_TESTS" FORCE)
  add_subdirectory("${DEPS_DIR}/assimp")

  # Help FindAssimp locate everything
  set(ASSIMP_INCLUDE_DIR "${DEPS_DIR}/assimp/include" CACHE STRING "" FORCE)
  set(ASSIMP_LIBRARY_DEBUG assimp CACHE STRING "" FORCE)
  set(ASSIMP_LIBRARY_RELEASE assimp CACHE STRING "" FORCE)
  add_library(Assimp::Assimp ALIAS assimp)
endif()
find_package(Assimp REQUIRED)

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

# Pybind11. Use a system package, if preferred. This needs to be before Magnum
# so the bindings can properly detect pybind11 added as a subproject.
if(USE_SYSTEM_PYBIND11)
  find_package(pybind11 REQUIRED)
else()
  add_subdirectory("${DEPS_DIR}/pybind11")
endif()

# Magnum. Use a system package, if preferred.
if(NOT USE_SYSTEM_MAGNUM)
  set(BUILD_PLUGINS_STATIC ON CACHE BOOL "BUILD_PLUGINS_STATIC" FORCE)
  set(BUILD_STATIC ON CACHE BOOL "BUILD_STATIC" FORCE)
  set(BUILD_STATIC_PIC ON CACHE BOOL "BUILD_STATIC_PIC" FORCE)

  # These are enabled by default but we don't need them right now -- disabling
  # for slightly faster builds. If you need any of these, simply delete a line.
  set(WITH_INTERCONNECT OFF CACHE BOOL "" FORCE)
  set(WITH_TESTSUITE OFF CACHE BOOL "" FORCE)
  set(WITH_DEBUGTOOLS OFF CACHE BOOL "" FORCE)
  set(WITH_PRIMITIVES OFF CACHE BOOL "" FORCE)
  set(WITH_TEXT OFF CACHE BOOL "" FORCE)
  set(WITH_TEXTURETOOLS OFF CACHE BOOL "" FORCE)

  # These are not enabled by default but we need them
  set(WITH_ANYSCENEIMPORTER ON CACHE BOOL "WITH_ANYSCENEIMPORTER" FORCE)
  set(WITH_ASSIMPIMPORTER ON CACHE BOOL "WITH_ASSIMPIMPORTER" FORCE)
  set(WITH_TINYGLTFIMPORTER ON CACHE BOOL "WITH_TINYGLTFIMPORTER" FORCE)
  set(WITH_ANYIMAGEIMPORTER ON CACHE BOOL "WITH_ANYIMAGEIMPORTER" FORCE)
  set(WITH_STBIMAGEIMPORTER ON CACHE BOOL "WITH_STBIMAGEIMPORTER" FORCE)
  set(WITH_STBIMAGECONVERTER ON CACHE BOOL "WITH_STBIMAGECONVERTER" FORCE)
  set(WITH_SDL2APPLICATION OFF CACHE BOOL "WITH_SDL2APPLICATION" FORCE)
  set(WITH_EIGEN ON CACHE BOOL "WITH_EIGEN" FORCE) # Eigen integration
  set(WITH_PYTHON ON CACHE BOOL "" FORCE) # Python bindings

  if(BUILD_GUI_VIEWERS)
    if(NOT USE_SYSTEM_GLFW)
      add_subdirectory("${DEPS_DIR}/glfw")
    endif()
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
  add_subdirectory("${DEPS_DIR}/magnum")
  add_subdirectory("${DEPS_DIR}/magnum-plugins")
  add_subdirectory("${DEPS_DIR}/magnum-integration")
  add_subdirectory("${DEPS_DIR}/magnum-bindings")
endif()

# tinyply
include_directories(SYSTEM "${DEPS_DIR}/tinyply/source")
add_subdirectory("${DEPS_DIR}/tinyply")

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
