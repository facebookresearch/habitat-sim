# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

set(DEPS_DIR "${CMAKE_CURRENT_LIST_DIR}/../deps")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}")

# Find Corrade first so we can use CORRADE_TARGET_*
if(NOT USE_SYSTEM_MAGNUM)
  # These are enabled by default but we don't need them right now -- disabling
  # for slightly faster builds. If you need any of these, simply delete a line.
  set(WITH_INTERCONNECT OFF CACHE BOOL "" FORCE)
  add_subdirectory("${DEPS_DIR}/corrade")
endif()
find_package(Corrade REQUIRED Utility)

# OpenMP
find_package(OpenMP)
# We don't find_package(OpenGL REQUIRED) here, but let Magnum do that instead
# as it sets up various things related to GLVND.

include_directories("deps")

# Eigen. Use a system package, if preferred.
if(USE_SYSTEM_EIGEN)
  find_package(Eigen3 REQUIRED)
  include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
else()
  include_directories(SYSTEM "${DEPS_DIR}/eigen")
  set(EIGEN3_INCLUDE_DIR "${DEPS_DIR}/eigen")
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${DEPS_DIR}/eigen/cmake")
endif()

if(NOT IMGUI_DIR)
  set(IMGUI_DIR "${DEPS_DIR}/imgui")
endif()

# sophus
include_directories(SYSTEM "${DEPS_DIR}/Sophus")

# tinyxml2
include_directories("${DEPS_DIR}/tinyxml2")
add_subdirectory("${DEPS_DIR}/tinyxml2")

# glog. NOTE: emscripten does not support 32-bit targets, which glog requires.
# Therefore we do not build glog and use a custom shim instead to emulate glog
if(CORRADE_TARGET_EMSCRIPTEN)
  add_library(glog INTERFACE)
else()
  # We don't want glog tests to be run as part of our build. The BUILD_TESTING
  # variable is set by include(CTest) as an option(). In CMake 3.13+ it should
  # be enough to do just set(BUILD_TESTING OFF) to avoid the option()
  # overriding it, but https://cmake.org/cmake/help/latest/policy/CMP0077.html.
  option(BUILD_TESTING "" OFF)
  # glog has gflags as an optional dependency, which we don't supply. In some
  # cases it'll result in system libs being picked up, leading to errors like
  # described in https://github.com/facebookresearch/habitat-sim/issues/205 or
  # https://github.com/facebookresearch/habitat-sim/issues/179. We don't need
  # anything from gflags, so just disable them completely.
  set(WITH_GFLAGS OFF CACHE BOOL "" FORCE)
  add_subdirectory("${DEPS_DIR}/glog")
endif()

# RapidJSON. Use a system package, if preferred.
if(USE_SYSTEM_RAPIDJSON)
  find_package(RapidJSON CONFIG REQUIRED)
  include_directories(SYSTEM ${RapidJSON_INCLUDE_DIR})
else()
  include_directories(SYSTEM "${DEPS_DIR}/rapidjson/include")
endif()

# Why all the weird options in the set() calls below?
# See https://stackoverflow.com/questions/3766740/overriding-a-default-option-value-in-cmake-from-a-parent-cmakelists-txt

if(BUILD_ASSIMP_SUPPORT)
  # Assimp. Use a system package, if preferred.
  if(NOT USE_SYSTEM_ASSIMP)
    set(ASSIMP_BUILD_ASSIMP_TOOLS OFF CACHE BOOL "ASSIMP_BUILD_ASSIMP_TOOLS" FORCE)
    set(ASSIMP_BUILD_TESTS OFF CACHE BOOL "ASSIMP_BUILD_TESTS" FORCE)
    set(BUILD_SHARED_LIBS OFF CACHE BOOL "ASSIMP_BUILD_TESTS" FORCE)
    # The following is important to avoid Assimp appending `d` to all our
    # binaries. Works only with Assimp >= 5.0.0, and after 5.0.1 this option is
    # prefixed with ASSIMP_, so better set both variants to future-proof this.
    set(INJECT_DEBUG_POSTFIX OFF CACHE BOOL "" FORCE)
    set(ASSIMP_INJECT_DEBUG_POSTFIX OFF CACHE BOOL "" FORCE)
    add_subdirectory("${DEPS_DIR}/assimp")

    # Help FindAssimp locate everything
    set(ASSIMP_INCLUDE_DIR "${DEPS_DIR}/assimp/include" CACHE STRING "" FORCE)
    set(ASSIMP_LIBRARY_DEBUG assimp CACHE STRING "" FORCE)
    set(ASSIMP_LIBRARY_RELEASE assimp CACHE STRING "" FORCE)
    add_library(Assimp::Assimp ALIAS assimp)
  endif()
  find_package(Assimp REQUIRED)
endif()

# v-hacd
if(BUILD_WITH_VHACD)
  set(NO_OPENCL ON CACHE BOOL "NO_OPENCL" FORCE)
  set(NO_OPENMP ON CACHE BOOL "NO_OPENMP" FORCE)
  # adding /src/VHACD_Lib instead of /src since /src contains unneccesary test files
  add_subdirectory("${DEPS_DIR}/v-hacd/src/VHACD_Lib")
endif()

# recast
set(RECASTNAVIGATION_DEMO OFF CACHE BOOL "RECASTNAVIGATION_DEMO" FORCE)
set(RECASTNAVIGATION_TESTS OFF CACHE BOOL "RECASTNAVIGATION_TESTS" FORCE)
set(RECASTNAVIGATION_EXAMPLES OFF CACHE BOOL "RECASTNAVIGATION_EXAMPLES" FORCE)
set(RECASTNAVIGATION_STATIC ON CACHE BOOL "RECASTNAVIGATION_STATIC" FORCE)
add_subdirectory("${DEPS_DIR}/recastnavigation/Recast")
add_subdirectory("${DEPS_DIR}/recastnavigation/Detour")
# Needed so that Detour doesn't hide the implementation of the method on dtQueryFilter
target_compile_definitions(Detour PUBLIC DT_VIRTUAL_QUERYFILTER)

if(BUILD_PYTHON_BINDINGS)
  # Before calling find_package(PythonInterp) search for python executable not
  # in the default paths to pick up activated virtualenv/conda python
  find_program(
    PYTHON_EXECUTABLE
    # macOS still defaults to `python` being Python 2, so look for `python3`
    # first
    NAMES python3 python
    PATHS ENV PATH # look in the PATH environment variable
    NO_DEFAULT_PATH # do not look anywhere else...
  )

  # Let the Find module do proper version checks on what we found (it uses the
  # same PYTHON_EXECUTABLE variable, will pick it up from the cache)
  find_package(PythonInterp 3.6 REQUIRED)

  message(STATUS "Bindings being generated for python at ${PYTHON_EXECUTABLE}")

  # Pybind11. Use a system package, if preferred. This needs to be before Magnum
  # so the bindings can properly detect pybind11 added as a subproject.
  if(USE_SYSTEM_PYBIND11)
    find_package(pybind11 REQUIRED)
  else()
    add_subdirectory("${DEPS_DIR}/pybind11")
  endif()
endif()

if(BUILD_WITH_BULLET AND NOT USE_SYSTEM_BULLET)
  # The below block except for the visiblity patch verbatim copied from
  # https://doc.magnum.graphics/magnum/namespaceMagnum_1_1BulletIntegration.html

  # Disable Bullet tests and demos
  set(BUILD_UNIT_TESTS OFF CACHE BOOL "" FORCE)
  set(BUILD_BULLET2_DEMOS OFF CACHE BOOL "" FORCE)
  set(BUILD_CPU_DEMOS OFF CACHE BOOL "" FORCE)
  set(BUILD_OPENGL3_DEMOS OFF CACHE BOOL "" FORCE)
  # While not needed for Magnum, you might actually want some of those
  set(BUILD_ENET OFF CACHE BOOL "" FORCE)
  set(BUILD_CLSOCKET OFF CACHE BOOL "" FORCE)
  set(BUILD_EXTRAS OFF CACHE BOOL "" FORCE)
  set(BUILD_BULLET3 OFF CACHE BOOL "" FORCE)
  # This is needed in case BUILD_EXTRAS is enabled, as you'd get a CMake syntax
  # error otherwise
  set(PKGCONFIG_INSTALL_PREFIX "lib${LIB_SUFFIX}/pkgconfig/")

  # caches CXX_FLAGS so we can reset them at the end
  set(_PREV_CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})

  # Bullet's buildsystem doesn't correctly express dependencies between static
  # libs, causing linker errors on Magnum side. If you have CMake 3.13, the
  # Find module is able to correct that on its own, otherwise you need to
  # enable BUILD_SHARED_LIBS to build as shared.
  if((NOT CORRADE_TARGET_EMSCRIPTEN) AND CMAKE_VERSION VERSION_LESS 3.13)
    set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE)
    # however the whole Habitat is built with -fvisibility=hidden and Bullet
    # doesn't export any of its symbols and relies on symbols being visible by
    # default. Which means we have to compile it without hidden visibility.
    # ... and because we have to build shared libs, we need exported symbols,
    string(REPLACE "-fvisibility=hidden" "" CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
  else()
    # On Emscripten we require 3.13, so there it's fine (and there we can't use
    # shared libs)
    set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
  endif()
  add_subdirectory(${DEPS_DIR}/bullet3 EXCLUDE_FROM_ALL)
  set(CMAKE_CXX_FLAGS ${_PREV_CMAKE_CXX_FLAGS})
endif()

# Magnum. Use a system package, if preferred.
if(NOT USE_SYSTEM_MAGNUM)
  set(BUILD_PLUGINS_STATIC ON CACHE BOOL "BUILD_PLUGINS_STATIC" FORCE)
  set(BUILD_STATIC ON CACHE BOOL "BUILD_STATIC" FORCE)
  set(BUILD_STATIC_PIC ON CACHE BOOL "BUILD_STATIC_PIC" FORCE)

  # These are enabled by default but we don't need them right now -- disabling
  # for slightly faster builds. If you need any of these, simply delete a line.
  set(WITH_TEXT OFF CACHE BOOL "" FORCE)
  set(WITH_TEXTURETOOLS OFF CACHE BOOL "" FORCE)

  # These are not enabled by default but we need them
  set(WITH_ANYSCENEIMPORTER ON CACHE BOOL "WITH_ANYSCENEIMPORTER" FORCE)
  if(BUILD_ASSIMP_SUPPORT)
    set(WITH_ASSIMPIMPORTER ON CACHE BOOL "WITH_ASSIMPIMPORTER" FORCE)
  endif()
  set(WITH_TINYGLTFIMPORTER ON CACHE BOOL "WITH_TINYGLTFIMPORTER" FORCE)
  set(WITH_ANYIMAGEIMPORTER ON CACHE BOOL "WITH_ANYIMAGEIMPORTER" FORCE)
  set(WITH_ANYIMAGECONVERTER ON CACHE BOOL "WITH_ANYIMAGECONVERTER" FORCE)
  set(WITH_PRIMITIVEIMPORTER ON CACHE BOOL "" FORCE)
  set(WITH_STANFORDIMPORTER ON CACHE BOOL "" FORCE)
  set(WITH_STBIMAGEIMPORTER ON CACHE BOOL "WITH_STBIMAGEIMPORTER" FORCE)
  set(WITH_STBIMAGECONVERTER ON CACHE BOOL "WITH_STBIMAGECONVERTER" FORCE)
  set(WITH_EMSCRIPTENAPPLICATION OFF CACHE BOOL "WITH_EMSCRIPTENAPPLICATION" FORCE)
  set(WITH_GLFWAPPLICATION OFF CACHE BOOL "WITH_GLFWAPPLICATION" FORCE)
  set(WITH_EIGEN ON CACHE BOOL "WITH_EIGEN" FORCE) # Eigen integration
  set(WITH_IMGUI ON CACHE BOOL "WITH_IMGUI" FORCE) # ImGui integration
  if(BUILD_PYTHON_BINDINGS)
    set(WITH_PYTHON ON CACHE BOOL "" FORCE) # Python bindings
  endif()
  # We only support WebGL2
  if(CORRADE_TARGET_EMSCRIPTEN)
    set(TARGET_GLES2 OFF CACHE BOOL "" FORCE)
  endif()
  if(BUILD_TEST)
    set(WITH_OPENGLTESTER ON CACHE BOOL "" FORCE)
  endif()

  # Basis Universal. The repo is extremely huge and so instead of a Git
  # submodule we bundle just the transcoder files, and only a subset of the
  # formats (BC7 mode 6 has > 1 MB tables, ATC/FXT1/PVRTC2 are quite rare and
  # not supported by Magnum).
  set(BASIS_UNIVERSAL_DIR "${DEPS_DIR}/basis-universal")
  set(
    CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -DBASISD_SUPPORT_BC7_MODE6_OPAQUE_ONLY=0 -DBASISD_SUPPORT_ATC=0 -DBASISD_SUPPORT_FXT1=0 -DBASISD_SUPPORT_PVRTC2=0"
  )
  set(WITH_BASISIMPORTER ON CACHE BOOL "" FORCE)

  if(BUILD_WITH_BULLET)
    # Build Magnum's BulletIntegration
    set(WITH_BULLET ON CACHE BOOL "" FORCE)
  endif()

  if(BUILD_GUI_VIEWERS)
    if(CORRADE_TARGET_EMSCRIPTEN)
      set(WITH_EMSCRIPTENAPPLICATION ON CACHE BOOL "WITH_EMSCRIPTENAPPLICATION" FORCE)
    else()
      if(NOT USE_SYSTEM_GLFW)
        add_subdirectory("${DEPS_DIR}/glfw")
      endif()
      set(WITH_GLFWAPPLICATION ON CACHE BOOL "WITH_GLFWAPPLICATION" FORCE)
    endif()
  endif()
  if(APPLE)
    set(WITH_WINDOWLESSCGLAPPLICATION ON CACHE BOOL "WITH_WINDOWLESSCGLAPPLICATION"
                                               FORCE
    )
  elseif(WIN32)
    set(WITH_WINDOWLESSWGLAPPLICATION ON CACHE BOOL "WITH_WINDOWLESSWGLAPPLICATION"
                                               FORCE
    )
  elseif(CORRADE_TARGET_EMSCRIPTEN)
    set(WITH_WINDOWLESSEGLAPPLICATION ON CACHE INTERNAL "WITH_WINDOWLESSEGLAPPLICATION"
                                               FORCE
    )
  elseif(UNIX)
    if(BUILD_GUI_VIEWERS)
      set(WITH_WINDOWLESSGLXAPPLICATION ON CACHE INTERNAL
                                                 "WITH_WINDOWLESSGLXAPPLICATION" FORCE
      )
      set(WITH_WINDOWLESSEGLAPPLICATION OFF CACHE INTERNAL
                                                  "WITH_WINDOWLESSEGLAPPLICATION" FORCE
      )
    else()
      set(WITH_WINDOWLESSGLXAPPLICATION OFF CACHE INTERNAL
                                                  "WITH_WINDOWLESSGLXAPPLICATION" FORCE
      )
      set(WITH_WINDOWLESSEGLAPPLICATION ON CACHE INTERNAL
                                                 "WITH_WINDOWLESSEGLAPPLICATION" FORCE
      )
    endif()
  endif()
  add_subdirectory("${DEPS_DIR}/magnum")
  add_subdirectory("${DEPS_DIR}/magnum-plugins")
  add_subdirectory("${DEPS_DIR}/magnum-integration")
  if(BUILD_PYTHON_BINDINGS)
    add_subdirectory("${DEPS_DIR}/magnum-bindings")
  endif()
endif()

if(NOT CORRADE_TARGET_EMSCRIPTEN)
  add_library(atomic_wait STATIC ${DEPS_DIR}/atomic_wait/atomic_wait.cpp)
  target_include_directories(atomic_wait PUBLIC ${DEPS_DIR}/atomic_wait)
endif()

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
