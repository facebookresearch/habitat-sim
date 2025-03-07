#.rst:
# Find Magnum integration library
# -------------------------------
#
# Finds the Magnum integration library. Basic usage::
#
#  find_package(MagnumIntegration REQUIRED)
#
# This command tries to find Magnum integration library and then defines the
# following:
#
#  MagnumIntegration_FOUND      - Whether the library was found
#
# This command alone is useless without specifying the components:
#
#  Bullet                       - Bullet Physics integration library
#  Dart                         - Dart Physics integration library
#  Eigen                        - Eigen integration library
#  Glm                          - GLM integration library
#  ImGui                        - ImGui integration library
#  Ovr                          - Oculus SDK integration library
#
# Example usage with specifying additional components is:
#
#  find_package(MagnumIntegration REQUIRED Bullet)
#
# For each component is then defined:
#
#  MagnumIntegration_*_FOUND    - Whether the component was found
#  MagnumIntegration::*         - Component imported target
#
# The package is found if either debug or release version of each requested
# library is found. If both debug and release libraries are found, proper
# version is chosen based on actual build configuration of the project (i.e.
# Debug build is linked to debug libraries, Release build to release
# libraries).
#
# Additionally these variables are defined for internal usage:
#
#  MAGNUMINTEGRATION_INCLUDE_DIR - Magnum integration include dir (w/o
#   dependencies)
#  MAGNUMINTEGRATION_*_LIBRARY_DEBUG - Debug version of given library, if found
#  MAGNUMINTEGRATION_*_LIBRARY_RELEASE - Release version of given library, if
#   found
#

#
#   This file is part of Magnum.
#
#   Copyright © 2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019,
#               2020, 2021, 2022, 2023, 2024, 2025
#             Vladimír Vondruš <mosra@centrum.cz>
#   Copyright © 2018 Konstantinos Chatzilygeroudis <costashatz@gmail.com>
#
#   Permission is hereby granted, free of charge, to any person obtaining a
#   copy of this software and associated documentation files (the "Software"),
#   to deal in the Software without restriction, including without limitation
#   the rights to use, copy, modify, merge, publish, distribute, sublicense,
#   and/or sell copies of the Software, and to permit persons to whom the
#   Software is furnished to do so, subject to the following conditions:
#
#   The above copyright notice and this permission notice shall be included
#   in all copies or substantial portions of the Software.
#
#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
#   THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
#   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
#   DEALINGS IN THE SOFTWARE.
#

# Magnum library dependencies
set(_MAGNUMINTEGRATION_DEPENDENCIES )
foreach(_component ${MagnumIntegration_FIND_COMPONENTS})
    if(_component STREQUAL Bullet)
        set(_MAGNUMINTEGRATION_${_component}_MAGNUM_DEPENDENCIES SceneGraph Shaders GL)
    elseif(_component STREQUAL Dart)
        set(_MAGNUMINTEGRATION_${_component}_MAGNUM_DEPENDENCIES SceneGraph Primitives MeshTools GL)
    elseif(_component STREQUAL ImGui)
        set(_MAGNUMINTEGRATION_${_component}_MAGNUM_DEPENDENCIES GL Shaders)
    endif()

    list(APPEND _MAGNUMINTEGRATION_DEPENDENCIES ${_MAGNUMINTEGRATION_${_component}_MAGNUM_DEPENDENCIES})
    list(APPEND _MAGNUMINTEGRATION_OPTIONAL_DEPENDENCIES ${_MAGNUMINTEGRATION_${_component}_MAGNUM_OPTIONAL_DEPENDENCIES})
endforeach()
find_package(Magnum REQUIRED ${_MAGNUMINTEGRATION_DEPENDENCIES})
if(_MAGNUMINTEGRATION_OPTIONAL_DEPENDENCIES)
    find_package(Magnum OPTIONAL_COMPONENTS ${_MAGNUMINTEGRATION_OPTIONAL_DEPENDENCIES})
endif()

# Global include dir that's unique to Magnum Integration. Often it will be
# installed alongside Magnum, which is why the hint, but if not, it shouldn't
# just pick MAGNUM_INCLUDE_DIR because then _MAGNUMINTEGRATION_*_INCLUDE_DIR
# will fail to be found. In case of CMake subprojects the versionIntegration.h
# is generated inside the build dir so this won't find it, instead
# src/CMakeLists.txt forcibly sets MAGNUMINTEGRATION_INCLUDE_DIR as an internal
# cache value to make that work.
find_path(MAGNUMINTEGRATION_INCLUDE_DIR Magnum/versionIntegration.h
    HINTS ${MAGNUM_INCLUDE_DIR})
mark_as_advanced(MAGNUMINTEGRATION_INCLUDE_DIR)

# CMake module dir for dependencies. It might not be present at all if no
# feature that needs them is enabled, in which case it'll be left at NOTFOUND.
# But in that case it should also not be subsequently needed for any
# find_package(). If this is called from a superproject, the
# _MAGNUMINTEGRATION_DEPENDENCY_MODULE_DIR is already set by
# modules/CMakeLists.txt.
find_path(_MAGNUMINTEGRATION_DEPENDENCY_MODULE_DIR
    NAMES
        FindBullet.cmake FindGLM.cmake FindImGui.cmake FindOVR.cmake
    PATH_SUFFIXES share/cmake/MagnumIntegration/dependencies)
mark_as_advanced(_MAGNUMINTEGRATION_DEPENDENCY_MODULE_DIR)

# If the module dir is found and is not present in CMAKE_MODULE_PATH already
# (such as when someone explicitly added it, or if it's the Magnum's modules/
# dir in case of a superproject), add it as the first before all other. Set a
# flag to remove it again at the end, so the modules don't clash with Find
# modules of the same name from other projects.
if(_MAGNUMINTEGRATION_DEPENDENCY_MODULE_DIR AND NOT _MAGNUMINTEGRATION_DEPENDENCY_MODULE_DIR IN_LIST CMAKE_MODULE_PATH)
    set(CMAKE_MODULE_PATH ${_MAGNUMINTEGRATION_DEPENDENCY_MODULE_DIR} ${CMAKE_MODULE_PATH})
    set(_MAGNUMINTEGRATION_REMOVE_DEPENDENCY_MODULE_DIR_FROM_CMAKE_PATH ON)
else()
    unset(_MAGNUMINTEGRATION_REMOVE_DEPENDENCY_MODULE_DIR_FROM_CMAKE_PATH)
endif()

# Component distinction (listing them explicitly to avoid mistakes with finding
# components from other repositories)
set(_MAGNUMINTEGRATION_LIBRARY_COMPONENTS Bullet Dart Eigen ImGui Glm)
if(CORRADE_TARGET_WINDOWS)
    list(APPEND _MAGNUMINTEGRATION_LIBRARY_COMPONENTS Ovr)
endif()
set(_MAGNUMINTEGRATION_HEADER_ONLY_COMPONENTS Eigen)
# Nothing is enabled by default right now
set(_MAGNUMINTEGRATION_IMPLICITLY_ENABLED_COMPONENTS )

# Inter-component dependencies (none yet)
# set(_MAGNUMINTEGRATION_Component_DEPENDENCIES Dependency)

# Ensure that all inter-component dependencies are specified as well
set(_MAGNUMINTEGRATION_ADDITIONAL_COMPONENTS )
foreach(_component ${MagnumIntegration_FIND_COMPONENTS})
    # Mark the dependencies as required if the component is also required
    if(MagnumIntegration_FIND_REQUIRED_${_component})
        foreach(_dependency ${_MAGNUMINTEGRATION_${_component}_DEPENDENCIES})
            set(MagnumIntegration_FIND_REQUIRED_${_dependency} TRUE)
        endforeach()
    endif()

    list(APPEND _MAGNUMINTEGRATION_ADDITIONAL_COMPONENTS ${_MAGNUMINTEGRATION_${_component}_DEPENDENCIES})
endforeach()

# Join the lists, remove duplicate components
set(_MAGNUMINTEGRATION_ORIGINAL_FIND_COMPONENTS ${MagnumIntegration_FIND_COMPONENTS})
if(_MAGNUMINTEGRATION_ADDITIONAL_COMPONENTS)
    list(INSERT MagnumIntegration_FIND_COMPONENTS 0 ${_MAGNUMINTEGRATION_ADDITIONAL_COMPONENTS})
endif()
if(MagnumIntegration_FIND_COMPONENTS)
    list(REMOVE_DUPLICATES MagnumIntegration_FIND_COMPONENTS)
endif()

# Special cases of include paths for header-only libraries. Libraries not
# listed here have a path suffix and include name derived from the library name
# in the loop below. Non-header-only libraries have a configure.h file.
set(_MAGNUMINTEGRATION_EIGEN_INCLUDE_PATH_NAMES GeometryIntegration.h)

# Find all components
foreach(_component ${MagnumIntegration_FIND_COMPONENTS})
    string(TOUPPER ${_component} _COMPONENT)

    # Create imported target in case the library is found. If the project is
    # added as subproject to CMake, the target already exists and all the
    # required setup is already done from the build tree.
    if(TARGET "MagnumIntegration::${_component}") # Quotes to fix KDE's hiliter
        set(MagnumIntegration_${_component}_FOUND TRUE)
    else()
        # Find library include dir for header-only libraries
        if(_component IN_LIST _MAGNUMINTEGRATION_HEADER_ONLY_COMPONENTS)
            # Include path names to find, unless specified above
            if(NOT _MAGNUMINTEGRATION_${_COMPONENT}_INCLUDE_PATH_NAMES)
                set(_MAGNUMINTEGRATION_${_COMPONENT}_INCLUDE_PATH_NAMES ${_comp
onent}Integration.h)
            endif()

            find_path(_MAGNUMINTEGRATION_${_COMPONENT}_INCLUDE_DIR
                NAMES ${_MAGNUMINTEGRATION_${_COMPONENT}_INCLUDE_PATH_NAMES}
                HINTS ${MAGNUMINTEGRATION_INCLUDE_DIR}/Magnum/${_component}Integration)
            mark_as_advanced(_MAGNUMINTEGRATION_${_COMPONENT}_CONFIGURE_FILE)

        # Non-header-only libraries have a configure file which we need to
        # subsequently read, so find that one directly
        elseif(_component IN_LIST _MAGNUMINTEGRATION_LIBRARY_COMPONENTS)
            find_file(_MAGNUMINTEGRATION_${_COMPONENT}_CONFIGURE_FILE configure.h
                HINTS ${MAGNUMINTEGRATION_INCLUDE_DIR}/Magnum/${_component}Integration)
            mark_as_advanced(_MAGNUMINTEGRATION_${_COMPONENT}_CONFIGURE_FILE)
        endif()

        # Library components
        if(_component IN_LIST _MAGNUMINTEGRATION_LIBRARY_COMPONENTS AND NOT _component IN_LIST _MAGNUMINTEGRATION_HEADER_ONLY_COMPONENTS)
            # Try to find both debug and release version
            find_library(MAGNUMINTEGRATION_${_COMPONENT}_LIBRARY_DEBUG Magnum${_component}Integration-d)
            find_library(MAGNUMINTEGRATION_${_COMPONENT}_LIBRARY_RELEASE Magnum${_component}Integration)
            mark_as_advanced(MAGNUMINTEGRATION_${_COMPONENT}_LIBRARY_DEBUG
                MAGNUMINTEGRATION_${_COMPONENT}_LIBRARY_RELEASE)

            # Determine if the library is static or dynamic by reading the
            # per-library config file. If the file wasn't found, skip this so
            # it fails on the FPHSA below and not right here.
            if(_MAGNUMINTEGRATION_${_COMPONENT}_CONFIGURE_FILE)
                file(READ ${_MAGNUMINTEGRATION_${_COMPONENT}_CONFIGURE_FILE} _magnumIntegrationConfigure)
                string(REGEX REPLACE ";" "\\\\;" _magnumIntegrationConfigure "${_magnumIntegrationConfigure}")
                string(REGEX REPLACE "\n" ";" _magnumIntegrationConfigure "${_magnumIntegrationConfigure}")
                list(FIND _magnumIntegrationConfigure "#define MAGNUM_${_COMPONENT}INTEGRATION_BUILD_STATIC" _magnumIntegrationBuildStatic)
                if(NOT _magnumIntegrationBuildStatic EQUAL -1)
                    # The variable is inconsistently named between C++ and
                    # CMake, so keep it underscored / private
                    set(_MAGNUMINTEGRATION_${_COMPONENT}_BUILD_STATIC ON)
                endif()
            endif()

            # On Windows, if we have a dynamic build of given library, find the
            # DLLs as well. Abuse find_program() since the DLLs should be
            # alongside usual executables. On MinGW they however have a lib
            # prefix.
            if(CORRADE_TARGET_WINDOWS AND NOT _MAGNUMINTEGRATION_${_COMPONENT}_BUILD_STATIC)
                find_program(MAGNUMINTEGRATION_${_COMPONENT}_DLL_DEBUG ${CMAKE_SHARED_LIBRARY_PREFIX}Magnum${_component}Integration-d.dll)
                find_program(MAGNUMINTEGRATION_${_COMPONENT}_DLL_RELEASE ${CMAKE_SHARED_LIBRARY_PREFIX}Magnum${_component}Integration.dll)
                mark_as_advanced(MAGNUMINTEGRATION_${_COMPONENT}_DLL_DEBUG
                    MAGNUMINTEGRATION_${_COMPONENT}_DLL_RELEASE)
            # If not on Windows or on a static build, unset the DLL variables
            # to avoid leaks when switching shared and static builds
            else()
                unset(MAGNUMINTEGRATION_${_COMPONENT}_DLL_DEBUG CACHE)
                unset(MAGNUMINTEGRATION_${_COMPONENT}_DLL_RELEASE CACHE)
            endif()

        # If not a header-only component it's something unknown, skip. FPHSA
        # will take care of handling this below.
        elseif(NOT _component IN_LIST _MAGNUMINTEGRATION_HEADER_ONLY_COMPONENTS)
            continue()
        endif()

        # Decide if the library was found. If not, skip the rest, which
        # populates the target properties and finds additional dependencies.
        # This means that the rest can also rely on that e.g. FindGLM.cmake is
        # present in _MAGNUMPLUGINS_DEPENDENCY_MODULE_DIR -- given that the
        # library needing GLM was found, it likely also installed FindGLM for
        # itself.
        if(
            # If the component is a header-only library it should have an
            # include dir
            (_component IN_LIST _MAGNUMINTEGRATION_HEADER_ONLY_COMPONENTS AND _MAGNUMINTEGRATION_${_COMPONENT}_INCLUDE_DIR) OR
            # Or, if it's a real library, it should have a configure file
            (_component IN_LIST _MAGNUMINTEGRATION_LIBRARY_COMPONENTS AND _MAGNUMINTEGRATION_${_COMPONENT}_CONFIGURE_FILE AND (
                # Or have a debug library, and a DLL found if expected
                (MAGNUMINTEGRATION_${_COMPONENT}_LIBRARY_DEBUG AND (
                    NOT DEFINED MAGNUMINTEGRATION_${_COMPONENT}_DLL_DEBUG OR
                    MAGNUMINTEGRATION_${_COMPONENT}_DLL_DEBUG)) OR
                # Or have a release library, and a DLL found if expected
                (MAGNUMINTEGRATION_${_COMPONENT}_LIBRARY_RELEASE AND (
                    NOT DEFINED MAGNUMINTEGRATION_${_COMPONENT}_DLL_RELEASE OR
                    MAGNUMINTEGRATION_${_COMPONENT}_DLL_RELEASE))))
        )
            set(MagnumIntegration_${_component}_FOUND TRUE)
        else()
            set(MagnumIntegration_${_component}_FOUND FALSE)
            continue()
        endif()

        # Target for header-only library components
        if(_component IN_LIST _MAGNUMINTEGRATION_HEADER_ONLY_COMPONENTS)
            add_library(MagnumIntegration::${_component} INTERFACE IMPORTED)

        # Target and location for libraries
        elseif(_component IN_LIST _MAGNUMINTEGRATION_LIBRARY_COMPONENTS)
            if(_MAGNUMINTEGRATION_${_COMPONENT}_BUILD_STATIC)
                add_library(MagnumIntegration::${_component} STATIC IMPORTED)
            else()
                add_library(MagnumIntegration::${_component} SHARED IMPORTED)
            endif()

            foreach(_CONFIG DEBUG RELEASE)
                if(NOT MAGNUMINTEGRATION_${_COMPONENT}_LIBRARY_${_CONFIG})
                    continue()
                endif()

                set_property(TARGET MagnumIntegration::${_component} APPEND PROPERTY
                    IMPORTED_CONFIGURATIONS ${_CONFIG})
                # Unfortunately for a DLL the two properties are swapped out,
                # *.lib goes to IMPLIB, so it's duplicated like this
                if(DEFINED MAGNUMINTEGRATION_${_COMPONENT}_DLL_${_CONFIG})
                    # Quotes to "fix" KDE's higlighter
                    set_target_properties("MagnumIntegration::${_component}" PROPERTIES
                        IMPORTED_LOCATION_${_CONFIG} ${MAGNUMINTEGRATION_${_COMPONENT}_DLL_${_CONFIG}}
                        IMPORTED_IMPLIB_${_CONFIG} ${MAGNUMINTEGRATION_${_COMPONENT}_LIBRARY_${_CONFIG}})
                else()
                    set_property(TARGET MagnumIntegration::${_component} PROPERTY
                        IMPORTED_LOCATION_${_CONFIG} ${MAGNUMINTEGRATION_${_COMPONENT}_LIBRARY_${_CONFIG}})
                endif()
            endforeach()
        endif()

        # Bullet integration library
        if(_component STREQUAL Bullet)
            # On Emscripten, Bullet could be taken from ports. If that's the
            # case, propagate proper compiler flag.
            if(CORRADE_TARGET_EMSCRIPTEN)
                # The library-specific configure file was read above already
                string(FIND "${_magnumIntegrationConfigure}" "#define MAGNUM_USE_EMSCRIPTEN_PORTS_BULLET" _magnum${_component}Integration_USE_EMSCRIPTEN_PORTS_BULLET)
                if(NOT _magnum${_component}Integration_USE_EMSCRIPTEN_PORTS_BULLET EQUAL -1)
                    set(MAGNUM_USE_EMSCRIPTEN_PORTS_BULLET 1)
                endif()
            endif()

            if(MAGNUM_USE_EMSCRIPTEN_PORTS_BULLET)
                if(CMAKE_VERSION VERSION_LESS 3.13)
                    message(FATAL_ERROR "BulletIntegration was compiled against emscripten-ports version but linking to it requires CMake 3.13 at least")
                endif()
                set_property(TARGET MagnumIntegration::${_component} APPEND PROPERTY
                    INTERFACE_COMPILE_OPTIONS "SHELL:-s USE_BULLET=1")
                set_property(TARGET MagnumIntegration::${_component} APPEND PROPERTY
                    INTERFACE_LINK_OPTIONS "SHELL:-s USE_BULLET=1")
            else()
                find_package(Bullet)
                set_property(TARGET MagnumIntegration::${_component} APPEND PROPERTY
                    INTERFACE_LINK_LIBRARIES Bullet::LinearMath)
            endif()

        # Eigen integration library
        elseif(_component STREQUAL Eigen)
            find_package(Eigen3)
            # We could drop this once we can use at least 3.3.1 (Ubuntu 16.04
            # has only 3.3 beta, which doesn't have this target yet), however
            # for Travis and AppVeyor we're using FindEigen3.cmake from the
            # downloaded sources (because the Eigen3Config.cmake, which
            # produces the actual targets, is not there -- only
            # Eigen3Config.cmake.in). See the YML files for an extended rant.
            # Also, FindEigen3 only defines EIGEN3_INCLUDE_DIR, not even
            # EIGEN3_INCLUDE_DIRS, so be extra careful.
            # http://eigen.tuxfamily.org/index.php?title=ChangeLog#Eigen_3.3.1
            set_property(TARGET MagnumIntegration::${_component} APPEND PROPERTY
                INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIR})

        # ImGui integration library
        elseif(_component STREQUAL ImGui)
            find_package(ImGui)
            set_property(TARGET MagnumIntegration::${_component} APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES ImGui::ImGui)

        # GLM integration library
        elseif(_component STREQUAL Glm)
            find_package(GLM)
            set_property(TARGET MagnumIntegration::${_component} APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES GLM::GLM)

        # Dart integration library
        elseif(_component STREQUAL Dart)
            find_package(DART 6.0.0 CONFIG REQUIRED)
            set_property(TARGET MagnumIntegration::${_component} APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES dart)

        # Oculus SDK integration library
        elseif(_component STREQUAL Ovr)
            find_package(OVR)
            set_property(TARGET MagnumIntegration::${_component} APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES OVR::OVR)
        endif()

        if(_component IN_LIST _MAGNUMINTEGRATION_LIBRARY_COMPONENTS)
            # Link to core Magnum library, add other Magnum required and
            # optional dependencies
            set_property(TARGET MagnumIntegration::${_component} APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES Magnum::Magnum)
            foreach(_dependency ${_MAGNUMINTEGRATION_${_component}_MAGNUM_DEPENDENCIES})
                set_property(TARGET MagnumIntegration::${_component} APPEND PROPERTY
                    INTERFACE_LINK_LIBRARIES Magnum::${_dependency})
            endforeach()
            foreach(_dependency ${_MAGNUMINTEGRATION_${_component}_MAGNUM_OPTIONAL_DEPENDENCIES})
                if(Magnum_${_dependency}_FOUND)
                    set_property(TARGET MagnumIntegration::${_component} APPEND     PROPERTY
                        INTERFACE_LINK_LIBRARIES Magnum::${_dependency})
                endif()
            endforeach()

            # Add inter-project dependencies
            foreach(_dependency ${_MAGNUMINTEGRATION_${_component}_DEPENDENCIES})
                set_property(TARGET MagnumIntegration::${_component} APPEND PROPERTY
                    INTERFACE_LINK_LIBRARIES MagnumIntegration::${_dependency})
            endforeach()
        endif()
    endif()
endforeach()

# For CMake 3.16+ with REASON_FAILURE_MESSAGE, provide additional potentially
# useful info about the failed components.
if(NOT CMAKE_VERSION VERSION_LESS 3.16)
    set(_MAGNUMINTEGRATION_REASON_FAILURE_MESSAGE )
    # Go only through the originally specified find_package() components, not
    # the dependencies added by us afterwards
    foreach(_component ${_MAGNUMINTEGRATION_ORIGINAL_FIND_COMPONENTS})
        if(MagnumIntegration_${_component}_FOUND)
            continue()
        endif()

        # If it's not known at all, tell the user -- it might be a new library
        # and an old Find module, or something platform-specific.
        if(NOT _component IN_LIST _MAGNUMINTEGRATION_LIBRARY_COMPONENTS)
            list(APPEND _MAGNUMINTEGRATION_REASON_FAILURE_MESSAGE "${_component} is not a known component on this platform.")
        # Otherwise, if it's not among implicitly built components, hint that
        # the user may need to enable it
        # TODO: currently, the _FOUND variable doesn't reflect if dependencies
        #   were found. When it will, this needs to be updated to avoid
        #   misleading messages.
        elseif(NOT _component IN_LIST _MAGNUMINTEGRATION_IMPLICITLY_ENABLED_COMPONENTS)
            string(TOUPPER ${_component} _COMPONENT)
            list(APPEND _MAGNUMINTEGRATION_REASON_FAILURE_MESSAGE "${_component} is not built by default. Make sure you enabled MAGNUM_WITH_${_COMPONENT} when building Magnum Integration.")
        # Otherwise we have no idea. Better be silent than to print something
        # misleading.
        else()
        endif()
    endforeach()

    string(REPLACE ";" " " _MAGNUMINTEGRATION_REASON_FAILURE_MESSAGE "${_MAGNUMINTEGRATION_REASON_FAILURE_MESSAGE}")
    set(_MAGNUMINTEGRATION_REASON_FAILURE_MESSAGE REASON_FAILURE_MESSAGE "${_MAGNUMINTEGRATION_REASON_FAILURE_MESSAGE}")
endif()

# Remove Magnum Integration dependency module dir from CMAKE_MODULE_PATH again.
# Do it before the FPHSA call which may exit early in case of a failure.
if(_MAGNUMINTEGRATION_REMOVE_DEPENDENCY_MODULE_DIR_FROM_CMAKE_PATH)
    list(REMOVE_ITEM CMAKE_MODULE_PATH ${_MAGNUMINTEGRATION_DEPENDENCY_MODULE_DIR})
    unset(_MAGNUMINTEGRATION_REMOVE_DEPENDENCY_MODULE_DIR_FROM_CMAKE_PATH)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MagnumIntegration
    REQUIRED_VARS MAGNUMINTEGRATION_INCLUDE_DIR
    HANDLE_COMPONENTS
    ${_MAGNUMINTEGRATION_REASON_FAILURE_MESSAGE})
