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
#               2020, 2021, 2022 Vladimír Vondruš <mosra@centrum.cz>
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

# Global integration include dir
find_path(MAGNUMINTEGRATION_INCLUDE_DIR Magnum
    HINTS ${MAGNUM_INCLUDE_DIR})
mark_as_advanced(MAGNUMINTEGRATION_INCLUDE_DIR)

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

# Find all components
foreach(_component ${MagnumIntegration_FIND_COMPONENTS})
    string(TOUPPER ${_component} _COMPONENT)

    # Create imported target in case the library is found. If the project is
    # added as subproject to CMake, the target already exists and all the
    # required setup is already done from the build tree.
    if(TARGET MagnumIntegration::${_component})
        set(MagnumIntegration_${_component}_FOUND TRUE)
    else()
        # Library components
        if(_component IN_LIST _MAGNUMINTEGRATION_LIBRARY_COMPONENTS AND NOT _component IN_LIST _MAGNUMINTEGRATION_HEADER_ONLY_COMPONENTS)
            add_library(MagnumIntegration::${_component} UNKNOWN IMPORTED)

            # Try to find both debug and release version
            find_library(MAGNUMINTEGRATION_${_COMPONENT}_LIBRARY_DEBUG Magnum${_component}Integration-d)
            find_library(MAGNUMINTEGRATION_${_COMPONENT}_LIBRARY_RELEASE Magnum${_component}Integration)
            mark_as_advanced(MAGNUMINTEGRATION_${_COMPONENT}_LIBRARY_DEBUG
                MAGNUMINTEGRATION_${_COMPONENT}_LIBRARY_RELEASE)

            if(MAGNUMINTEGRATION_${_COMPONENT}_LIBRARY_RELEASE)
                set_property(TARGET MagnumIntegration::${_component} APPEND PROPERTY
                    IMPORTED_CONFIGURATIONS RELEASE)
                set_property(TARGET MagnumIntegration::${_component} PROPERTY
                    IMPORTED_LOCATION_RELEASE ${MAGNUMINTEGRATION_${_COMPONENT}_LIBRARY_RELEASE})
            endif()

            if(MAGNUMINTEGRATION_${_COMPONENT}_LIBRARY_DEBUG)
                set_property(TARGET MagnumIntegration::${_component} APPEND PROPERTY
                    IMPORTED_CONFIGURATIONS DEBUG)
                set_property(TARGET MagnumIntegration::${_component} PROPERTY
                    IMPORTED_LOCATION_DEBUG ${MAGNUMINTEGRATION_${_COMPONENT}_LIBRARY_DEBUG})
            endif()

        # Header-only library components
        elseif(_component IN_LIST _MAGNUMINTEGRATION_HEADER_ONLY_COMPONENTS)
            add_library(MagnumIntegration::${_component} INTERFACE IMPORTED)

        # Something unknown, skip. FPHSA will take care of handling this below.
        else()
            continue()
        endif()

        # Bullet integration library
        if(_component STREQUAL Bullet)
            # On Emscripten, Bullet could be taken from ports. If that's the
            # case, propagate proper compiler flag.
            if(CORRADE_TARGET_EMSCRIPTEN)
                find_file(_MAGNUMINTEGRATION_${_COMPONENT}_CONFIGURE_FILE configure.h
                    HINTS ${MAGNUMINTEGRATION_INCLUDE_DIR}/Magnum/${_component}Integration)
                file(READ ${_MAGNUMINTEGRATION_${_COMPONENT}_CONFIGURE_FILE} _magnum${_component}IntegrationConfigure)
                string(FIND "${_magnum${_component}IntegrationConfigure}" "#define MAGNUM_USE_EMSCRIPTEN_PORTS_BULLET" _magnum${_component}Integration_USE_EMSCRIPTEN_PORTS_BULLET)
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

            set(_MAGNUMINTEGRATION_${_COMPONENT}_INCLUDE_PATH_NAMES MotionState.h)

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

            set(_MAGNUMINTEGRATION_${_COMPONENT}_INCLUDE_PATH_NAMES Integration.h)

        # ImGui integration library
        elseif(_component STREQUAL ImGui)
            find_package(ImGui)
            set_property(TARGET MagnumIntegration::${_component} APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES ImGui::ImGui)

            set(_MAGNUMINTEGRATION_${_COMPONENT}_INCLUDE_PATH_NAMES Integration.h)

        # GLM integration library
        elseif(_component STREQUAL Glm)
            find_package(GLM)
            set_property(TARGET MagnumIntegration::${_component} APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES GLM::GLM)

            set(_MAGNUMINTEGRATION_${_COMPONENT}_INCLUDE_PATH_NAMES Integration.h)

        # Dart integration library
        elseif(_component STREQUAL Dart)
            find_package(DART 6.0.0 CONFIG REQUIRED)
            set_property(TARGET MagnumIntegration::${_component} APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES dart)

            set(_MAGNUMINTEGRATION_${_COMPONENT}_INCLUDE_PATH_NAMES ConvertShapeNode.h)

        # Oculus SDK integration library
        elseif(_component STREQUAL Ovr)
            find_package(OVR)
            set_property(TARGET MagnumIntegration::${_component} APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES OVR::OVR)

            set(_MAGNUMINTEGRATION_${_COMPONENT}_INCLUDE_PATH_NAMES OvrIntegration.h)
        endif()

        # Find library includes
        if(_component IN_LIST _MAGNUMINTEGRATION_LIBRARY_COMPONENTS)
            find_path(_MAGNUMINTEGRATION_${_COMPONENT}_INCLUDE_DIR
                NAMES ${_MAGNUMINTEGRATION_${_COMPONENT}_INCLUDE_PATH_NAMES}
                HINTS ${MAGNUMINTEGRATION_INCLUDE_DIR}/Magnum/${_component}Integration)
            mark_as_advanced(_MAGNUMINTEGRATION_${_COMPONENT}_INCLUDE_DIR)
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

        # Decide if the library was found
        if(_component IN_LIST _MAGNUMINTEGRATION_LIBRARY_COMPONENTS AND _MAGNUMINTEGRATION_${_COMPONENT}_INCLUDE_DIR AND (_component IN_LIST _MAGNUMINTEGRATION_HEADER_ONLY_COMPONENTS OR MAGNUMINTEGRATION_${_COMPONENT}_LIBRARY_DEBUG OR MAGNUMINTEGRATION_${_COMPONENT}_LIBRARY_RELEASE))
            set(MagnumIntegration_${_component}_FOUND TRUE)
        else()
            set(MagnumIntegration_${_component}_FOUND FALSE)
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

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MagnumIntegration
    REQUIRED_VARS MAGNUMINTEGRATION_INCLUDE_DIR
    HANDLE_COMPONENTS
    ${_MAGNUMINTEGRATION_REASON_FAILURE_MESSAGE})
