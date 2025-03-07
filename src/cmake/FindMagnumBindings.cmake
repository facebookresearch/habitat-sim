#.rst:
# Find Magnum bindings
# --------------------
#
# Finds helper headers for the Magnum bindings. Basic usage::
#
#  find_package(MagnumBindings REQUIRED)
#
# This command tries to find Magnum bindings and then defines the following:
#
#  MagnumBindings_FOUND         - Whether Magnum bindings were found
#
# This command will not try to find any actual bindings. The bindings are:
#
#  Python                       - Python bindings
#
# Example usage with specifying the components is::
#
#  find_package(MagnumBindings REQUIRED Python)
#
# For each component is then defined:
#
#  MagnumBindings_*_FOUND       - Whether the bindings were found
#  MagnumBindings::*            - Imported target
#
# The package is found if the headers are found. None of the bindings expose
# any library at the moment.
#
# Additionally these variables are defined for internal usage:
#
#  MAGNUMBINDINGS_INCLUDE_DIR   - Magnum bindings include dir (w/o dependencies)
#

#
#   This file is part of Magnum.
#
#   Copyright © 2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019,
#               2020, 2021, 2022, 2023, 2024, 2025
#             Vladimír Vondruš <mosra@centrum.cz>
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

find_package(Magnum REQUIRED)

# Global include dir that's unique to Magnum Bindings. Often they will be
# installed alongside Magnum, which is why the hint, but if not, it shouldn't
# just pick MAGNUM_INCLUDE_DIR because then _MAGNUMBINDINGS_*_INCLUDE_DIR will
# fail to be found. In case of CMake subprojects the versionBindings.h is
# generated inside the build dir so this won't find it, instead
# src/CMakeLists.txt forcibly sets MAGNUMBINDINGS_INCLUDE_DIR as an internal
# cache value to make that work.
find_path(MAGNUMBINDINGS_INCLUDE_DIR Magnum/versionBindings.h
    HINTS ${MAGNUM_INCLUDE_DIR})
mark_as_advanced(MAGNUMBINDINGS_INCLUDE_DIR)

# CMake module dir for dependencies. It might not be present at all if no
# feature that needs them is enabled, in which case it'll be left at NOTFOUND.
# But in that case it should also not be subsequently needed for any
# find_package(). If this is called from a superproject, the
# _MAGNUMBINDINGS_DEPENDENCY_MODULE_DIR is already set by
# modules/CMakeLists.txt.
#
# There's no dependency Find modules so far. Once there are, uncomment this and
# list the modules in NAMES.
#find_path(_MAGNUMBINDINGS_DEPENDENCY_MODULE_DIR
#    NAMES
#    PATH_SUFFIXES share/cmake/MagnumBindings/dependencies)
#mark_as_advanced(_MAGNUMBINDINGS_DEPENDENCY_MODULE_DIR)

# If the module dir is found and is not present in CMAKE_MODULE_PATH already
# (such as when someone explicitly added it, or if it's the Magnum's modules/
# dir in case of a superproject), add it as the first before all other. Set a
# flag to remove it again at the end, so the modules don't clash with Find
# modules of the same name from other projects.
if(_MAGNUMBINDINGS_DEPENDENCY_MODULE_DIR AND NOT _MAGNUMBINDINGS_DEPENDENCY_MODULE_DIR IN_LIST CMAKE_MODULE_PATH)
    set(CMAKE_MODULE_PATH ${_MAGNUMBINDINGS_DEPENDENCY_MODULE_DIR} ${CMAKE_MODULE_PATH})
    set(_MAGNUMBINDINGS_REMOVE_DEPENDENCY_MODULE_DIR_FROM_CMAKE_PATH ON)
else()
    unset(_MAGNUMBINDINGS_REMOVE_DEPENDENCY_MODULE_DIR_FROM_CMAKE_PATH)
endif()

# Component distinction (listing them explicitly to avoid mistakes with finding
# components from other repositories)
set(_MAGNUMBINDINGS_HEADER_ONLY_COMPONENTS Python)
# Nothing is enabled by default right now
set(_MAGNUMBINDINGS_IMPLICITLY_ENABLED_COMPONENTS )

# No inter-component dependencies right now

# Ensure that all inter-component dependencies are specified as well
set(_MAGNUMBINDINGS_ADDITIONAL_COMPONENTS )
foreach(_component ${MagnumBindings_FIND_COMPONENTS})
    # Mark the dependencies as required if the component is also required
    if(MagnumBindings_FIND_REQUIRED_${_component})
        foreach(_dependency ${_MAGNUMBINDINGS_${_component}_DEPENDENCIES})
            set(MagnumBindings_FIND_REQUIRED_${_dependency} TRUE)
        endforeach()
    endif()

    list(APPEND _MAGNUMBINDINGS_ADDITIONAL_COMPONENTS ${_MAGNUMBINDINGS_${_component}_DEPENDENCIES})
endforeach()

# Join the lists, remove duplicate components
set(_MAGNUMBINDINGS_ORIGINAL_FIND_COMPONENTS ${MagnumBindings_FIND_COMPONENTS})
if(_MAGNUMBINDINGS_ADDITIONAL_COMPONENTS)
    list(INSERT MagnumBindings_FIND_COMPONENTS 0 ${_MAGNUMBINDINGS_ADDITIONAL_COMPONENTS})
endif()
if(MagnumBindings_FIND_COMPONENTS)
    list(REMOVE_DUPLICATES MagnumBindings_FIND_COMPONENTS)
endif()

# Special cases of include paths. Libraries not listed here have a path suffix
# and include name derived from the library name in the loop below. (So far no
# special cases.)

# Find all components
foreach(_component ${MagnumBindings_FIND_COMPONENTS})
    string(TOUPPER ${_component} _COMPONENT)

    # Create imported target in case the library is found. If the project is
    # added as subproject to CMake, the target already exists and all the
    # required setup is already done from the build tree.
    if(TARGET "MagnumBindings::${_component}") # Quotes to fix KDE's higlighter
        set(MagnumBindings_${_component}_FOUND TRUE)
    else()
        # Header-only components
        if(_component IN_LIST _MAGNUMBINDINGS_HEADER_ONLY_COMPONENTS)
            # Include path names to find, unless specified above
            if(NOT _MAGNUMBINDINGS_${_COMPONENT}_INCLUDE_PATH_NAMES)
                set(_MAGNUMBINDINGS_${_COMPONENT}_INCLUDE_PATH_NAMES ${_component}Bindings.h)
            endif()
        endif()

        # Find library includes
        if(_component IN_LIST _MAGNUMBINDINGS_HEADER_ONLY_COMPONENTS)
            find_path(_MAGNUMBINDINGS_${_COMPONENT}_INCLUDE_DIR
                NAMES ${_MAGNUMBINDINGS_${_COMPONENT}_INCLUDE_PATH_NAMES}
                HINTS ${MAGNUMBINDINGS_INCLUDE_DIR})
            mark_as_advanced(_MAGNUMBINDINGS_${_COMPONENT}_INCLUDE_DIR)
        endif()

        # Decide if the component was found. If not, skip the rest, which
        # populates the target properties and finds additional dependencies.
        if(_component IN_LIST _MAGNUMBINDINGS_HEADER_ONLY_COMPONENTS AND _MAGNUMBINDINGS_${_COMPONENT}_INCLUDE_DIR)
            set(MagnumBindings_${_component}_FOUND TRUE)
        else()
            set(MagnumBindings_${_component}_FOUND FALSE)
            continue()
        endif()

        # Target for header-only library components
        if(_component IN_LIST _MAGNUMBINDINGS_HEADER_ONLY_COMPONENTS)
            add_library(MagnumBindings::${_component} INTERFACE IMPORTED)
        endif()

        # No special setup for Python bindings

        if(_component IN_LIST _MAGNUMBINDINGS_HEADER_ONLY_COMPONENTS)
            # Link to core Magnum library
            set_property(TARGET MagnumBindings::${_component} APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES Magnum::Magnum)

            # Add bindings include dir
            set_property(TARGET MagnumBindings::${_component} APPEND PROPERTY
                INTERFACE_INCLUDE_DIRECTORIES ${MAGNUMBINDINGS_INCLUDE_DIR})
        endif()
    endif()
endforeach()

# For CMake 3.16+ with REASON_FAILURE_MESSAGE, provide additional potentially
# useful info about the failed components.
if(NOT CMAKE_VERSION VERSION_LESS 3.16)
    set(_MAGNUMBINDINGS_REASON_FAILURE_MESSAGE)
    # Go only through the originally specified find_package() components, not
    # the dependencies added by us afterwards
    foreach(_component ${_MAGNUMBINDINGS_ORIGINAL_FIND_COMPONENTS})
        if(MagnumBindings_${_component}_FOUND)
            continue()
        endif()

        # If it's not known at all, tell the user -- it might be a new library
        # and an old Find module, or something platform-specific.
        if(NOT _component IN_LIST _MAGNUMBINDINGS_LIBRARY_COMPONENTS AND NOT _component IN_LIST _MAGNUMBINDINGS_PLUGIN_COMPONENTS)
            list(APPEND _MAGNUMBINDINGS_REASON_FAILURE_MESSAGE "${_component} is not a known component on this platform.")
        # Otherwise, if it's not among implicitly built components, hint that
        # the user may need to enable it
        # TODO: currently, the _FOUND variable doesn't reflect if dependencies
        #   were found. When it will, this needs to be updated to avoid
        #   misleading messages.
        elseif(NOT _component IN_LIST _MAGNUMBINDINGS_IMPLICITLY_ENABLED_COMPONENTS)
            string(TOUPPER ${_component} _COMPONENT)
            list(APPEND _MAGNUMBINDINGS_REASON_FAILURE_MESSAGE "${_component} is not built by default. Make sure you enabled MAGNUM_WITH_${_COMPONENT} when building Magnum Bindings")
        # Otherwise we have no idea. Better be silent than to print something
        # misleading.
        else()
        endif()
    endforeach()

    string(REPLACE ";" " " _MAGNUMBINDINGS_REASON_FAILURE_MESSAGE "${_MAGNUMBINDINGS_REASON_FAILURE_MESSAGE}")
    set(_MAGNUMBINDINGS_REASON_FAILURE_MESSAGE REASON_FAILURE_MESSAGE "${_MAGNUMBINDINGS_REASON_FAILURE_MESSAGE}")
endif()

# Remove Magnum Extras dependency module dir from CMAKE_MODULE_PATH again. Do
# it before the FPHSA call which may exit early in case of a failure.
if(_MAGNUMBINDINGS_REMOVE_DEPENDENCY_MODULE_DIR_FROM_CMAKE_PATH)
    list(REMOVE_ITEM CMAKE_MODULE_PATH ${_MAGNUMBINDINGS_DEPENDENCY_MODULE_DIR})
    unset(_MAGNUMBINDINGS_REMOVE_DEPENDENCY_MODULE_DIR_FROM_CMAKE_PATH)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MagnumBindings
    REQUIRED_VARS MAGNUMBINDINGS_INCLUDE_DIR
    HANDLE_COMPONENTS
    ${_MAGNUMBINDINGS_REASON_FAILURE_MESSAGE})
