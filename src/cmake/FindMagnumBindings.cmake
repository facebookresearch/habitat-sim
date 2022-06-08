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
#               2020, 2021, 2022 Vladimír Vondruš <mosra@centrum.cz>
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

# Global bindings include dir
find_path(MAGNUMBINDINGS_INCLUDE_DIR Magnum
    HINTS ${MAGNUMBINDINGS_INCLUDE_DIR})
mark_as_advanced(MAGNUMBINDINGS_INCLUDE_DIR)

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

# Find all components
foreach(_component ${MagnumBindings_FIND_COMPONENTS})
    string(TOUPPER ${_component} _COMPONENT)

    # Create imported target in case the library is found. If the project is
    # added as subproject to CMake, the target already exists and all the
    # required setup is already done from the build tree.
    if(TARGET MagnumBindings::${_component})
        set(MagnumBindings_${_component}_FOUND TRUE)
    else()
        # Header-only components
        if(_component IN_LIST _MAGNUMBINDINGS_HEADER_ONLY_COMPONENTS)
          add_library(MagnumBindings::${_component} INTERFACE IMPORTED)
        endif()

        # Python bindings
        if(_component STREQUAL Python)
            set(_MAGNUMBINDINGS_${_COMPONENT}_INCLUDE_PATH_NAMES Magnum/SceneGraph/PythonBindings.h)
        endif()

        if(_component IN_LIST _MAGNUMBINDINGS_HEADER_ONLY_COMPONENTS)
            # Find includes
            find_path(_MAGNUMBINDINGS_${_COMPONENT}_INCLUDE_DIR
                NAMES ${_MAGNUMBINDINGS_${_COMPONENT}_INCLUDE_PATH_NAMES}
                HINTS ${MAGNUMBINDINGS_INCLUDE_DIR})
            mark_as_advanced(_MAGNUMBINDINGS_${_COMPONENT}_INCLUDE_DIR)

            # Link to core Magnum library
            set_property(TARGET MagnumBindings::${_component} APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES Magnum::Magnum)

            # Add bindings incldue dir
            set_property(TARGET MagnumBindings::${_component} APPEND PROPERTY
                INTERFACE_INCLUDE_DIRECTORIES ${MAGNUMBINDINGS_INCLUDE_DIR})
        endif()

        # Decide if the component was found
        if(_component IN_LIST _MAGNUMBINDINGS_HEADER_ONLY_COMPONENTS AND _MAGNUMBINDINGS_${_COMPONENT}_INCLUDE_DIR)
            set(MagnumBindings_${_component}_FOUND TRUE)
        else()
            set(MagnumBindings_${_component}_FOUND FALSE)
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
            list(APPEND _MAGNUMBINDINGS_REASON_FAILURE_MESSAGE "${_component} is not built by default. Make sure you enabled WITH_${_COMPONENT} when building Magnum Bindings")
        # Otherwise we have no idea. Better be silent than to print something
        # misleading.
        else()
        endif()
    endforeach()

    string(REPLACE ";" " " _MAGNUMBINDINGS_REASON_FAILURE_MESSAGE "${_MAGNUMBINDINGS_REASON_FAILURE_MESSAGE}")
    set(_MAGNUMBINDINGS_REASON_FAILURE_MESSAGE REASON_FAILURE_MESSAGE "${_MAGNUMBINDINGS_REASON_FAILURE_MESSAGE}")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MagnumBindings
    REQUIRED_VARS MAGNUMBINDINGS_INCLUDE_DIR
    HANDLE_COMPONENTS
    ${_MAGNUMBINDINGS_REASON_FAILURE_MESSAGE})
