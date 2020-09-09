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
#               2020 Vladimír Vondruš <mosra@centrum.cz>
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
set(_MAGNUMBINDINGS_HEADER_ONLY_COMPONENT_LIST Python)

# Convert components lists to regular expressions so I can use if(MATCHES).
# TODO: Drop this once CMake 3.3 and if(IN_LIST) can be used
foreach(_WHAT HEADER_ONLY)
    string(REPLACE ";" "|" _MAGNUMBINDINGS_${_WHAT}_COMPONENTS "${_MAGNUMBINDINGS_${_WHAT}_COMPONENT_LIST}")
    set(_MAGNUMBINDINGS_${_WHAT}_COMPONENTS "^(${_MAGNUMBINDINGS_${_WHAT}_COMPONENTS})$")
endforeach()

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
        if(_component MATCHES ${_MAGNUMBINDINGS_HEADER_ONLY_COMPONENTS})
          add_library(MagnumBindings::${_component} INTERFACE IMPORTED)
        endif()

        # Python bindings
        if(_component STREQUAL Python)
            set(_MAGNUMBINDINGS_${_COMPONENT}_INCLUDE_PATH_NAMES Magnum/SceneGraph/PythonBindings.h)
        endif()

        if(_component MATCHES ${_MAGNUMBINDINGS_HEADER_ONLY_COMPONENTS})
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
        if(_component MATCHES ${_MAGNUMBINDINGS_HEADER_ONLY_COMPONENTS} AND _MAGNUMBINDINGS_${_COMPONENT}_INCLUDE_DIR)
            set(MagnumBindings_${_component}_FOUND TRUE)
        else()
            set(MagnumBindings_${_component}_FOUND FALSE)
        endif()
    endif()
endforeach()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MagnumBindings
    REQUIRED_VARS MAGNUMBINDINGS_INCLUDE_DIR
    HANDLE_COMPONENTS)
