#.rst:
# Find Assimp
# -------------
#
# Finds the Assimp library. This module defines:
#
#  Assimp_FOUND           - True if Assimp library is found
#  Assimp::Assimp         - Assimp imported target
#
# Additionally these variables are defined for internal usage:
#
#  ASSIMP_LIBRARY         - Assimp library
#  ASSIMP_INCLUDE_DIR     - Include dir
#

#
#   This file is part of Magnum.
#
#   Copyright © 2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019,
#               2020 Vladimír Vondruš <mosra@centrum.cz>
#   Copyright © 2017 Jonathan Hale <squareys@googlemail.com>
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

find_path(ASSIMP_INCLUDE_DIR NAMES assimp/anim.h HINTS include)

if(WIN32 AND MSVC)
    # Adapted from https://github.com/assimp/assimp/blob/799fd74714f9ffac29004c6b5a674b3402524094/CMakeLists.txt#L645-L655
    # with versions below MSVC 2015 (14.0 / 1900) removed, and the discouraged
    # use of MSVCxy replaced with MSVC_VERSION. See also
    # https://en.wikipedia.org/wiki/Microsoft_Visual_C%2B%2B#Internal_version_numbering
    if(MSVC_TOOLSET_VERSION) # available only since CMake 3.12
        set(ASSIMP_MSVC_VERSION vc${MSVC_TOOLSET_VERSION})
    elseif(MSVC_VERSION VERSION_LESS 1910)
        set(ASSIMP_MSVC_VERSION vc140)
    elseif(MSVC_VERSION VERSION_LESS 1920)
        set(ASSIMP_MSVC_VERSION vc141)
    elseif(MSVC_VERSION VERSION_LESS 1930)
        set(ASSIMP_MSVC_VERSION vc142)
    else()
        message(FATAL_ERROR "Unsupported MSVC version")
    endif()

    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(ASSIMP_LIBRARY_DIR "lib64")
    elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
        set(ASSIMP_LIBRARY_DIR "lib32")
    endif()

    find_library(ASSIMP_LIBRARY_RELEASE assimp-${ASSIMP_MSVC_VERSION}-mt.lib PATHS ${ASSIMP_LIBRARY_DIR})
    find_library(ASSIMP_LIBRARY_DEBUG assimp-${ASSIMP_MSVC_VERSION}-mtd.lib PATHS ${ASSIMP_LIBRARY_DIR})
else()
    find_library(ASSIMP_LIBRARY_RELEASE assimp)
    find_library(ASSIMP_LIBRARY_DEBUG assimpd)
endif()

# Static build of Assimp (built with Vcpkg, any system) depends on IrrXML, find
# that one as well. If not found, simply don't link to it --- it might be a
# dynamic build (on Windows it's a *.lib either way), or a static build using
# system IrrXML. Related issue: https://github.com/Microsoft/vcpkg/issues/5012
if(ASSIMP_LIBRARY_DEBUG MATCHES "${CMAKE_STATIC_LIBRARY_SUFFIX}$" OR ASSIMP_LIBRARY_RELEASE MATCHES "${CMAKE_STATIC_LIBRARY_SUFFIX}$")
    find_library(ASSIMP_IRRXML_LIBRARY_RELEASE IrrXML)
    find_library(ASSIMP_IRRXML_LIBRARY_DEBUG IrrXMLd)
endif()

include(SelectLibraryConfigurations)
select_library_configurations(ASSIMP)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Assimp DEFAULT_MSG
    ASSIMP_LIBRARY
    ASSIMP_INCLUDE_DIR)

if(NOT TARGET Assimp::Assimp)
    add_library(Assimp::Assimp UNKNOWN IMPORTED)

    if(ASSIMP_LIBRARY_DEBUG AND ASSIMP_LIBRARY_RELEASE)
        set_target_properties(Assimp::Assimp PROPERTIES
            IMPORTED_LOCATION_DEBUG ${ASSIMP_LIBRARY_DEBUG}
            IMPORTED_LOCATION_RELEASE ${ASSIMP_LIBRARY_RELEASE})
    else()
        set_target_properties(Assimp::Assimp PROPERTIES
            IMPORTED_LOCATION ${ASSIMP_LIBRARY})
    endif()

    # Link to IrrXML as well, if found. See the comment above for details.
    if(ASSIMP_IRRXML_LIBRARY_RELEASE)
        set_property(TARGET Assimp::Assimp APPEND PROPERTY
            INTERFACE_LINK_LIBRARIES $<$<NOT:$<CONFIG:Debug>>:${ASSIMP_IRRXML_LIBRARY_RELEASE}>)
    endif()
    if(ASSIMP_IRRXML_LIBRARY_DEBUG)
        set_property(TARGET Assimp::Assimp APPEND PROPERTY
            INTERFACE_LINK_LIBRARIES $<$<CONFIG:Debug>:${ASSIMP_IRRXML_LIBRARY_DEBUG}>)
    endif()

    set_target_properties(Assimp::Assimp PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES ${ASSIMP_INCLUDE_DIR})
endif()
