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
#  Assimp_LIBRARY         - Assimp library
#  Assimp_INCLUDE_DIR     - Include dir
#

#
#   This file is part of Magnum.
#
#   Copyright © 2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019,
#               2020, 2021, 2022 Vladimír Vondruš <mosra@centrum.cz>
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

# For Vcpkg we *have to* use CMake config files as there it's a huge tree of
# static dependencies that would be impossible to figure out otherwise (see
# https://github.com/microsoft/vcpkg/pull/14554 for details), however vanilla
# Assimp config files are completely broken and thus we have to attempt to
# detect & ignore / patch them in most cases:
#
# 1. On macOS, assimp::assimp references `libassimpd.dylib.5` while it should
#    be `libassimpd.5.dylib`. The assimpTargets.cmake then commits an
#    irreversible suicide as the file does not exist. This got fixed in
#    https://github.com/assimp/assimp/commit/ae3236f4819f8b41e197694fd5f7a6df0f63c323
#    and later replaced with https://github.com/assimp/assimp/pull/3455 but
#    there's no version containing either of those yet. The only sane way to
#    fix this is to not do find_package(assimp CONFIG) on macOS. We do that by
#    setting CMAKE_DISABLE_FIND_PACKAGE_assimp so it's still possible to
#    disable this behavior from the outside (for example when using vcpkg, or
#    latest master) by explicitly setting that to OFF. (The automagic but
#    insane way to fix this would be by overriding message() to ignore the
#    FATAL_ERROR printed by assimpTargets.cmake, but overriden macros stay
#    alive forever, even in outer scopes, and that's not something I want to
#    live with. I had to do a similar hack for SPIRV-Tools already and I feel
#    dirty.)
# 2. On anything except Windows, the files doen't set IMPORTED_CONFIGURATIONS,
#    resulting in a warning about
#       IMPORTED_LOCATION not set for imported target "assimp::assimp"
#    and subsequently failing with
#       ninja: error: 'assimp::assimp-NOTFOUND', needed by '<target>',
#       missing and no known rule to make it
#    This got fixed with https://github.com/assimp/assimp/pull/3215 and again
#    later replaced with https://github.com/assimp/assimp/pull/3455 but there's
#    no version containing those yet either. This fortunately can be fixed "in
#    post" so we just detect that case below and skip aliasing the target to
#    assimp::assimp.
# 3. It doesn't end there though -- on static builds there's nothing that would
#    describe the actual static dependencies, which is THE MAIN REASON I wanted
#    to use the config file. So instead, when it looks like a static build and
#    there's no information about INTERFACE_LINK_LIBRARIES, we attempt to do it
#    ourselves instead.
# 4. Furthermore, the vanilla config file contains if(ON) and because its CMake
#    requirement is set to an insanely low version 2.6, CMake complains about a
#    policy change. To suppress it, neither cmake_policy(SET CMP0012 NEW) nor
#    find_package(... NO_POLICY_SCOPE) does anything, fortunately there's this
#    variable that's able to punch through the scope created by find_package():
#    https://cmake.org/cmake/help/latest/variable/CMAKE_POLICY_DEFAULT_CMPNNNN.html
#
# In conclusion, dynamic builds on MSVC are the only case where vanilla config
# files work, but we still need to go through this pain for vcpkg for the
# static dependencies.

# Assimp installs a config file that can give us all its dependencies in case
# of a static build. In case the assimp target is already present, we have it
# as a CMake subproject, so don't attempt to look for it again.
if(NOT TARGET assimp)
    # See Exhibit 1 above for details
    if(APPLE AND NOT DEFINED CMAKE_DISABLE_FIND_PACKAGE_assimp)
        set(CMAKE_DISABLE_FIND_PACKAGE_assimp ON)
    endif()
    # See Exhibit 4 above for details
    set(CMAKE_POLICY_DEFAULT_CMP0012 NEW)
    find_package(assimp CONFIG QUIET)
    unset(CMAKE_POLICY_DEFAULT_CMP0012)

    # Old config files (Assimp 3.2, i.e.) don't define any target at all, in
    # which case we don't even attempt to use anything from the config file
    if(assimp_FOUND AND TARGET assimp::assimp)
        # See Exhibit 2 above for details
        get_target_property(_ASSIMP_IMPORTED_CONFIGURATIONS assimp::assimp IMPORTED_CONFIGURATIONS)
        if(NOT _ASSIMP_IMPORTED_CONFIGURATIONS)
            set(_ASSIMP_HAS_USELESS_CONFIG ON)
        endif()

        # See Exhibit 3 above for details
        get_target_property(_ASSIMP_INTERFACE_LINK_LIBRARIES assimp::assimp INTERFACE_LINK_LIBRARIES)
        if(NOT ASSIMP_BUILD_SHARED_LIBS AND NOT _ASSIMP_INTERFACE_LINK_LIBRARIES)
            set(_ASSIMP_HAS_USELESS_CONFIG ON)
        endif()
    endif()
endif()

# In case we have Assimp as a CMake subproject or the config file was present,
# simply alias our target to that. The assimp config file is actually
# assimp::assimp and while the CMake subproject defines assimp::assimp as well,
# it's like that only since version 5:
#   https://github.com/assimp/assimp/commit/b43cf9233703305cfd8dfe7844fce959879b4f0c
#   https://github.com/assimp/assimp/commit/30d3c8c6a37a3b098702dfb714fe8e5e2abbfa5e
# The target aliasing is skipped in case the config files are crap, see above
if((TARGET assimp OR TARGET assimp::assimp) AND NOT _ASSIMP_HAS_USELESS_CONFIG)
    if(TARGET assimp)
        set(_ASSIMP_TARGET assimp)
    else()
        set(_ASSIMP_TARGET assimp::assimp)
    endif()

    get_target_property(_ASSIMP_INTERFACE_INCLUDE_DIRECTORIES ${_ASSIMP_TARGET} INTERFACE_INCLUDE_DIRECTORIES)
    # In case of a CMake subproject (which always has the assimp target, not
    # assimp::assimp, so we don't need to use ${_ASSIMP_TARGET}), the target
    # doesn't define any usable INTERFACE_INCLUDE_DIRECTORIES for some reason
    # (the $<BUILD_INTERFACE:> in there doesn't get expanded), so let's extract
    # that from the SOURCE_DIR property instead.
    if(_ASSIMP_INTERFACE_INCLUDE_DIRECTORIES MATCHES "<BUILD_INTERFACE:")
        get_target_property(_ASSIMP_INTERFACE_INCLUDE_DIRECTORIES assimp SOURCE_DIR)
        get_filename_component(_ASSIMP_INTERFACE_INCLUDE_DIRECTORIES ${_ASSIMP_INTERFACE_INCLUDE_DIRECTORIES} DIRECTORY)
        set(_ASSIMP_INTERFACE_INCLUDE_DIRECTORIES ${_ASSIMP_INTERFACE_INCLUDE_DIRECTORIES}/include)
    endif()

    if(NOT TARGET Assimp::Assimp)
        # Aliases of (global) targets are only supported in CMake 3.11, so we
        # work around it by this. This is easier than fetching all possible
        # properties (which are impossible to track of) and then attempting to
        # rebuild them into a new target.
        add_library(Assimp::Assimp INTERFACE IMPORTED)
        set_target_properties(Assimp::Assimp PROPERTIES
            INTERFACE_LINK_LIBRARIES ${_ASSIMP_TARGET})
        set_target_properties(Assimp::Assimp PROPERTIES
            # Needs to be wrapped in quotes because the shit Assimp config file
            # sets it to "${_IMPORT_PREFIX}/include;${_IMPORT_PREFIX}/include"
            # (twice the same dir) for some reason.
            INTERFACE_INCLUDE_DIRECTORIES "${_ASSIMP_INTERFACE_INCLUDE_DIRECTORIES}")
    endif()

    # Just to make FPHSA print some meaningful location, nothing else
    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args("Assimp" DEFAULT_MSG
        _ASSIMP_INTERFACE_INCLUDE_DIRECTORIES)

    return()
endif()

find_path(Assimp_INCLUDE_DIR NAMES assimp/anim.h
    HINTS
        include
        # For backwards compatibility
        ${ASSIMP_INCLUDE_DIR})

if(WIN32 AND MSVC)
    # Adapted from https://github.com/assimp/assimp/blob/799fd74714f9ffac29004c6b5a674b3402524094/CMakeLists.txt#L645-L655
    # with versions below MSVC 2015 (14.0 / 1900) removed, and the discouraged
    # use of MSVCxy replaced with MSVC_VERSION. See also
    # https://en.wikipedia.org/wiki/Microsoft_Visual_C%2B%2B#Internal_version_numbering
    if(MSVC_TOOLSET_VERSION) # available only since CMake 3.12
        set(Assimp_MSVC_VERSION vc${MSVC_TOOLSET_VERSION})
    elseif(MSVC_VERSION VERSION_LESS 1910)
        set(Assimp_MSVC_VERSION vc140)
    elseif(MSVC_VERSION VERSION_LESS 1920)
        set(Assimp_MSVC_VERSION vc141)
    elseif(MSVC_VERSION VERSION_LESS 1930)
        set(Assimp_MSVC_VERSION vc142)
    else()
        message(FATAL_ERROR "Unsupported MSVC version")
    endif()

    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(Assimp_LIBRARY_DIR "lib64")
    elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
        set(Assimp_LIBRARY_DIR "lib32")
    endif()

    find_library(Assimp_LIBRARY_RELEASE assimp-${Assimp_MSVC_VERSION}-mt.lib
        PATHS ${Assimp_LIBRARY_DIR}
        # For backwards compatibility
        HINTS ${ASSIMP_LIBRARY_RELEASE})
    find_library(Assimp_LIBRARY_DEBUG assimp-${Assimp_MSVC_VERSION}-mtd.lib
        PATHS ${Assimp_LIBRARY_DIR}
        # For backwards compatibility
        HINTS ${ASSIMP_LIBRARY_DEBUG})
else()
    find_library(Assimp_LIBRARY_RELEASE assimp)
    find_library(Assimp_LIBRARY_DEBUG assimpd)
endif()

# Static build of Assimp depends on IrrXML and zlib, find those as well. If not
# found, simply don't link to them --- it might be a dynamic build (on Windows
# it's a *.lib either way), or a static build using system IrrXML / zlib.
# Related issue: https://github.com/Microsoft/vcpkg/issues/5012
if(Assimp_LIBRARY_DEBUG MATCHES "${CMAKE_STATIC_LIBRARY_SUFFIX}$" OR Assimp_LIBRARY_RELEASE MATCHES "${CMAKE_STATIC_LIBRARY_SUFFIX}$")
    find_library(Assimp_IRRXML_LIBRARY_RELEASE IrrXML)
    find_library(Assimp_IRRXML_LIBRARY_DEBUG IrrXMLd)
    find_library(Assimp_ZLIB_LIBRARY_RELEASE zlibstatic)
    find_library(Assimp_ZLIB_LIBRARY_DEBUG zlibstaticd)
    mark_as_advanced(
        Assimp_IRRXML_LIBRARY_RELEASE
        Assimp_IRRXML_LIBRARY_DEBUG
        Assimp_ZLIB_LIBRARY_RELEASE
        Assimp_ZLIB_LIBRARY_DEBUG)
endif()

include(SelectLibraryConfigurations)
select_library_configurations(Assimp)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Assimp DEFAULT_MSG
    Assimp_LIBRARY
    Assimp_INCLUDE_DIR)

if(NOT TARGET Assimp::Assimp)
    add_library(Assimp::Assimp UNKNOWN IMPORTED)

    if(Assimp_LIBRARY_DEBUG)
        set_property(TARGET Assimp::Assimp APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
        set_target_properties(Assimp::Assimp PROPERTIES IMPORTED_LOCATION_DEBUG ${Assimp_LIBRARY_DEBUG})
    endif()
    if(Assimp_LIBRARY_RELEASE)
        set_property(TARGET Assimp::Assimp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
        set_target_properties(Assimp::Assimp PROPERTIES IMPORTED_LOCATION_RELEASE ${Assimp_LIBRARY_RELEASE})
    endif()

    # Link to IrrXML / zlib as well, if found. See the comment above for
    # details. Allow mixing up debug and release libraries because that's what
    # the IMPORTED_LOCATION does as well -- if building as Debug and a Debug
    # library is not available, it picks the Release one.
    foreach(_extra IRRXML ZLIB)
        if(Assimp_${_extra}_LIBRARY_RELEASE AND Assimp_${_extra}_LIBRARY_DEBUG)
            set_property(TARGET Assimp::Assimp APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES $<$<NOT:$<CONFIG:Debug>>:${Assimp_${_extra}_LIBRARY_RELEASE}>)
            set_property(TARGET Assimp::Assimp APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES $<$<CONFIG:Debug>:${Assimp_${_extra}_LIBRARY_DEBUG}>)
        elseif(Assimp_${_extra}_LIBRARY_DEBUG)
            set_property(TARGET Assimp::Assimp APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES ${Assimp_${_extra}_LIBRARY_DEBUG})
        elseif(Assimp_${_extra}_LIBRARY_RELEASE)
            set_property(TARGET Assimp::Assimp APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES ${Assimp_${_extra}_LIBRARY_RELEASE})
        endif()
    endforeach()

    set_target_properties(Assimp::Assimp PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES ${Assimp_INCLUDE_DIR})
endif()
