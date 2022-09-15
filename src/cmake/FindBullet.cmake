#.rst:
# Find Bullet
# -----------
#
# Finds the Bullet libraries. This module defines:
#
#  Bullet_FOUND         - True if Bullet is found
#  Bullet::Dynamics     - Bullet Dynamics imported target. Depends on
#   Bullet::Collision and Bullet::LinearMath.
#  Bullet::Collision    - Bullet Collision imported target. Depends on
#   Bullet::LinearMath.
#  Bullet::LinearMath   - Bullet Linear Math imported target
#  Bullet::SoftBody     - Bullet Soft Body imported target. Depends on
#   Bullet::Dynamics, Bullet::Collision and Bullet::LinearMath.
#
# Additionally these variables are defined for internal usage:
#
#  Bullet_Dynamics_LIBRARY_{DEBUG,RELEASE} - Bullet Dynamics library location
#  Bullet_Collision_LIBRARY_{DEBUG,RELEASE} - Bullet Collision library location
#  Bullet_LinearMath_LIBRARY_{DEBUG,RELEASE} - Bullet Linear Math library
#   location
#  Bullet_SoftBody_LIBRARY_{DEBUG,RELEASE} - Bullet SoftBody library location
#  Bullet_INCLUDE_DIR   - Include dir
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

# In case we have Bullet as a CMake subproject, BulletCollision should be
# defined (as opposed to Bullet3Common, which can be disabled with
# BUILD_BULLET3). If it's not, try to find the Bullet config file -- under
# Vcpkg at least, it defines imported targets that we can use. Vanilla Bullet
# also installs a config file, however that file is rather unhelpful since it
# doesn't expose any imported targets that I could rely on. And instead of
# extracting everything out of BULLET_LIBRARIES I'd rather just search for it
# myself.
# TODO: builtin support for Bullet from Emscripten Ports (-s USE_BULLET=1)
#
if(TARGET BulletCollision)
    # Double-check that we actually have Bullet as a CMake subproject by
    # checking its BUILD_BULLET3 option. If we do and we're building static
    # libs, Bullet's own targets don't express inter-library dependencies
    # correctly, causing linker errors on our side. If we're on CMake 3.13, we
    # can fix that from outside, otherwise unfortunately bad luck.
    # TODO: this will probably blow up in Vcpkg as well, the patch there should
    #   get expanded to remove the if(BUILD_SHARED_LIBS) around
    #   target_link_libraries(). Ideally, such patch should be pushed to Bullet
    #   upstream.
    get_target_property(_BULLET_LIBRARY_TYPE BulletCollision TYPE)
    if(DEFINED BUILD_BULLET3 AND _BULLET_LIBRARY_TYPE STREQUAL STATIC_LIBRARY)
        if(CMAKE_VERSION VERSION_LESS 3.13)
            message(SEND_ERROR "Statically-built Bullet as a CMake subproject requires CMake 3.13 in order to patch in correct inter-library dependencies and avoid linker errors. To continue, either upgrade CMake, enable BUILD_SHARED_LIBS to build Bullet as shared, or use an external Bullet installation.")
        else()
            cmake_policy(PUSH)
            cmake_policy(SET CMP0079 NEW)
            # Not using PUBLIC to avoid a potential conflict between an
            # all-plain and all-keyword signature.
            target_link_libraries(BulletCollision LinearMath)
            target_link_libraries(BulletDynamics BulletCollision)
            target_link_libraries(BulletSoftBody BulletDynamics)
            cmake_policy(POP)
        endif()
    endif()
else()
    find_package(Bullet CONFIG QUIET)
endif()

# Bullet's math library has a special name, so it has to be handled separately.
# Sigh. TODO: other libs such as Bullet3Common, Robotics, InverseDynamics?
set(_BULLET_SANE_LIBRARIES Dynamics Collision SoftBody)
set(_BULLET_LIBRARIES LinearMath ${_BULLET_SANE_LIBRARIES})

# We have a CMake subproject or a Vcpkg package, base our targets on those.
# That's all needed, so exit right after.
if(TARGET BulletCollision)
    foreach(_library ${_BULLET_SANE_LIBRARIES})
        if(NOT TARGET Bullet::${_library})
            # Aliases of (global) targets are only supported in CMake 3.11, so
            # we work around it by this. This is easier than fetching all
            # possible properties (which are impossible to track of) and then
            # attempting to rebuild them into a new target.
            add_library(Bullet::${_library} INTERFACE IMPORTED)
            set_target_properties(Bullet::${_library} PROPERTIES
                INTERFACE_LINK_LIBRARIES Bullet${_library})
        endif()
    endforeach()

    # Bullet3Common doesn't have an INTERFACE_INCLUDE_DIRECTORIES property as
    # bullet only uses include_directories(), not the target_*() variant. This
    # means that linking to any of the targets will not drag along any include
    # directory, which we have to fix -- and since everything depends on
    # LinearMath, we can add it just for that target.
    #
    # In case of a CMake subproject, we derive the include directory from the
    # target SOURCE_DIR.
    #
    # In case of Vcpkg, SOURCE_DIR is likely meaningless, but because the Vcpkg
    # package uses a patched config file, we can use BULLET_INCLUDE_DIR.
    if(BULLET_INCLUDE_DIR)
        set(_BULLET_INTERFACE_INCLUDE_DIRECTORIES ${BULLET_INCLUDE_DIR})
    else()
        get_target_property(_BULLET_INTERFACE_INCLUDE_DIRECTORIES BulletCollision SOURCE_DIR)
        get_filename_component(_BULLET_INTERFACE_INCLUDE_DIRECTORIES ${_BULLET_INTERFACE_INCLUDE_DIRECTORIES} DIRECTORY)
    endif()

    # Compile definitions, which is basically just USE_DOUBLE_PRECISION. If
    # Bullet was found externally, this is contained in the BULLET_DEFINITIONS
    # variable (which might be empty). If the variable isn't defined, it means
    # we have a Bullet subproject. OF COURSE this isn't propagated in
    # INTERFACE_COMPILE_DEFINITIONS, we can't expect any modicum of usability
    # there, so we have to fetch that from the CMake option instead.
    if(NOT DEFINED BULLET_DEFINITIONS AND USE_DOUBLE_PRECISION)
        set(BULLET_DEFINITIONS "-DBT_USE_DOUBLE_PRECISION")
    endif()

    # Why, Bullet, why this library has to have such a different name?
    if(NOT TARGET Bullet::LinearMath)
        # Aliases of (global) targets [..] CMake 3.11 [...], as above
        add_library(Bullet::LinearMath INTERFACE IMPORTED)
        set_target_properties(Bullet::LinearMath PROPERTIES
            INTERFACE_LINK_LIBRARIES LinearMath
            INTERFACE_INCLUDE_DIRECTORIES ${_BULLET_INTERFACE_INCLUDE_DIRECTORIES}
            # This might define BT_USE_DOUBLE_PRECISION, or not
            INTERFACE_COMPILE_OPTIONS "${BULLET_DEFINITIONS}")
    endif()

    # Just to make FPHSA print some meaningful location, nothing else. Luckily
    # we can just reuse what we had to find above.
    include(FindPackageHandleStandardArgs)
    find_package_handle_standard_args("Bullet" DEFAULT_MSG
        _BULLET_INTERFACE_INCLUDE_DIRECTORIES)

    return()
endif()

include(SelectLibraryConfigurations)

# The vanilla FindBullet.cmake in addition searches in lib/{Debug,Release} and
# out/{debug,release}8/libs. The former seems Windows-specific (see
# https://gitlab.kitware.com/cmake/cmake/-/commit/f180b24ef43d48fcec265656ee73ab9130fe39bd
# ) but I don't see such paths being used anymore -- the install on Windows
# just puts everything directly into lib/ and bin/. The weird path with 8 in it
# is there unchanged since the beginning (2009) and without any comment, I'll
# assume that's just obsolete.
foreach(_library ${_BULLET_SANE_LIBRARIES})
    find_library(Bullet_${_library}_LIBRARY_RELEASE NAMES Bullet${_library})
    find_library(Bullet_${_library}_LIBRARY_DEBUG
        NAMES
            # Vanilla Bullet adds the _Debug suffix to Debug libraries
            # https://github.com/bulletphysics/bullet3/blob/ad931b8c392d8dd5e4472121c9b5dc23a2efcec2/CMakeLists.txt#L206
            Bullet${_library}_Debug
            # ... however in many cases the packages override that to just _d
            # https://github.com/msys2/MINGW-packages/blob/master/mingw-w64-bullet/PKGBUILD
            Bullet${_library}_d)

    select_library_configurations(Bullet_${_library})
endforeach()
# Why, Bullet, why?
find_library(Bullet_LinearMath_LIBRARY_RELEASE NAMES LinearMath)
find_library(Bullet_LinearMath_LIBRARY_DEBUG
    NAMES
        LinearMath_Debug
        LinearMath_d)
select_library_configurations(Bullet_LinearMath)

# Include dir
find_path(Bullet_INCLUDE_DIR NAMES btBulletCollisionCommon.h
    PATH_SUFFIXES bullet)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Bullet DEFAULT_MSG
    # Those are the default set searched for by vanilla FindBullet, so assume
    # those are essential and everything else is optional(?)
    Bullet_Dynamics_LIBRARY
    Bullet_Collision_LIBRARY
    Bullet_LinearMath_LIBRARY
    Bullet_SoftBody_LIBRARY

    Bullet_INCLUDE_DIR)

mark_as_advanced(FORCE Bullet_INCLUDE_DIR)

foreach(_library ${_BULLET_LIBRARIES})
    # Usually other Find modules (such as FindSDL2) add an extra step for
    # handling macOS frameworks, but vanilla FindBullet doesn't so it's not
    # needed? http://public.kitware.com/pipermail/cmake/2016-April/063179.html
    # TODO: When extra optional libraries are added, this needs to check for
    # their presence as well
    if(NOT TARGET Bullet::${_library})
        add_library(Bullet::${_library} UNKNOWN IMPORTED)
        if(Bullet_${_library}_LIBRARY_RELEASE)
            set_property(TARGET Bullet::${_library} APPEND PROPERTY
                IMPORTED_CONFIGURATIONS RELEASE)
            set_target_properties(Bullet::${_library} PROPERTIES
                IMPORTED_LOCATION_RELEASE ${Bullet_${_library}_LIBRARY_RELEASE})
        endif()
        if(Bullet_${_library}_LIBRARY_DEBUG)
            set_property(TARGET Bullet::${_library} APPEND PROPERTY
                IMPORTED_CONFIGURATIONS DEBUG)
            set_target_properties(Bullet::${_library} PROPERTIES
                IMPORTED_LOCATION_DEBUG ${Bullet_${_library}_LIBRARY_DEBUG})
        endif()

        # Everything depends on LinearMath, so put the include dir as well as
        # compile definitions (such as BT_USE_DOUBLE_PRECISION) there
        if(_library STREQUAL LinearMath)
            set_property(TARGET Bullet::${_library} APPEND PROPERTY
                INTERFACE_INCLUDE_DIRECTORIES ${Bullet_INCLUDE_DIR})
            # BULLET_DEFINITIONS gets defined by BulletConfig from the
            # find_package() we did at first. That's also the only useful
            # thing from it, the rest we can find by hand as well.
            if(BULLET_DEFINITIONS)
                set_property(TARGET Bullet::${_library} APPEND PROPERTY
                    INTERFACE_COMPILE_OPTIONS "${BULLET_DEFINITIONS}")
            endif()

        # Collision depends on LinearMath
        elseif(_library STREQUAL Collision)
            set_property(TARGET Bullet::${_library} APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES Bullet::LinearMath)

        # Dynamics depends on Collision and LinearMath
        elseif(_library STREQUAL Dynamics)
            set_property(TARGET Bullet::${_library} APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES Bullet::Collision Bullet::LinearMath)

        # SoftBody depends on Dynamics, Collision and LinearMath (according to
        # ldd at least, not sure what's the real dependency chain)
        elseif(_library STREQUAL SoftBody)
            set_property(TARGET Bullet::${_library} APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES Bullet::Dynamics Bullet::Collision Bullet::LinearMath)

        # Sanity check in case we expand the library list
        else()
            message(FATAL_ERROR "Unhandled dependencies of Bullet::${_library}")
        endif()
    endif()
endforeach()
