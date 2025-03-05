#.rst:
# Find Magnum plugins
# -------------------
#
# Finds the Magnum plugins library. Basic usage::
#
#  find_package(MagnumPlugins REQUIRED)
#
# This command tries to find Magnum plugins and then defines the following:
#
#  MagnumPlugins_FOUND          - Whether Magnum plugins were found
#
# This command will not try to find any actual plugin. The plugins are:
#
#  AssimpImporter               - Assimp importer
#  AstcImporter                 - ASTC importer
#  BasisImageConverter          - Basis image converter
#  BasisImporter                - Basis importer
#  BcDecImageConverter          - BCn image decoder using bcdec
#  DdsImporter                  - DDS importer
#  DevIlImageImporter           - Image importer using DevIL
#  DrFlacAudioImporter          - FLAC audio importer using dr_flac
#  DrMp3AudioImporter           - MP3 audio importer using dr_mp3
#  DrWavAudioImporter           - WAV audio importer using dr_wav
#  EtcDecImageConverter         - ETC/EAC image decoder using etcdec
#  Faad2AudioImporter           - AAC audio importer using FAAD2
#  FreeTypeFont                 - FreeType font
#  GlslangShaderConverter       - Glslang shader converter
#  GltfImporter                 - glTF importer
#  GltfSceneConverter           - glTF converter
#  HarfBuzzFont                 - HarfBuzz font
#  IcoImporter                  - ICO importer
#  JpegImageConverter           - JPEG image converter
#  JpegImporter                 - JPEG importer
#  KtxImageConverter            - KTX image converter
#  KtxImporter                  - KTX importer
#  MeshOptimizerSceneConverter  - MeshOptimizer scene converter
#  MiniExrImageConverter        - OpenEXR image converter using miniexr
#  OpenGexImporter              - OpenGEX importer
#  PngImageConverter            - PNG image converter
#  PngImporter                  - PNG importer
#  PrimitiveImporter            - Primitive importer
#  SpirvToolsShaderConverter    - SPIR-V Tools shader converter
#  SpngImporter                 - PNG importer using libspng
#  StanfordImporter             - Stanford PLY importer
#  StanfordSceneConverter       - Stanford PLY converter
#  StbDxtImageConverter         - BC1/BC3 image compressor using stb_dxt
#  StbImageConverter            - Image converter using stb_image_write
#  StbImageImporter             - Image importer using stb_image
#  StbResizeImageConverter      - Image resizing using stb_image_resize
#  StbTrueTypeFont              - TrueType font using stb_truetype
#  StbVorbisAudioImporter       - OGG audio importer using stb_vorbis
#  StlImporter                  - STL importer
#  UfbxImporter                 - FBX and OBJ importer using ufbx
#  WebPImageConverter           - WebP image converter
#  WebPImporter                 - WebP importer
#
# If Magnum is built with MAGNUM_BUILD_DEPRECATED enabled, these additional
# plugins are available for backwards compatibility purposes:
#
#  CgltfImporter                - glTF importer using cgltf
#  TinyGltfImporter             - GLTF importer using tiny_gltf
#
# Some plugins expose their internal state through separate libraries. The
# libraries are:
#
#  OpenDdl                      - OpenDDL parser, used as a base for the
#   OpenGexImporter plugin
#
# Example usage with specifying the plugins is::
#
#  find_package(MagnumPlugins REQUIRED FreeTypeFont PngImporter)
#
# For each plugin is then defined:
#
#  MagnumPlugins_*_FOUND        - Whether the plugin was found
#  MagnumPlugins::*             - Plugin imported target
#
# The package is found if either debug or release version of each requested
# plugin is found. If both debug and release plugins are found, proper version
# is chosen based on actual build configuration of the project (i.e. ``Debug``
# build is linked to debug plugins, ``Release`` build to release plugins). See
# ``FindMagnum.cmake`` for more information about autodetection of
# ``MAGNUM_PLUGINS_DIR``.
#
# Additionally these variables are defined for internal usage:
#
#  MAGNUMPLUGINS_INCLUDE_DIR    - Magnum plugins include dir (w/o dependencies)
#  MAGNUMPLUGINS_*_LIBRARY      - Plugin library (w/o dependencies)
#  MAGNUMPLUGINS_*_LIBRARY_DEBUG - Debug version of given library, if found
#  MAGNUMPLUGINS_*_LIBRARY_RELEASE - Release version of given library, if found
#

#
#   This file is part of Magnum.
#
#   Copyright © 2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019,
#               2020, 2021, 2022, 2023, 2024, 2025
#             Vladimír Vondruš <mosra@centrum.cz>
#   Copyright © 2019 Jonathan Hale <squareys@googlemail.com>
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
set(_MAGNUMPLUGINS_DEPENDENCIES )
foreach(_component ${MagnumPlugins_FIND_COMPONENTS})
    if(_component MATCHES ".+AudioImporter$")
        set(_MAGNUMPLUGINS_${_component}_MAGNUM_DEPENDENCIES Audio)
    elseif(_component MATCHES ".+(Importer|ImageConverter|SceneConverter)")
        set(_MAGNUMPLUGINS_${_component}_MAGNUM_DEPENDENCIES Trade)
    elseif(_component MATCHES ".+(Font|FontConverter)$")
        set(_MAGNUMPLUGINS_${_component}_MAGNUM_DEPENDENCIES Text)
    endif()

    if(_component STREQUAL AssimpImporter)
        list(APPEND _MAGNUMPLUGINS_${_component}_MAGNUM_DEPENDENCIES AnyImageImporter)
    elseif(_component STREQUAL CgltfImporter)
        # TODO remove when the deprecated plugin is gone
        list(APPEND _MAGNUMPLUGINS_${_component}_MAGNUM_DEPENDENCIES AnyImageImporter)
    elseif(_component STREQUAL GltfImporter)
        list(APPEND _MAGNUMPLUGINS_${_component}_MAGNUM_DEPENDENCIES AnyImageImporter)
    elseif(_component STREQUAL MeshOptimizerSceneConverter)
        list(APPEND _MAGNUMPLUGINS_${_component}_MAGNUM_DEPENDENCIES MeshTools)
    elseif(_component STREQUAL OpenGexImporter)
        list(APPEND _MAGNUMPLUGINS_${_component}_MAGNUM_DEPENDENCIES AnyImageImporter)
    elseif(_component STREQUAL PrimitiveImporter)
        list(APPEND _MAGNUMPLUGINS_${_component}_MAGNUM_DEPENDENCIES Primitives)
    elseif(_component STREQUAL StanfordImporter)
        list(APPEND _MAGNUMPLUGINS_${_component}_MAGNUM_DEPENDENCIES MeshTools)
    elseif(_component STREQUAL StanfordSceneConverter)
        list(APPEND _MAGNUMPLUGINS_${_component}_MAGNUM_DEPENDENCIES MeshTools)
    elseif(_component STREQUAL UfbxImporter)
        list(APPEND _MAGNUMPLUGINS_${_component}_MAGNUM_DEPENDENCIES AnyImageImporter)
    elseif(_component STREQUAL TinyGltfImporter)
        # TODO remove when the deprecated plugin is gone
        list(APPEND _MAGNUMPLUGINS_${_component}_MAGNUM_DEPENDENCIES AnyImageImporter)
    endif()

    list(APPEND _MAGNUMPLUGINS_DEPENDENCIES ${_MAGNUMPLUGINS_${_component}_MAGNUM_DEPENDENCIES})
endforeach()
find_package(Magnum REQUIRED ${_MAGNUMPLUGINS_DEPENDENCIES})

# Global include dir that's unique to Magnum Plugins. Often they will be
# installed alongside Magnum, which is why the hint, but if not, it shouldn't
# just pick MAGNUM_INCLUDE_DIR because then _MAGNUMPLUGINS_*_INCLUDE_DIR will
# fail to be found. In case of CMake subprojects the versionPlugins.h is
# generated inside the build dir so this won't find it, instead
# src/CMakeLists.txt forcibly sets MAGNUMPLUGINS_INCLUDE_DIR as an internal
# cache value to make that work.
find_path(MAGNUMPLUGINS_INCLUDE_DIR Magnum/versionPlugins.h
    HINTS ${MAGNUM_INCLUDE_DIR})
mark_as_advanced(MAGNUMPLUGINS_INCLUDE_DIR)

# CMake module dir for dependencies. It might not be present at all if no
# feature that needs them is enabled, in which case it'll be left at NOTFOUND.
# But in that case it should also not be subsequently needed for any
# find_package(). If this is called from a superproject, the
# _MAGNUMPLUGINS_DEPENDENCY_MODULE_DIR is already set by
# modules/CMakeLists.txt.
find_path(_MAGNUMPLUGINS_DEPENDENCY_MODULE_DIR
    NAMES
        FindAssimp.cmake FindBasisUniversal.cmake FindDevIL.cmake
        FindFAAD2.cmake FindGlslang.cmake FindHarfBuzz.cmake
        FindOpenEXR.cmake FindSpirvTools.cmake FindSpng.cmake FindWebP.cmake
        FindZstd.cmake
    PATH_SUFFIXES share/cmake/MagnumPlugins/dependencies)
mark_as_advanced(_MAGNUMPLUGINS_DEPENDENCY_MODULE_DIR)

# If the module dir is found and is not present in CMAKE_MODULE_PATH already
# (such as when someone explicitly added it, or if it's the Magnum's modules/
# dir in case of a superproject), add it as the first before all other. Set a
# flag to remove it again at the end, so the modules don't clash with Find
# modules of the same name from other projects.
if(_MAGNUMPLUGINS_DEPENDENCY_MODULE_DIR AND NOT _MAGNUMPLUGINS_DEPENDENCY_MODULE_DIR IN_LIST CMAKE_MODULE_PATH)
    set(CMAKE_MODULE_PATH ${_MAGNUMPLUGINS_DEPENDENCY_MODULE_DIR} ${CMAKE_MODULE_PATH})
    set(_MAGNUMPLUGINS_REMOVE_DEPENDENCY_MODULE_DIR_FROM_CMAKE_PATH ON)
else()
    unset(_MAGNUMPLUGINS_REMOVE_DEPENDENCY_MODULE_DIR_FROM_CMAKE_PATH)
endif()

# Component distinction (listing them explicitly to avoid mistakes with finding
# components from other repositories)
set(_MAGNUMPLUGINS_LIBRARY_COMPONENTS OpenDdl)
set(_MAGNUMPLUGINS_PLUGIN_COMPONENTS
    AssimpImporter AstcImporter BasisImageConverter BasisImporter
    BcDecImageConverter DdsImporter DevIlImageImporter DrFlacAudioImporter
    DrMp3AudioImporter DrWavAudioImporter EtcDecImageConverter
    Faad2AudioImporter FreeTypeFont GlslangShaderConverter GltfImporter
    GltfSceneConverter HarfBuzzFont IcoImporter JpegImageConverter JpegImporter
    KtxImageConverter KtxImporter MeshOptimizerSceneConverter
    MiniExrImageConverter OpenExrImageConverter OpenExrImporter
    OpenGexImporter PngImageConverter PngImporter PrimitiveImporter
    SpirvToolsShaderConverter SpngImporter StanfordImporter
    StanfordSceneConverter StbDxtImageConverter StbImageConverter
    StbImageImporter StbResizeImageConverter StbTrueTypeFont
    StbVorbisAudioImporter StlImporter UfbxImporter WebPImageConverter
    WebPImporter)
# Nothing is enabled by default right now
set(_MAGNUMPLUGINS_IMPLICITLY_ENABLED_COMPONENTS )

# Inter-component dependencies
set(_MAGNUMPLUGINS_HarfBuzzFont_DEPENDENCIES FreeTypeFont)
set(_MAGNUMPLUGINS_OpenGexImporter_DEPENDENCIES OpenDdl)

# CgltfImporter and TinyGltfImporter, available only on a deprecated build
if(MAGNUM_BUILD_DEPRECATED)
    list(APPEND _MAGNUMPLUGINS_PLUGIN_COMPONENTS CgltfImporter TinyGltfImporter)
    set(_MAGNUMPLUGINS_CgltfImporter_DEPENDENCIES GltfImporter)
endif()

# Ensure that all inter-component dependencies are specified as well
set(_MAGNUMPLUGINS_ADDITIONAL_COMPONENTS )
foreach(_component ${MagnumPlugins_FIND_COMPONENTS})
    # Mark the dependencies as required if the component is also required
    if(MagnumPlugins_FIND_REQUIRED_${_component})
        foreach(_dependency ${_MAGNUMPLUGINS_${_component}_DEPENDENCIES})
            set(MagnumPlugins_FIND_REQUIRED_${_dependency} TRUE)
        endforeach()
    endif()

    list(APPEND _MAGNUMPLUGINS_ADDITIONAL_COMPONENTS ${_MAGNUMPLUGINS_${_component}_DEPENDENCIES})
endforeach()

# Join the lists, remove duplicate components
set(_MAGNUMPLUGINS_ORIGINAL_FIND_COMPONENTS ${MagnumPlugins_FIND_COMPONENTS})
if(_MAGNUMPLUGINS_ADDITIONAL_COMPONENTS)
    list(INSERT MagnumPlugins_FIND_COMPONENTS 0 ${_MAGNUMPLUGINS_ADDITIONAL_COMPONENTS})
endif()
if(MagnumPlugins_FIND_COMPONENTS)
    list(REMOVE_DUPLICATES MagnumPlugins_FIND_COMPONENTS)
endif()

# Special cases of include paths. Libraries not listed here have a path suffix
# and include name derived from the library name in the loop below. (So far no
# special cases.)

# Find all components
foreach(_component ${MagnumPlugins_FIND_COMPONENTS})
    string(TOUPPER ${_component} _COMPONENT)

    # Create imported target in case the library is found. If the project is
    # added as subproject to CMake, the target already exists and all the
    # required setup is already done from the build tree.
    if(TARGET "MagnumPlugins::${_component}") # Quotes to fix KDE's higlighter
        set(MagnumPlugins_${_component}_FOUND TRUE)
    else()
        # Find plugin/library includes. Each has a configure.h file so there
        # doesn't need to be any specialized per-library handling.
        if(_component IN_LIST _MAGNUMPLUGINS_PLUGIN_COMPONENTS OR _component IN_LIST _MAGNUMPLUGINS_LIBRARY_COMPONENTS)
            if(_component IN_LIST _MAGNUMPLUGINS_LIBRARY_COMPONENTS)
                set(_include_path_directory Magnum)
            else()
                set(_include_path_directory MagnumPlugins)
            endif()

            find_file(_MAGNUMPLUGINS_${_COMPONENT}_CONFIGURE_FILE configure.h
                HINTS ${MAGNUMPLUGINS_INCLUDE_DIR}/${_include_path_directory}/${_component})
            mark_as_advanced(_MAGNUMPLUGINS_${_COMPONENT}_CONFIGURE_FILE)

            # Determine if the plugin/library is static or dynamic by reading
            # the per-library config file. Plugins use this for automatic
            # import if static, libraries for finding a DLL location if shared.
            # If the file wasn't found, skip this so it fails on the FPHSA
            # below and not right here.
            if(_MAGNUMPLUGINS_${_COMPONENT}_CONFIGURE_FILE)
                file(READ ${_MAGNUMPLUGINS_${_COMPONENT}_CONFIGURE_FILE} _magnumPluginsConfigure)
                string(REGEX REPLACE ";" "\\\\;" _magnumPluginsConfigure "${_magnumPluginsConfigure}")
                string(REGEX REPLACE "\n" ";" _magnumPluginsConfigure "${_magnumPluginsConfigure}")
                list(FIND _magnumPluginsConfigure "#define MAGNUM_${_COMPONENT}_BUILD_STATIC" _magnumPluginsBuildStatic)
                if(NOT _magnumPluginsBuildStatic EQUAL -1)
                    # The variable is inconsistently named between C++ and
                    # CMake, so keep it underscored / private
                    set(_MAGNUMPLUGINS_${_COMPONENT}_BUILD_STATIC ON)
                endif()
            endif()
        endif()

        # Library components
        if(_component IN_LIST _MAGNUMPLUGINS_LIBRARY_COMPONENTS)
            # Try to find both debug and release version
            find_library(MAGNUMPLUGINS_${_COMPONENT}_LIBRARY_DEBUG Magnum${_component}-d)
            find_library(MAGNUMPLUGINS_${_COMPONENT}_LIBRARY_RELEASE Magnum${_component})
            mark_as_advanced(MAGNUMPLUGINS_${_COMPONENT}_LIBRARY_DEBUG
                MAGNUMPLUGINS_${_COMPONENT}_LIBRARY_RELEASE)

            # On Windows, if we have a dynamic build of given library, find the
            # DLLs as well. Abuse find_program() since the DLLs should be
            # alongside usual executables. On MinGW they however have a lib
            # prefix.
            if(CORRADE_TARGET_WINDOWS AND NOT _MAGNUMPLUGINS_${_COMPONENT}_BUILD_STATIC)
                find_program(MAGNUMPLUGINS_${_COMPONENT}_DLL_DEBUG ${CMAKE_SHARED_LIBRARY_PREFIX}Magnum${_component}-d.dll)
                find_program(MAGNUMPLUGINS_${_COMPONENT}_DLL_RELEASE ${CMAKE_SHARED_LIBRARY_PREFIX}Magnum${_component}.dll)
                mark_as_advanced(MAGNUMPLUGINS_${_COMPONENT}_DLL_DEBUG
                    MAGNUMPLUGINS_${_COMPONENT}_DLL_RELEASE)
            # If not on Windows or on a static build, unset the DLL variables
            # to avoid leaks when switching shared and static builds
            else()
                unset(MAGNUMPLUGINS_${_COMPONENT}_DLL_DEBUG CACHE)
                unset(MAGNUMPLUGINS_${_COMPONENT}_DLL_RELEASE CACHE)
            endif()

        # Plugin components
        elseif(_component IN_LIST _MAGNUMPLUGINS_PLUGIN_COMPONENTS)
            # AudioImporter plugin specific name suffixes
            if(_component MATCHES ".+AudioImporter$")
                set(_MAGNUMPLUGINS_${_COMPONENT}_PATH_SUFFIX audioimporters)

            # Importer plugin specific name suffixes
            elseif(_component MATCHES ".+Importer$")
                set(_MAGNUMPLUGINS_${_COMPONENT}_PATH_SUFFIX importers)

            # Font plugin specific name suffixes
            elseif(_component MATCHES ".+Font$")
                set(_MAGNUMPLUGINS_${_COMPONENT}_PATH_SUFFIX fonts)

            # ImageConverter plugin specific name suffixes
            elseif(_component MATCHES ".+ImageConverter$")
                set(_MAGNUMPLUGINS_${_COMPONENT}_PATH_SUFFIX imageconverters)

            # SceneConverter plugin specific name suffixes
            elseif(_component MATCHES ".+SceneConverter$")
                set(_MAGNUMPLUGINS_${_COMPONENT}_PATH_SUFFIX sceneconverters)

            # FontConverter plugin specific name suffixes
            elseif(_component MATCHES ".+FontConverter$")
                set(_MAGNUMPLUGINS_${_COMPONENT}_PATH_SUFFIX fontconverters)
            endif()

            # Dynamic plugins don't have any prefix (e.g. `lib` on Linux),
            # search with empty prefix and then reset that back so we don't
            # accidentally break something else
            set(_tmp_prefixes "${CMAKE_FIND_LIBRARY_PREFIXES}")
            set(CMAKE_FIND_LIBRARY_PREFIXES "${CMAKE_FIND_LIBRARY_PREFIXES};")

            # Try to find both debug and release version. Dynamic and static
            # debug libraries are in different places. Static debug plugins are
            # in magnum/ with a -d suffix while dynamic debug plugins are in
            # magnum-d/ with no suffix. Problem is that Vcpkg's library linking
            # automagic needs the static libs to be in the root library
            # directory along with everything else and so we need to search for
            # the -d suffixed version *before* the unsuffixed so it doesn't
            # pick the release library for both debug and release.
            find_library(MAGNUMPLUGINS_${_COMPONENT}_LIBRARY_DEBUG ${_component}-d
                PATH_SUFFIXES magnum/${_MAGNUMPLUGINS_${_COMPONENT}_PATH_SUFFIX})
            find_library(MAGNUMPLUGINS_${_COMPONENT}_LIBRARY_DEBUG ${_component}
                PATH_SUFFIXES magnum-d/${_MAGNUMPLUGINS_${_COMPONENT}_PATH_SUFFIX})
            find_library(MAGNUMPLUGINS_${_COMPONENT}_LIBRARY_RELEASE ${_component}
                PATH_SUFFIXES magnum/${_MAGNUMPLUGINS_${_COMPONENT}_PATH_SUFFIX})
            mark_as_advanced(MAGNUMPLUGINS_${_COMPONENT}_LIBRARY_DEBUG
                MAGNUMPLUGINS_${_COMPONENT}_LIBRARY_RELEASE)

            # Reset back
            set(CMAKE_FIND_LIBRARY_PREFIXES "${_tmp_prefixes}")

        # Something unknown, skip. FPHSA will take care of handling this below.
        else()
            continue()
        endif()

        # Decide if the plugin/library was found. If not, skip the rest, which
        # populates the target properties and finds additional dependencies.
        # This means that the rest can also rely on that e.g. FindZstd.cmake is
        # present in _MAGNUMPLUGINS_DEPENDENCY_MODULE_DIR -- given that the
        # library needing Zstd was found, it likely also installed FindZstd for
        # itself.
        if(
            # If the component is a library or a plugin, it should have the
            # configure file
            (_component IN_LIST _MAGNUMPLUGINS_PLUGIN_COMPONENTS OR _component IN_LIST _MAGNUMPLUGINS_LIBRARY_COMPONENTS) AND _MAGNUMPLUGINS_${_COMPONENT}_CONFIGURE_FILE AND (
                # And it should have a debug library, and a DLL found if
                # expected
                (MAGNUMPLUGINS_${_COMPONENT}_LIBRARY_DEBUG AND (
                    NOT DEFINED MAGNUMPLUGINS_${_COMPONENT}_DLL_DEBUG OR
                    MAGNUMPLUGINS_${_COMPONENT}_DLL_DEBUG)) OR
                # Or have a release library, and a DLL found if expected
                (MAGNUMPLUGINS_${_COMPONENT}_LIBRARY_RELEASE AND (
                    NOT DEFINED MAGNUMPLUGINS_${_COMPONENT}_DLL_RELEASE OR
                    MAGNUMPLUGINS_${_COMPONENT}_DLL_RELEASE)))
        )
            set(MagnumPlugins_${_component}_FOUND TRUE)
        else()
            set(MagnumPlugins_${_component}_FOUND FALSE)
            continue()
        endif()

        # Target and location for libraries
        if(_component IN_LIST _MAGNUMPLUGINS_LIBRARY_COMPONENTS)
            if(_MAGNUMPLUGINS_${_COMPONENT}_BUILD_STATIC)
                add_library(MagnumPlugins::${_component} STATIC IMPORTED)
            else()
                add_library(MagnumPlugins::${_component} SHARED IMPORTED)
            endif()

            foreach(_CONFIG DEBUG RELEASE)
                if(NOT MAGNUMPLUGINS_${_COMPONENT}_LIBRARY_${_CONFIG})
                    continue()
                endif()

                set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                    IMPORTED_CONFIGURATIONS ${_CONFIG})
                # Unfortunately for a DLL the two properties are swapped out,
                # *.lib goes to IMPLIB, so it's duplicated like this
                if(DEFINED MAGNUMPLUGINS_${_COMPONENT}_DLL_${_CONFIG})
                    # Quotes to "fix" KDE's higlighter
                    set_target_properties("MagnumPlugins::${_component}" PROPERTIES
                        IMPORTED_LOCATION_${_CONFIG} ${MAGNUMPLUGINS_${_COMPONENT}_DLL_${_CONFIG}}
                        IMPORTED_IMPLIB_${_CONFIG} ${MAGNUMPLUGINS_${_COMPONENT}_LIBRARY_${_CONFIG}})
                else()
                    set_property(TARGET MagnumPlugins::${_component} PROPERTY
                        IMPORTED_LOCATION_${_CONFIG} ${MAGNUMPLUGINS_${_COMPONENT}_LIBRARY_${_CONFIG}})
                endif()
            endforeach()

        # Target and location for plugins. Not dealing with DLL locations for
        # those.
        elseif(_component IN_LIST _MAGNUMPLUGINS_PLUGIN_COMPONENTS)
            add_library(MagnumPlugins::${_component} UNKNOWN IMPORTED)

            foreach(_CONFIG DEBUG RELEASE)
                if(NOT MAGNUMPLUGINS_${_COMPONENT}_LIBRARY_${_CONFIG})
                    continue()
                endif()

                set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                    IMPORTED_CONFIGURATIONS ${_CONFIG})
                set_property(TARGET MagnumPlugins::${_component} PROPERTY
                    IMPORTED_LOCATION_${_CONFIG} ${MAGNUMPLUGINS_${_COMPONENT}_LIBRARY_${_CONFIG}})
            endforeach()
        endif()

        # AssimpImporter plugin dependencies
        if(_component STREQUAL AssimpImporter)
            find_package(Assimp)
            set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES Assimp::Assimp)

        # AstcImporter has no dependencies

        # BasisImageConverter / BasisImporter has only compiled-in
        # dependencies, except in case of vcpkg, then we need to link to a
        # library. Use a similar logic as in FindBasisUniversal, so in case an
        # user wants to disable this, they can point BASIS_UNIVERSAL_DIR to
        # something else (or just anything, because in that case it'll be a
        # no-op.
        elseif(_component STREQUAL BasisImageConverter)
            find_package(basisu CONFIG QUIET)
            if(basisu_FOUND AND NOT BASIS_UNIVERSAL_DIR)
                set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                    INTERFACE_LINK_LIBRARIES basisu_encoder)
            else()
                # Our own build may depend on Zstd, as we replace the bundled
                # files with an external library. Include it if present,
                # otherwise assume it's compiled without.
                find_package(Zstd)
                if(Zstd_FOUND)
                    set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                        INTERFACE_LINK_LIBRARIES Zstd::Zstd)
                endif()
            endif()
        elseif(_component STREQUAL BasisImporter)
            find_package(basisu CONFIG QUIET)
            if(basisu_FOUND AND NOT BASIS_UNIVERSAL_DIR)
                set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                    INTERFACE_LINK_LIBRARIES basisu_transcoder)
            else()
                # Our own build may depend on Zstd, as we replace the bundled
                # files with an external library. Include it if present,
                # otherwise assume it's compiled without.
                find_package(Zstd)
                if(Zstd_FOUND)
                    set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                        INTERFACE_LINK_LIBRARIES Zstd::Zstd)
                endif()
            endif()

        # BcDecImageConverter has no dependencies
        # CgltfImporter has no dependencies
        # DdsImporter has no dependencies

        # DevIlImageImporter plugin dependencies
        elseif(_component STREQUAL DevIlImageImporter)
            find_package(DevIL)
            set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES ${IL_LIBRARIES} ${ILU_LIBRARIES})

        # DrFlacAudioImporter has no dependencies
        # DrMp3AudioImporter has no dependencies
        # DrWavAudioImporter has no dependencies
        # EtcDecImageConverter has no dependencies

        # Faad2AudioImporter plugin dependencies
        elseif(_component STREQUAL Faad2AudioImporter)
            find_package(FAAD2)
            set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES FAAD2::FAAD2)

        # FreeTypeFont plugin dependencies
        elseif(_component STREQUAL FreeTypeFont)
            find_package(Freetype)
            # Need to handle special cases where both debug and release
            # libraries are available (in form of debug;A;optimized;B in
            # FREETYPE_LIBRARIES), thus appending them one by one
            if(FREETYPE_LIBRARY_DEBUG AND FREETYPE_LIBRARY_RELEASE)
                set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                    INTERFACE_LINK_LIBRARIES "$<$<NOT:$<CONFIG:Debug>>:${FREETYPE_LIBRARY_RELEASE}>;$<$<CONFIG:Debug>:${FREETYPE_LIBRARY_DEBUG}>")
            else()
                set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                    INTERFACE_LINK_LIBRARIES ${FREETYPE_LIBRARIES})
            endif()

        # GlslangShaderConverter plugin dependencies
        elseif(_component STREQUAL GlslangShaderConverter)
            find_package(Glslang REQUIRED)
            set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES Glslang::Glslang)

        # GltfImporter has no dependencies
        # GltfSceneConverter has no dependencies

        # HarfBuzzFont plugin dependencies
        elseif(_component STREQUAL HarfBuzzFont)
            find_package(Freetype)
            find_package(HarfBuzz)
            # Need to handle special cases where both debug and release
            # libraries are available (in form of debug;A;optimized;B in
            # FREETYPE_LIBRARIES), thus appending them one by one
            if(FREETYPE_LIBRARY_DEBUG AND FREETYPE_LIBRARY_RELEASE)
                set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                    INTERFACE_LINK_LIBRARIES "$<$<NOT:$<CONFIG:Debug>>:${FREETYPE_LIBRARY_RELEASE}>;$<$<CONFIG:Debug>:${FREETYPE_LIBRARY_DEBUG}>")
            else()
                set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                    INTERFACE_LINK_LIBRARIES ${FREETYPE_LIBRARIES})
            endif()
            set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES HarfBuzz::HarfBuzz)

        # IcoImporter has no dependencies

        # JpegImporter / JpegImageConverter plugin dependencies
        elseif(_component STREQUAL JpegImageConverter OR _component STREQUAL JpegImporter)
            find_package(JPEG)
            # Need to handle special cases where both debug and release
            # libraries are available (in form of debug;A;optimized;B in
            # JPEG_LIBRARIES), thus appending them one by one
            if(JPEG_LIBRARY_DEBUG AND JPEG_LIBRARY_RELEASE)
                set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                    INTERFACE_LINK_LIBRARIES "$<$<NOT:$<CONFIG:Debug>>:${JPEG_LIBRARY_RELEASE}>;$<$<CONFIG:Debug>:${JPEG_LIBRARY_DEBUG}>")
            else()
                set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                    INTERFACE_LINK_LIBRARIES ${JPEG_LIBRARIES})
            endif()

        # KtxImageConverter has no dependencies
        # KtxImporter has no dependencies

        # MeshOptimizerSceneConverter plugin dependencies
        elseif(_component STREQUAL MeshOptimizerSceneConverter)
            if(NOT TARGET meshoptimizer)
                find_package(meshoptimizer REQUIRED CONFIG)
                set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                    INTERFACE_LINK_LIBRARIES meshoptimizer::meshoptimizer)
            else()
                set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                    INTERFACE_LINK_LIBRARIES meshoptimizer)
            endif()

        # MiniExrImageConverter has no dependencies

        # OpenExrImporter / OpenExrImageConverter plugin dependencies
        elseif(_component STREQUAL OpenExrImporter OR _component STREQUAL OpenExrImageConverter)
            # Force our own FindOpenEXR module, which then delegates to the
            # config if appropriate
            find_package(OpenEXR REQUIRED MODULE)
            set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES OpenEXR::OpenEXR)
            # OpenEXR uses exceptions, which need an explicit flag on
            # Emscripten. This is most likely not propagated through its CMake
            # config file, so doing that explicitly here.
            if(CORRADE_TARGET_EMSCRIPTEN)
                if(CMAKE_VERSION VERSION_LESS 3.13)
                    message(FATAL_ERROR "CMake 3.13+ is required in order to specify Emscripten linker options")
                endif()
                set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                    INTERFACE_LINK_OPTIONS "SHELL:-s DISABLE_EXCEPTION_CATCHING=0")
            endif()

        # No special setup for the OpenDdl library
        # OpenGexImporter has no dependencies

        # PngImageConverter / PngImporter plugin dependencies
        elseif(_component STREQUAL PngImageConverter OR _component STREQUAL PngImporter)
            find_package(PNG)
            # Need to handle special cases where both debug and release
            # libraries are available (in form of debug;A;optimized;B in
            # PNG_LIBRARIES), thus appending them one by one. Imported target
            # that would make this obsolete is unfortunately only since CMake
            # 3.5. We need to link to zlib explicitly in this case as well
            # (whereas PNG_LIBRARIES contains that already), fortunately zlib
            # has an imported target in 3.4 already.
            if(PNG_LIBRARY_DEBUG AND PNG_LIBRARY_RELEASE)
                set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                    INTERFACE_LINK_LIBRARIES "$<$<NOT:$<CONFIG:Debug>>:${PNG_LIBRARY_RELEASE}>;$<$<CONFIG:Debug>:${PNG_LIBRARY_DEBUG}>" ZLIB::ZLIB)
            else()
                set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                    INTERFACE_LINK_LIBRARIES ${PNG_LIBRARIES})
            endif()

        # PrimitiveImporter has no dependencies

        # SpirvToolsShaderConverter plugin dependencies
        elseif(_component STREQUAL SpirvToolsShaderConverter)
            find_package(SpirvTools REQUIRED)
            set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES SpirvTools::SpirvTools SpirvTools::Opt)

        # SpngImporter plugin dependencies
        elseif(_component STREQUAL SpngImporter)
            find_package(Spng REQUIRED)
            set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES Spng::Spng)

        # StanfordImporter has no dependencies
        # StanfordSceneConverter has no dependencies
        # StbDxtImageConverter has no dependencies
        # StbImageConverter has no dependencies

        # StbImageImporter plugin dependencies
        elseif(_component STREQUAL StbImageImporter)
            # To solve a LTO-specific linker error. See StbImageImporter's
            # CMakeLists.txt for details.
            if(CORRADE_TARGET_EMSCRIPTEN AND NOT EMSCRIPTEN_VERSION VERSION_LESS 3.1.42 AND EMSCRIPTEN_VERSION VERSION_LESS 3.1.46)
                if(CMAKE_VERSION VERSION_LESS 3.13)
                    message(FATAL_ERROR "CMake 3.13+ is required in order to specify Emscripten linker options")
                endif()
                target_link_options(MagnumPlugins::${_component} INTERFACE $<$<CONFIG:Release>:-Wl,-u,scalbnf>)
            endif()

        # StbResizeImageConverter has no dependencies
        # StbTrueTypeFont has no dependencies

        # StbVorbisAudioImporter plugin dependencies
        elseif(_component STREQUAL StbVorbisAudioImporter)
            # To solve a LTO-specific linker error. See StbVorbisAudioImporter's
            # CMakeLists.txt for details.
            if(CORRADE_TARGET_EMSCRIPTEN AND NOT EMSCRIPTEN_VERSION VERSION_LESS 3.1.42 AND EMSCRIPTEN_VERSION VERSION_LESS 3.1.46)
                if(CMAKE_VERSION VERSION_LESS 3.13)
                    message(FATAL_ERROR "CMake 3.13+ is required in order to specify Emscripten linker options")
                endif()
                target_link_options(MagnumPlugins::${_component} INTERFACE $<$<CONFIG:Release>:-Wl,-u,scalbnf>)
            endif()

        # StlImporter has no dependencies
        # UfbxImporter has no dependencies
        # TinyGltfImporter has no dependencies

        # WebPImageConverter / WebPImporter plugin dependencies
        elseif(_component STREQUAL WebPImageConverter OR _component STREQUAL WebPImporter)
            find_package(WebP REQUIRED)
            set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES WebP::WebP)

        endif()

        # Automatic import of static plugins
        if(_component IN_LIST _MAGNUMPLUGINS_PLUGIN_COMPONENTS AND _MAGNUMPLUGINS_${_COMPONENT}_BUILD_STATIC)
            get_filename_component(_MAGNUMPLUGINS_${_COMPONENT}_INCLUDE_DIR ${_MAGNUMPLUGINS_${_COMPONENT}_CONFIGURE_FILE} DIRECTORY)
            set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                INTERFACE_SOURCES ${_MAGNUMPLUGINS_${_COMPONENT}_INCLUDE_DIR}/importStaticPlugin.cpp)
        endif()

        if(_component IN_LIST _MAGNUMPLUGINS_PLUGIN_COMPONENTS OR _component IN_LIST _MAGNUMPLUGINS_LIBRARY_COMPONENTS)
            # Link to core Magnum library, add other Magnum dependencies
            set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES Magnum::Magnum)
            foreach(_dependency ${_MAGNUMPLUGINS_${_component}_MAGNUM_DEPENDENCIES})
                set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                    INTERFACE_LINK_LIBRARIES Magnum::${_dependency})
            endforeach()

            # Add inter-project dependencies
            foreach(_dependency ${_MAGNUMPLUGINS_${_component}_DEPENDENCIES})
                set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                    INTERFACE_LINK_LIBRARIES MagnumPlugins::${_dependency})
            endforeach()
        endif()
    endif()
endforeach()

# For CMake 3.16+ with REASON_FAILURE_MESSAGE, provide additional potentially
# useful info about the failed components.
if(NOT CMAKE_VERSION VERSION_LESS 3.16)
    set(_MAGNUMPLUGINS_REASON_FAILURE_MESSAGE)
    # Go only through the originally specified find_package() components, not
    # the dependencies added by us afterwards
    foreach(_component ${_MAGNUMPLUGINS_ORIGINAL_FIND_COMPONENTS})
        if(MagnumPlugins_${_component}_FOUND)
            continue()
        endif()

        # If it's not known at all, tell the user -- it might be a new library
        # and an old Find module, or something platform-specific.
        if(NOT _component IN_LIST _MAGNUMPLUGINS_LIBRARY_COMPONENTS AND NOT _component IN_LIST _MAGNUMPLUGINS_PLUGIN_COMPONENTS)
            list(APPEND _MAGNUMPLUGINS_REASON_FAILURE_MESSAGE "${_component} is not a known component on this platform.")
        # Otherwise, if it's not among implicitly built components, hint that
        # the user may need to enable it
        # TODO: currently, the _FOUND variable doesn't reflect if dependencies
        #   were found. When it will, this needs to be updated to avoid
        #   misleading messages.
        elseif(NOT _component IN_LIST _MAGNUMPLUGINS_IMPLICITLY_ENABLED_COMPONENTS)
            string(TOUPPER ${_component} _COMPONENT)
            list(APPEND _MAGNUMPLUGINS_REASON_FAILURE_MESSAGE "${_component} is not built by default. Make sure you enabled MAGNUM_WITH_${_COMPONENT} when building Magnum Plugins.")
        # Otherwise we have no idea. Better be silent than to print something
        # misleading.
        else()
        endif()
    endforeach()

    string(REPLACE ";" " " _MAGNUMPLUGINS_REASON_FAILURE_MESSAGE "${_MAGNUMPLUGINS_REASON_FAILURE_MESSAGE}")
    set(_MAGNUMPLUGINS_REASON_FAILURE_MESSAGE REASON_FAILURE_MESSAGE "${_MAGNUMPLUGINS_REASON_FAILURE_MESSAGE}")
endif()

# Remove Magnum Plugins dependency module dir from CMAKE_MODULE_PATH again. Do
# it before the FPHSA call which may exit early in case of a failure.
if(_MAGNUMPLUGINS_REMOVE_DEPENDENCY_MODULE_DIR_FROM_CMAKE_PATH)
    list(REMOVE_ITEM CMAKE_MODULE_PATH ${_MAGNUMPLUGINS_DEPENDENCY_MODULE_DIR})
    unset(_MAGNUMPLUGINS_REMOVE_DEPENDENCY_MODULE_DIR_FROM_CMAKE_PATH)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MagnumPlugins
    REQUIRED_VARS MAGNUMPLUGINS_INCLUDE_DIR
    HANDLE_COMPONENTS
    ${_MAGNUMPLUGINS_REASON_FAILURE_MESSAGE})
