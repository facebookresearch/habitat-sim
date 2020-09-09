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
#  BasisImageConverter          - Basis image converter
#  BasisImporter                - Basis importer
#  DdsImporter                  - DDS importer
#  DevIlImageImporter           - Image importer using DevIL
#  DrFlacAudioImporter          - FLAC audio importer using dr_flac
#  DrMp3AudioImporter           - MP3 audio importer using dr_mp3
#  DrWavAudioImporter           - WAV audio importer using dr_wav
#  Faad2AudioImporter           - AAC audio importer using FAAD2
#  FreeTypeFont                 - FreeType font
#  HarfBuzzFont                 - HarfBuzz font
#  IcoImporter                  - ICO importer
#  JpegImageConverter           - JPEG image converter
#  JpegImporter                 - JPEG importer
#  MeshOptimizerSceneConverter  - MeshOptimizer scene converter
#  MiniExrImageConverter        - OpenEXR image converter using miniexr
#  OpenGexImporter              - OpenGEX importer
#  PngImageConverter            - PNG image converter
#  PngImporter                  - PNG importer
#  PrimitiveImporter            - Primitive importer
#  StanfordImporter             - Stanford PLY importer
#  StanfordSceneConverter       - Stanford PLY converter
#  StbImageConverter            - Image converter using stb_image_write
#  StbImageImporter             - Image importer using stb_image
#  StbTrueTypeFont              - TrueType font using stb_truetype
#  StbVorbisAudioImporter       - OGG audio importer using stb_vorbis
#  StlImporter                  - STL importer
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
#               2020 Vladimír Vondruš <mosra@centrum.cz>
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
    elseif(_component STREQUAL TinyGltfImporter)
        list(APPEND _MAGNUMPLUGINS_${_component}_MAGNUM_DEPENDENCIES AnyImageImporter)
    endif()

    list(APPEND _MAGNUMPLUGINS_DEPENDENCIES ${_MAGNUMPLUGINS_${_component}_MAGNUM_DEPENDENCIES})
endforeach()
find_package(Magnum REQUIRED ${_MAGNUMPLUGINS_DEPENDENCIES})

# Global plugin include dir
find_path(MAGNUMPLUGINS_INCLUDE_DIR MagnumPlugins
    HINTS ${MAGNUM_INCLUDE_DIR})
mark_as_advanced(MAGNUMPLUGINS_INCLUDE_DIR)

# Component distinction (listing them explicitly to avoid mistakes with finding
# components from other repositories)
set(_MAGNUMPLUGINS_LIBRARY_COMPONENT_LIST OpenDdl)
set(_MAGNUMPLUGINS_PLUGIN_COMPONENT_LIST
    AssimpImporter BasisImageConverter BasisImporter DdsImporter
    DevIlImageImporter DrFlacAudioImporter DrMp3AudioImporter
    DrWavAudioImporter Faad2AudioImporter FreeTypeFont HarfBuzzFont IcoImporter
    JpegImageConverter JpegImporter MeshOptimizerSceneConverter
    MiniExrImageConverter OpenGexImporter PngImageConverter PngImporter
    PrimitiveImporter StanfordImporter StanfordSceneConverter StbImageConverter
    StbImageImporter StbTrueTypeFont StbVorbisAudioImporter StlImporter
    TinyGltfImporter)

# Inter-component dependencies
set(_MAGNUMPLUGINS_HarfBuzzFont_DEPENDENCIES FreeTypeFont)
set(_MAGNUMPLUGINS_OpenGexImporter_DEPENDENCIES OpenDdl)

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
if(_MAGNUMPLUGINS_ADDITIONAL_COMPONENTS)
    list(INSERT MagnumPlugins_FIND_COMPONENTS 0 ${_MAGNUMPLUGINS_ADDITIONAL_COMPONENTS})
endif()
if(MagnumPlugins_FIND_COMPONENTS)
    list(REMOVE_DUPLICATES MagnumPlugins_FIND_COMPONENTS)
endif()

# Convert components lists to regular expressions so I can use if(MATCHES).
# TODO: Drop this once CMake 3.3 and if(IN_LIST) can be used
foreach(_WHAT LIBRARY PLUGIN)
    string(REPLACE ";" "|" _MAGNUMPLUGINS_${_WHAT}_COMPONENTS "${_MAGNUMPLUGINS_${_WHAT}_COMPONENT_LIST}")
    set(_MAGNUMPLUGINS_${_WHAT}_COMPONENTS "^(${_MAGNUMPLUGINS_${_WHAT}_COMPONENTS})$")
endforeach()

# Find all components
foreach(_component ${MagnumPlugins_FIND_COMPONENTS})
    string(TOUPPER ${_component} _COMPONENT)

    # Create imported target in case the library is found. If the project is
    # added as subproject to CMake, the target already exists and all the
    # required setup is already done from the build tree.
    if(TARGET MagnumPlugins::${_component})
        set(MagnumPlugins_${_component}_FOUND TRUE)
    else()
        # Library components
        if(_component MATCHES ${_MAGNUMPLUGINS_LIBRARY_COMPONENTS})
            add_library(MagnumPlugins::${_component} UNKNOWN IMPORTED)

            # Set library defaults, find the library
            set(_MAGNUMPLUGINS_${_COMPONENT}_INCLUDE_PATH_SUFFIX Magnum/${_component})
            set(_MAGNUMPLUGINS_${_COMPONENT}_INCLUDE_PATH_NAMES ${_component}.h)

            # Try to find both debug and release version
            find_library(MAGNUMPLUGINS_${_COMPONENT}_LIBRARY_DEBUG Magnum${_component}-d)
            find_library(MAGNUMPLUGINS_${_COMPONENT}_LIBRARY_RELEASE Magnum${_component})
            mark_as_advanced(MAGNUMPLUGINS_${_COMPONENT}_LIBRARY_DEBUG
                MAGNUMPLUGINS_${_COMPONENT}_LIBRARY_RELEASE)
        endif()

        # Plugin components
        if(_component MATCHES ${_MAGNUMPLUGINS_PLUGIN_COMPONENTS})
            add_library(MagnumPlugins::${_component} UNKNOWN IMPORTED)

            # AudioImporter plugin specific name suffixes
            if(_component MATCHES ".+AudioImporter$")
                set(_MAGNUMPLUGINS_${_COMPONENT}_PATH_SUFFIX audioimporters)

                # Audio importer class is Audio::*Importer, thus we need to
                # convert *AudioImporter.h to *Importer.h
                string(REPLACE "AudioImporter" "Importer" _MAGNUMPLUGINS_${_COMPONENT}_HEADER_NAME "${_component}")
                set(_MAGNUMPLUGINS_${_COMPONENT}_INCLUDE_PATH_NAMES ${_MAGNUMPLUGINS_${_COMPONENT}_HEADER_NAME}.h)

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

            # Don't override the exception for *AudioImporter plugins
            set(_MAGNUMPLUGINS_${_COMPONENT}_INCLUDE_PATH_SUFFIX MagnumPlugins/${_component})
            if(NOT _MAGNUMPLUGINS_${_COMPONENT}_INCLUDE_PATH_NAMES)
                set(_MAGNUMPLUGINS_${_COMPONENT}_INCLUDE_PATH_NAMES ${_component}.h)
            endif()

            # Dynamic plugins don't have any prefix (e.g. `lib` on Linux),
            # search with empty prefix and then reset that back so we don't
            # accidentaly break something else
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
        endif()

        # Library location for plugins/libraries
        if(_component MATCHES ${_MAGNUMPLUGINS_PLUGIN_COMPONENTS} OR _component MATCHES ${_MAGNUMPLUGINS_LIBRARY_COMPONENTS})
            if(MAGNUMPLUGINS_${_COMPONENT}_LIBRARY_RELEASE)
                set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                    IMPORTED_CONFIGURATIONS RELEASE)
                set_property(TARGET MagnumPlugins::${_component} PROPERTY
                    IMPORTED_LOCATION_RELEASE ${MAGNUMPLUGINS_${_COMPONENT}_LIBRARY_RELEASE})
            endif()

            if(MAGNUMPLUGINS_${_COMPONENT}_LIBRARY_DEBUG)
                set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                    IMPORTED_CONFIGURATIONS DEBUG)
                set_property(TARGET MagnumPlugins::${_component} PROPERTY
                    IMPORTED_LOCATION_DEBUG ${MAGNUMPLUGINS_${_COMPONENT}_LIBRARY_DEBUG})
            endif()
        endif()

        # AssimpImporter plugin dependencies
        if(_component STREQUAL AssimpImporter)
            find_package(Assimp)
            set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES Assimp::Assimp)

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
            endif()
        elseif(_component STREQUAL BasisImporter)
            find_package(basisu CONFIG QUIET)
            if(basisu_FOUND AND NOT BASIS_UNIVERSAL_DIR)
                set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                    INTERFACE_LINK_LIBRARIES basisu_transcoder)
            endif()

        # DdsImporter has no dependencies

        # DevIlImageImporter plugin dependencies
        elseif(_component STREQUAL DevIlImageImporter)
            find_package(DevIL)
            set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                INTERFACE_LINK_LIBRARIES ${IL_LIBRARIES} ${ILU_LIBRARIES})

        # DrFlacAudioImporter has no dependencies
        # DrMp3AudioImporter has no dependencies
        # DrWavAudioImporter has no dependencies

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
        endif()

        # PrimitiveImporter has no dependencies
        # StanfordImporter has no dependencies
        # StanfordSceneConverter has no dependencies
        # StbImageConverter has no dependencies
        # StbImageImporter has no dependencies
        # StbTrueTypeFont has no dependencies
        # StbVorbisAudioImporter has no dependencies
        # StlImporter has no dependencies
        # TinyGltfImporter has no dependencies

        # Find plugin/library includes
        if(_component MATCHES ${_MAGNUMPLUGINS_PLUGIN_COMPONENTS} OR _component MATCHES ${_MAGNUMPLUGINS_LIBRARY_COMPONENTS})
            find_path(_MAGNUMPLUGINS_${_COMPONENT}_INCLUDE_DIR
                NAMES ${_MAGNUMPLUGINS_${_COMPONENT}_INCLUDE_PATH_NAMES}
                HINTS ${MAGNUMPLUGINS_INCLUDE_DIR}/${_MAGNUMPLUGINS_${_COMPONENT}_INCLUDE_PATH_SUFFIX})
            mark_as_advanced(_MAGNUMPLUGINS_${_COMPONENT}_INCLUDE_DIR)
        endif()

        # Automatic import of static plugins
        if(_component MATCHES ${_MAGNUMPLUGINS_PLUGIN_COMPONENTS})
            file(READ ${_MAGNUMPLUGINS_${_COMPONENT}_INCLUDE_DIR}/configure.h _magnumPlugins${_component}Configure)
            string(FIND "${_magnumPlugins${_component}Configure}" "#define MAGNUM_${_COMPONENT}_BUILD_STATIC" _magnumPlugins${_component}_BUILD_STATIC)
            if(NOT _magnumPlugins${_component}_BUILD_STATIC EQUAL -1)
                set_property(TARGET MagnumPlugins::${_component} APPEND PROPERTY
                    INTERFACE_SOURCES ${_MAGNUMPLUGINS_${_COMPONENT}_INCLUDE_DIR}/importStaticPlugin.cpp)
            endif()
        endif()

        if(_component MATCHES ${_MAGNUMPLUGINS_PLUGIN_COMPONENTS} OR _component MATCHES ${_MAGNUMPLUGINS_LIBRARY_COMPONENTS})
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

        # Decide if the plugin/library was found
        if((_component MATCHES ${_MAGNUMPLUGINS_PLUGIN_COMPONENTS} OR _component MATCHES ${_MAGNUMPLUGINS_LIBRARY_COMPONENTS}) AND _MAGNUMPLUGINS_${_COMPONENT}_INCLUDE_DIR AND (MAGNUMPLUGINS_${_COMPONENT}_LIBRARY_DEBUG OR MAGNUMPLUGINS_${_COMPONENT}_LIBRARY_RELEASE))
            set(MagnumPlugins_${_component}_FOUND TRUE)
        else()
            set(MagnumPlugins_${_component}_FOUND FALSE)
        endif()
    endif()
endforeach()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MagnumPlugins
    REQUIRED_VARS MAGNUMPLUGINS_INCLUDE_DIR
    HANDLE_COMPONENTS)
