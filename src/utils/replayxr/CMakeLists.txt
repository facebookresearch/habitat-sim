# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# TODO: Clean-up dependencies.
find_package(
  Magnum
  REQUIRED
  GL
  MeshTools
  Primitives
  Shaders
  EmscriptenApplication
)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)

# The library_webxr.js and webxr.h files are bundled from
# https://github.com/VhiteRabbit/emscripten-webxr/tree/e101ce7cfc3c92962a65771f43c2d61775edd348

# Closure Compiler chokes on the JS file, disable it for this example
string(REPLACE "--closure 1" "" CMAKE_EXE_LINKER_FLAGS_RELEASE
               "${CMAKE_EXE_LINKER_FLAGS_RELEASE}"
)

# TODO: Clean-up dependencies.
add_executable(replayxr webxr.h WebXrExample.cpp)
target_link_libraries(
  replayxr
  PRIVATE core
          metadata
          io
          gfx
          sensor
          gfx_batch
          Magnum::Application
          Magnum::GL
          Magnum::Magnum
          Magnum::MeshTools
          Magnum::Primitives
          Magnum::Shaders
)

set_property(
  TARGET replayxr
  APPEND_STRING
  PROPERTY
    LINK_FLAGS
    "\
    --js-library ${CMAKE_CURRENT_SOURCE_DIR}/library_webxr.js\
    -sASSERTIONS\
    -sEXPORTED_RUNTIME_METHODS=dynCall\
    "
)
set_target_properties(
  replayxr
  PROPERTIES
    # Override the Magnum-set default which is in $<CONFIG>/bin or $<CONFIG>/lib
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

# Symlink client files
set(
  resources
  index.html driver.js ${MAGNUM_EMSCRIPTENAPPLICATION_JS} ${MAGNUM_WEBAPPLICATION_CSS}
  ${DATA_DIR} # Symlink data folder so that the server can access assets from ./data
)
foreach(resource ${resources})
  get_filename_component(filename ${resource} NAME)
  get_filename_component(path ${resource} ABSOLUTE)
  add_custom_command(
    TARGET replayxr POST_BUILD COMMAND ${CMAKE_COMMAND} -E create_symlink ${path}
                                       ${CMAKE_CURRENT_BINARY_DIR}/${filename}
  )
endforeach()
