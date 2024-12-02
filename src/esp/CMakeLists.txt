# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# https://embeddedartistry.com/blog/2017/3/7/clang-weverything
# TODO(msb) enable CORRADE_USE_PEDANTIC
# set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)

option(BUILD_WARNINGS_AS_ERRORS "Build with warnings as errors" OFF)
if(BUILD_WARNINGS_AS_ERRORS)
  if(MSVC)
    add_compile_options(/WX)
  else()
    # TODO(msb) remove -Wall once we enable CORRADE_USE_PEDANTIC
    add_compile_options(-Wall)
    add_compile_options(-Werror)
  endif()
endif()

###########################
# Set conditional build variables used in configure.h files

if(BUILD_ASSIMP_SUPPORT)
  set(ESP_BUILD_ASSIMP_SUPPORT ON)
endif()

if(BUILD_WITH_AUDIO)
  set(ESP_BUILD_WITH_AUDIO ON)
endif()

if(BUILD_WITH_BACKGROUND_RENDERER)
  set(ESP_BUILD_WITH_BACKGROUND_RENDERER ON)
endif()

if(BUILD_WITH_BULLET)
  set(ESP_BUILD_WITH_BULLET ON)
endif()

if(BUILD_WITH_CUDA)
  set(ESP_BUILD_WITH_CUDA ON)
endif()

##########################
# Set path variables

# Set PBR configuration default path. Used in gfx/configure.h
set(ESP_DEFAULT_PBRSHADER_CONFIG_REL_PATH ./data/default.pbr_config.json)
set(ESP_DEFAULT_PBRSHADER_CONFIG
    ${PROJECT_SOURCE_DIR}/.${ESP_DEFAULT_PBRSHADER_CONFIG_REL_PATH}
)

# Set Physics Manager configuration default path. Used in physics/configure.h
set(ESP_DEFAULT_PHYSICS_CONFIG_REL_PATH ./data/default.physics_config.json)
set(ESP_DEFAULT_PHYSICS_CONFIG
    ${PROJECT_SOURCE_DIR}/.${ESP_DEFAULT_PHYSICS_CONFIG_REL_PATH}
)

#################################
# Configure.h file generation
# core
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/core/configure.h.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/core/configure.h
)
# gfx
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/gfx/configure.h.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/gfx/configure.h
)
# physics
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/physics/configure.h.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/physics/configure.h
)
# sensor
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/sensor/configure.h.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/sensor/configure.h
)

add_subdirectory(core)
add_subdirectory(geo)
add_subdirectory(gfx)
add_subdirectory(gfx_batch)
add_subdirectory(assets)
add_subdirectory(metadata)
add_subdirectory(io)
add_subdirectory(scene)
add_subdirectory(physics)
add_subdirectory(nav)
add_subdirectory(sensor)
add_subdirectory(sim)
