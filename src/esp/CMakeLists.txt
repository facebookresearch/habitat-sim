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
add_subdirectory(agent)
add_subdirectory(sensor)
add_subdirectory(sim)
