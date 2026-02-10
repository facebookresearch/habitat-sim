# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

find_package(Magnum REQUIRED GlfwApplication)
find_package(
  MagnumPlugins
  REQUIRED
  BasisImporter
  GltfImporter
  KtxImporter
  StanfordImporter
  StbImageImporter
  StbImageConverter
  UfbxImporter
)

add_executable(replayer replayer.cpp)
target_link_libraries(
  replayer
  PRIVATE habitat_sim
          gfx_batch
          Magnum::Application
          Magnum::GlfwApplication
          MagnumPlugins::BasisImporter
          MagnumPlugins::GltfImporter
          MagnumPlugins::KtxImporter
          MagnumPlugins::StanfordImporter
          MagnumPlugins::StbImageImporter
          MagnumPlugins::StbImageConverter
          MagnumPlugins::UfbxImporter
)

# Create convenience symlink at the repo root's build/ directory so
# developers don't have to hunt through build/<wheel_tag>/... paths.
if(DEFINED HSIM_REPO_BUILD_DIR)
  add_custom_command(
    TARGET replayer
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E create_symlink $<TARGET_FILE:replayer>
            "${HSIM_REPO_BUILD_DIR}/replayer"
    COMMENT "Symlinked: build/replayer"
  )
endif()
