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
