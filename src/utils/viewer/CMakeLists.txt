# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

find_package(Magnum REQUIRED DebugTools GlfwApplication Text)
find_package(
  MagnumPlugins
  REQUIRED
  BasisImporter
  GltfImporter
  KtxImporter
  StanfordImporter
  StbImageImporter
  StbImageConverter
  StbTrueTypeFont
  UfbxImporter
)

set(viewer_SOURCES viewer.cpp ObjectPickingHelper.cpp ObjectPickingHelper.h)

corrade_add_resource(viewer_RESOURCES resources.conf)

#set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)
add_executable(viewer ${viewer_SOURCES} ${viewer_RESOURCES})

target_link_libraries(
  viewer
  PRIVATE habitat_sim
          Magnum::DebugTools
          Magnum::GlfwApplication
          Magnum::Text
          MagnumPlugins::BasisImporter
          MagnumPlugins::GltfImporter
          MagnumPlugins::KtxImporter
          MagnumPlugins::StanfordImporter
          MagnumPlugins::StbImageImporter
          MagnumPlugins::StbImageConverter
          MagnumPlugins::StbTrueTypeFont
          MagnumPlugins::UfbxImporter
)
