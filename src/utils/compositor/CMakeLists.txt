# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

find_package(Magnum REQUIRED
  # Tools, tools, tools everywhere
  MaterialTools
  MeshTools
  SceneTools
  TextureTools
  Trade)

# This is where static plugins are really a pain to deal with as we need
# basically all of them. With dynamic plugins none of this would be needed.
find_package(Magnum REQUIRED
  AnySceneImporter)
find_package(MagnumPlugins REQUIRED
  AssimpImporter
  GltfImporter
  StlImporter
  StbImageImporter
  StbResizeImageConverter
  KtxImageConverter
  GltfSceneConverter)

add_library(CompositorObjects OBJECT
  CompositorState.cpp
  CompositorState.h)
target_include_directories(CompositorObjects PRIVATE $<TARGET_PROPERTY:Magnum::Magnum,INTERFACE_INCLUDE_DIRECTORIES>)

add_executable(compositor-replicacad compositor-replicacad.cpp $<TARGET_OBJECTS:CompositorObjects>)
target_link_libraries(compositor-replicacad PRIVATE
  Magnum::MaterialTools
  Magnum::MeshTools
  Magnum::SceneTools
  Magnum::TextureTools
  Magnum::Trade
  Magnum::AnySceneImporter
  MagnumPlugins::GltfImporter
  MagnumPlugins::StbImageImporter
  MagnumPlugins::KtxImageConverter
  MagnumPlugins::GltfSceneConverter)

add_executable(compositor-ycb compositor-ycb.cpp $<TARGET_OBJECTS:CompositorObjects>)
target_link_libraries(compositor-ycb PRIVATE
  Magnum::MeshTools
  Magnum::SceneTools
  Magnum::TextureTools
  Magnum::Trade
  Magnum::AnySceneImporter
  MagnumPlugins::GltfImporter
  # TODO Gotta admit, this plugin is awfully slow at opening all these
  # uselessly huge 4K textures. Or maybe it's the resizing operation taking
  # too long? With an external (release) Magnum build and libpng-based
  # importers it's not this awful.
  MagnumPlugins::StbImageImporter
  MagnumPlugins::StbResizeImageConverter
  MagnumPlugins::KtxImageConverter
  MagnumPlugins::GltfSceneConverter)

add_executable(compositor-fetch compositor-fetch.cpp $<TARGET_OBJECTS:CompositorObjects>)
target_link_libraries(compositor-fetch PRIVATE
  Magnum::MaterialTools
  Magnum::MeshTools
  Magnum::SceneTools
  Magnum::TextureTools
  Magnum::Trade
  Magnum::AnySceneImporter
  MagnumPlugins::AssimpImporter
  MagnumPlugins::GltfImporter
  MagnumPlugins::StlImporter
  MagnumPlugins::StbImageImporter
  MagnumPlugins::StbResizeImageConverter
  MagnumPlugins::KtxImageConverter
  MagnumPlugins::GltfSceneConverter)
