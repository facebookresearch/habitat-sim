# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

find_package(Magnum REQUIRED AnySceneImporter)
find_package(
  MagnumPlugins
  REQUIRED
  BasisImporter
  GltfImporter
  StanfordImporter
  StbImageImporter
  StbImageConverter
  UfbxImporter
)
if(BUILD_ASSIMP_SUPPORT)
  find_package(MagnumPlugins REQUIRED AssimpImporter)
endif()
set(datatool_SOURCES Datatool.cpp SceneLoader.cpp Mp3dInstanceMeshData.cpp)

add_executable(datatool ${datatool_SOURCES})
set(DEPS_DIR "${CMAKE_CURRENT_LIST_DIR}/../../deps")
target_include_directories(datatool SYSTEM PRIVATE "${DEPS_DIR}/tinyobjloader")

target_link_libraries(
  datatool
  PRIVATE habitat_sim
          Magnum::AnySceneImporter
          MagnumPlugins::BasisImporter
          MagnumPlugins::GltfImporter
          MagnumPlugins::StanfordImporter
          MagnumPlugins::StbImageImporter
          MagnumPlugins::StbImageConverter
          MagnumPlugins::UfbxImporter
)
if(BUILD_ASSIMP_SUPPORT)
  target_link_libraries(datatool PUBLIC MagnumPlugins::AssimpImporter)
endif()
