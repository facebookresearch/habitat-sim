# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

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
set(Datatool_SOURCES Datatool.cpp SceneLoader.cpp Mp3dInstanceMeshData.cpp)

add_executable(Datatool ${Datatool_SOURCES})
set(DEPS_DIR "${CMAKE_CURRENT_LIST_DIR}/../../deps")
target_include_directories(Datatool SYSTEM PRIVATE "${DEPS_DIR}/tinyobjloader")

target_link_libraries(
  Datatool
  PRIVATE habitat_sim
          MagnumPlugins::BasisImporter
          MagnumPlugins::GltfImporter
          MagnumPlugins::StanfordImporter
          MagnumPlugins::StbImageImporter
          MagnumPlugins::StbImageConverter
          MagnumPlugins::UfbxImporter
)
if(BUILD_ASSIMP_SUPPORT)
  target_link_libraries(
    Datatool PUBLIC MagnumPlugins::AssimpImporter PRIVATE Assimp::Assimp
  )
endif()
