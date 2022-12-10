# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

set(
  assets_SOURCES
  Asset.cpp
  Asset.h
  BaseMesh.cpp
  BaseMesh.h
  CollisionMeshData.h
  GenericSemanticMeshData.cpp
  GenericSemanticMeshData.h
  GenericMeshData.cpp
  GenericMeshData.h
  MeshData.h
  MeshMetaData.h
  RenderAssetInstanceCreationInfo.cpp
  RenderAssetInstanceCreationInfo.h
  ResourceManager.cpp
  ResourceManager.h
)

if(BUILD_PTEX_SUPPORT)
  list(APPEND assets_SOURCES PTexMeshData.cpp PTexMeshData.h)
endif()

find_package(
  Magnum
  REQUIRED
  GL
  MeshTools
  SceneGraph
  SceneTools
  Shaders
  Trade
  Primitives
)

add_library(
  assets STATIC
  ${assets_SOURCES}
)

target_link_libraries(
  assets
  PUBLIC core
         metadata
         physics
         scene
         nav
         Magnum::GL
         Magnum::Magnum
         Magnum::MeshTools
         Magnum::SceneGraph
         Magnum::SceneTools
         Magnum::Shaders
         Magnum::Trade
         Magnum::Primitives
  PRIVATE geo io
)

if(BUILD_WITH_VHACD)
  target_link_libraries(assets PUBLIC vhacd)
endif()

if(OpenMP_CXX_FOUND)
  target_link_libraries(assets PUBLIC OpenMP::OpenMP_CXX)
endif()
