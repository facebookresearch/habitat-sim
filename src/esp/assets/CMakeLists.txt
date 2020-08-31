set(
  assets_SOURCES
  Asset.cpp
  Asset.h
  attributes/AttributesBase.h
  attributes/ObjectAttributes.h
  attributes/ObjectAttributes.cpp
  attributes/SceneAttributes.h
  attributes/SceneAttributes.cpp
  attributes/PhysicsManagerAttributes.h
  attributes/PhysicsManagerAttributes.cpp
  attributes/PrimitiveAssetAttributes.h
  attributes/PrimitiveAssetAttributes.cpp
  BaseMesh.cpp
  BaseMesh.h
  CollisionMeshData.h
  GenericInstanceMeshData.cpp
  GenericInstanceMeshData.h
  GenericMeshData.cpp
  GenericMeshData.h
  MeshData.h
  MeshMetaData.h
  Mp3dInstanceMeshData.cpp
  Mp3dInstanceMeshData.h
  managers/AttributesManagerBase.h
  managers/AssetAttributesManager.h
  managers/AssetAttributesManager.cpp
  managers/ObjectAttributesManager.h
  managers/ObjectAttributesManager.cpp
  managers/PhysicsAttributesManager.h
  managers/PhysicsAttributesManager.cpp
  managers/StageAttributesManager.h
  managers/StageAttributesManager.cpp
  ResourceManager.cpp
  ResourceManager.h
)

if(BUILD_PTEX_SUPPORT)
  list(APPEND assets_SOURCES PTexMeshData.cpp PTexMeshData.h)
endif()

find_package(
  Magnum
  REQUIRED
  AnyImageImporter
  AnySceneImporter
  GL
  MeshTools
  SceneGraph
  Shaders
  Trade
  Primitives
)

find_package(
  MagnumPlugins
  REQUIRED
  BasisImporter
  PrimitiveImporter
  StanfordImporter
  StbImageImporter
  StbImageConverter
  TinyGltfImporter
)

if(BUILD_ASSIMP_SUPPORT)
  find_package(MagnumPlugins REQUIRED AssimpImporter)
endif()

add_library(
  assets STATIC
  ${assets_SOURCES}
)

target_link_libraries(
  assets
  PUBLIC core
         physics
         scene
         nav
         Magnum::AnyImageImporter
         Magnum::AnySceneImporter
         Magnum::GL
         Magnum::Magnum
         Magnum::MeshTools
         Magnum::SceneGraph
         Magnum::Shaders
         Magnum::Trade
         Magnum::Primitives
         MagnumPlugins::BasisImporter
         MagnumPlugins::PrimitiveImporter
         MagnumPlugins::StanfordImporter
         MagnumPlugins::StbImageImporter
         MagnumPlugins::StbImageConverter
         MagnumPlugins::TinyGltfImporter
  PRIVATE geo io
)

if(BUILD_ASSIMP_SUPPORT)
  target_link_libraries(
    assets PUBLIC MagnumPlugins::AssimpImporter PRIVATE Assimp::Assimp
  )
endif()

if(OpenMP_CXX_FOUND)
  target_link_libraries(assets PUBLIC OpenMP::OpenMP_CXX)
endif()
