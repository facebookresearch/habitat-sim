set(
  metadata_SOURCES
  AbstractManagedObject.h
  attributes/AttributesBase.h
  attributes/ObjectAttributes.h
  attributes/ObjectAttributes.cpp
  attributes/SceneAttributes.h
  attributes/SceneAttributes.cpp
  attributes/PhysicsManagerAttributes.h
  attributes/PhysicsManagerAttributes.cpp
  attributes/PrimitiveAssetAttributes.h
  attributes/PrimitiveAssetAttributes.cpp
  managers/AttributesManagerBase.h
  managers/AssetAttributesManager.h
  managers/AssetAttributesManager.cpp
  managers/ObjectAttributesManager.h
  managers/ObjectAttributesManager.cpp
  managers/PhysicsAttributesManager.h
  managers/PhysicsAttributesManager.cpp
  managers/StageAttributesManager.h
  managers/StageAttributesManager.cpp
)
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


if(BUILD_ASSIMP_SUPPORT)
  find_package(MagnumPlugins REQUIRED AssimpImporter)
endif()

add_library(
  metadata STATIC
  ${metadata_SOURCES}
)

target_link_libraries(
  metadata
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
  PRIVATE geo io
)

if(BUILD_ASSIMP_SUPPORT)
  target_link_libraries(
    metadata PUBLIC MagnumPlugins::AssimpImporter PRIVATE Assimp::Assimp
  )
endif()

if(OpenMP_CXX_FOUND)
  target_link_libraries(metadata PUBLIC OpenMP::OpenMP_CXX)
endif()
