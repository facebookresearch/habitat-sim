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
find_package(Magnum REQUIRED Primitives)

add_library(
  metadata STATIC
  ${metadata_SOURCES}
)

target_link_libraries(
  metadata
  PUBLIC Magnum::Magnum Magnum::Primitives
  PRIVATE geo io
)
