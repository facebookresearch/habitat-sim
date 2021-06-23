# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

set(
  metadata_SOURCES
  attributes/AttributesBase.h
  attributes/AttributesBase.cpp
  attributes/LightLayoutAttributes.h
  attributes/LightLayoutAttributes.cpp
  attributes/ObjectAttributes.h
  attributes/ObjectAttributes.cpp
  attributes/PhysicsManagerAttributes.h
  attributes/PhysicsManagerAttributes.cpp
  attributes/PrimitiveAssetAttributes.h
  attributes/PrimitiveAssetAttributes.cpp
  attributes/SceneAttributes.h
  attributes/SceneAttributes.cpp
  attributes/SceneDatasetAttributes.h
  attributes/SceneDatasetAttributes.cpp
  managers/AttributesManagerBase.h
  managers/AbstractObjectAttributesManagerBase.h
  managers/AssetAttributesManager.h
  managers/AssetAttributesManager.cpp
  managers/LightLayoutAttributesManager.h
  managers/LightLayoutAttributesManager.cpp
  managers/ObjectAttributesManager.h
  managers/ObjectAttributesManager.cpp
  managers/PhysicsAttributesManager.h
  managers/PhysicsAttributesManager.cpp
  managers/SceneAttributesManager.h
  managers/SceneAttributesManager.cpp
  managers/SceneDatasetAttributesManager.h
  managers/SceneDatasetAttributesManager.cpp
  managers/StageAttributesManager.h
  managers/StageAttributesManager.cpp
  MetadataMediator.h
  MetadataMediator.cpp
  MetadataUtils.h
  MetadataUtils.cpp
)
find_package(Magnum REQUIRED Primitives)

add_library(
  metadata STATIC
  ${metadata_SOURCES}
)

target_link_libraries(
  metadata
  PUBLIC Magnum::Magnum Magnum::Primitives
  PRIVATE core geo io
)
