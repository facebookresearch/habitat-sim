# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

set(
  metadata_SOURCES
  attributes/AttributesBase.h
  attributes/DatasetAttributes.h
  attributes/DatasetAttributes.cpp
  attributes/ObjectAttributes.h
  attributes/ObjectAttributes.cpp
  attributes/SceneAttributes.h
  attributes/SceneAttributes.cpp
  attributes/PhysicsManagerAttributes.h
  attributes/PhysicsManagerAttributes.cpp
  attributes/PrimitiveAssetAttributes.h
  attributes/PrimitiveAssetAttributes.cpp
  managers/AttributesManagerBase.h
  managers/AbstractObjectAttributesManagerBase.h
  managers/AssetAttributesManager.h
  managers/AssetAttributesManager.cpp
  managers/DatasetAttributesManager.h
  managers/DatasetAttributesManager.cpp
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
  PRIVATE core geo io
)
