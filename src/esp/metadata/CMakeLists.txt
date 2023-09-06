# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

set(
  metadata_SOURCES
  attributes/AttributesBase.h
  attributes/AttributesBase.cpp
  attributes/AttributesEnumMaps.h
  attributes/AttributesEnumMaps.cpp
  attributes/AbstractObjectAttributes.h
  attributes/AbstractObjectAttributes.cpp
  attributes/ArticulatedObjectAttributes.h
  attributes/ArticulatedObjectAttributes.cpp
  attributes/LightLayoutAttributes.h
  attributes/LightLayoutAttributes.cpp
  attributes/ObjectAttributes.h
  attributes/ObjectAttributes.cpp
  attributes/PhysicsManagerAttributes.h
  attributes/PhysicsManagerAttributes.cpp
  attributes/PbrShaderAttributes.h
  attributes/PbrShaderAttributes.cpp
  attributes/PrimitiveAssetAttributes.h
  attributes/PrimitiveAssetAttributes.cpp
  attributes/SceneInstanceAttributes.h
  attributes/SceneInstanceAttributes.cpp
  attributes/SceneDatasetAttributes.h
  attributes/SceneDatasetAttributes.cpp
  attributes/StageAttributes.h
  attributes/StageAttributes.cpp
  managers/AttributesManagerBase.h
  managers/AbstractObjectAttributesManager.h
  managers/AOAttributesManager.h
  managers/AOAttributesManager.cpp
  managers/AssetAttributesManager.h
  managers/AssetAttributesManager.cpp
  managers/LightLayoutAttributesManager.h
  managers/LightLayoutAttributesManager.cpp
  managers/ObjectAttributesManager.h
  managers/ObjectAttributesManager.cpp
  managers/PbrShaderAttributesManager.h
  managers/PbrShaderAttributesManager.cpp
  managers/PhysicsAttributesManager.h
  managers/PhysicsAttributesManager.cpp
  managers/SceneInstanceAttributesManager.h
  managers/SceneInstanceAttributesManager.cpp
  managers/SceneDatasetAttributesManager.h
  managers/SceneDatasetAttributesManager.cpp
  managers/StageAttributesManager.h
  managers/StageAttributesManager.cpp
  MetadataMediator.h
  MetadataMediator.cpp
  URDFParser.cpp
  URDFParser.h
)
find_package(Magnum REQUIRED Primitives)

add_library(
  metadata STATIC
  ${metadata_SOURCES}
)

target_include_directories(metadata PRIVATE "${DEPS_DIR}/tinyxml2")

target_link_libraries(
  metadata
  PUBLIC Magnum::Magnum Magnum::Primitives
  PRIVATE core geo io tinyxml2
)
