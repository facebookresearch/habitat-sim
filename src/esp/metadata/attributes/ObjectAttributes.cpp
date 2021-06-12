// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ObjectAttributes.h"
namespace esp {
namespace metadata {
namespace attributes {

// All keys must be lowercase
const std::map<std::string, esp::assets::AssetType>
    AbstractObjectAttributes::AssetTypeNamesMap = {
        {"mp3d", esp::assets::AssetType::MP3D_MESH},
        {"navmesh", esp::assets::AssetType::NAVMESH},
        {"ptex", esp::assets::AssetType::FRL_PTEX_MESH},
        {"semantic", esp::assets::AssetType::INSTANCE_MESH},
        {"suncg", esp::assets::AssetType::SUNCG_SCENE},
};

// All keys must be lowercase
const std::map<std::string, ObjectInstanceShaderType>
    AbstractObjectAttributes::ShaderTypeNamesMap = {
        {"flat", ObjectInstanceShaderType::Flat},
        {"phong", ObjectInstanceShaderType::Phong},
        {"pbr", ObjectInstanceShaderType::PBR},
};

AbstractObjectAttributes::AbstractObjectAttributes(
    const std::string& attributesClassKey,
    const std::string& handle)
    : AbstractAttributes(attributesClassKey, handle) {
  setFrictionCoefficient(0.5);
  setRestitutionCoefficient(0.1);
  setScale({1.0, 1.0, 1.0});
  setCollisionAssetSize({1.0, 1.0, 1.0});
  setMargin(0.04);
  setOrientUp({0, 1, 0});
  setOrientFront({0, 0, -1});
  // default rendering and collisions will be mesh for physics objects and
  // scenes. Primitive-based objects do not currently support mesh collisions,
  // however, due to issues with how non-triangle meshes (i.e. wireframes) are
  // handled in @ref GenericMeshData::setMeshData
  setRenderAssetIsPrimitive(false);
  setCollisionAssetIsPrimitive(false);
  setUseMeshCollision(true);
  setIsCollidable(true);
  setUnitsToMeters(1.0);
  setRenderAssetHandle("");
  setCollisionAssetHandle("");
}  // AbstractObjectAttributes ctor

ObjectAttributes::ObjectAttributes(const std::string& handle)
    : AbstractObjectAttributes("ObjectAttributes", handle) {
  // fill necessary attribute defaults
  setMass(1.0);
  setCOM({0, 0, 0});
  setInertia({0, 0, 0});
  setLinearDamping(0.2);
  setAngularDamping(0.2);

  setComputeCOMFromShape(true);

  setBoundingBoxCollisions(false);
  setJoinCollisionMeshes(true);
  // default to unknown for objects
  setShaderType(static_cast<int>(ObjectInstanceShaderType::Unknown));
  // TODO remove this once ShaderType support is complete
  setRequiresLighting(true);
  setIsVisible(true);
  setSemanticId(0);
}  // ObjectAttributes ctor

StageAttributes::StageAttributes(const std::string& handle)
    : AbstractObjectAttributes("StageAttributes", handle) {
  setGravity({0, -9.8, 0});
  setOrigin({0, 0, 0});
  // default to unknown for stages
  setShaderType(static_cast<int>(ObjectInstanceShaderType::Flat));
  // TODO remove this once ShaderType support is complete
  setRequiresLighting(false);
  // 0 corresponds to esp::assets::AssetType::UNKNOWN->treated as general mesh
  setCollisionAssetType(0);
  // 4 corresponds to esp::assets::AssetType::INSTANCE_MESH
  setSemanticAssetType(4);
}  // StageAttributes ctor

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
