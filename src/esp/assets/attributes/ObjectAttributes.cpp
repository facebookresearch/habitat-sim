// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ObjectAttributes.h"
namespace esp {
namespace assets {
namespace attributes {

// All keys must be lowercase
const std::map<std::string, esp::assets::AssetType>
    AbstractObjectAttributes::AssetTypeNamesMap = {
        {"mp3d", AssetType::MP3D_MESH},
        {"navmesh", AssetType::NAVMESH},
        {"ptex", AssetType::FRL_PTEX_MESH},
        {"semantic", AssetType::INSTANCE_MESH},
        {"suncg", AssetType::SUNCG_SCENE},
};

const std::string AbstractObjectAttributes::JSONConfigTestString =
    R"({
      "scale":[2,3,4],
      "margin": 0.9,
      "friction coefficient": 0.321,
      "restitution coefficient": 0.456,
      "requires lighting": false,
      "units to meters": 1.1,
      "up":[2.1,0,0],
      "front":[0,2.1,0],
      "render mesh": "testJSONRenderAsset.glb",
      "collision mesh": "testJSONCollisionAsset.glb")";

const std::string ObjectAttributes::JSONConfigTestString =
    AbstractObjectAttributes::JSONConfigTestString +
    R"(,
      "mass": 9,
      "use bounding box for collision": true,
      "join collision meshes":true,
      "inertia": [1.1, 0.9, 0.3],
      "COM": [0.1,0.2,0.3]
    })";

const std::string StageAttributes::JSONConfigTestString =
    AbstractObjectAttributes::JSONConfigTestString +
    R"(,
      "gravity": [9,8,7],
      "origin":[1,2,3],
      "semantic mesh":"testJSONSemanticAsset.glb",
      "nav mesh":"testJSONNavMeshAsset.glb",
      "house filename":"testJSONHouseFileName.glb"
    })";
AbstractObjectAttributes::AbstractObjectAttributes(
    const std::string& attributesClassKey,
    const std::string& handle)
    : AbstractAttributes(attributesClassKey, handle) {
  setFrictionCoefficient(0.5);
  setRestitutionCoefficient(0.1);
  setScale({1.0, 1.0, 1.0});
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
  setRequiresLighting(true);
  setIsVisible(true);
  setSemanticId(0);
  setIsCollidable(true);
}  // ObjectAttributes ctor

StageAttributes::StageAttributes(const std::string& handle)
    : AbstractObjectAttributes("StageAttributes", handle) {
  setGravity({0, -9.8, 0});
  setOrigin({0, 0, 0});

  setRequiresLighting(false);
  // 0 corresponds to esp::assets::AssetType::UNKNOWN->treated as general mesh
  setCollisionAssetType(0);
  // 4 corresponds to esp::assets::AssetType::INSTANCE_MESH
  setSemanticAssetType(4);
}  // StageAttributes ctor

}  // namespace attributes
}  // namespace assets
}  // namespace esp
