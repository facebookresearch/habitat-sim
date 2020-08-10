// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Attributes.h"

namespace esp {
namespace assets {

//----------------------------------------//
//  Derived attribute implementations
//----------------------------------------//

// All keys must be lowercase
const std::map<std::string, esp::assets::AssetType>
    AbstractPhysicsAttributes::AssetTypeNamesMap = {
        {"mp3d", AssetType::MP3D_MESH},
        {"navmesh", AssetType::NAVMESH},
        {"ptex", AssetType::FRL_PTEX_MESH},
        {"semantic", AssetType::INSTANCE_MESH},
        {"suncg", AssetType::SUNCG_SCENE},
};

AbstractPhysicsAttributes::AbstractPhysicsAttributes(
    const std::string& attributesClassKey,
    const std::string& handle)
    : AbstractAttributes(attributesClassKey, handle) {
  setFrictionCoefficient(0.5);
  setRestitutionCoefficient(0.1);
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
}  // AbstractPhysicsAttributes ctor

PhysicsObjectAttributes::PhysicsObjectAttributes(const std::string& handle)
    : AbstractPhysicsAttributes("PhysicsObjectAttributes", handle) {
  // fill necessary attribute defaults
  setMass(1.0);
  setMargin(0.04);
  setScale({1.0, 1.0, 1.0});
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
}  // PhysicsObjectAttributes ctor

PhysicsSceneAttributes::PhysicsSceneAttributes(const std::string& handle)
    : AbstractPhysicsAttributes("PhysicsSceneAttributes", handle) {
  setGravity({0, -9.8, 0});
  // TODO do these defaults need to be maintained here?
  setFrictionCoefficient(0.4);
  setRestitutionCoefficient(0.05);
  setOrigin({0, 0, 0});
  setOrientUp({0, 1, 0});
  setOrientFront({0, 0, -1});

  setRequiresLighting(false);
  // 0 corresponds to esp::assets::AssetType::UNKNOWN->treated as general mesh
  setCollisionAssetType(0);
  // 4 corresponds to esp::assets::AssetType::INSTANCE_MESH
  setSemanticAssetType(4);

}  // PhysicsSceneAttributes ctor

PhysicsManagerAttributes::PhysicsManagerAttributes(const std::string& handle)
    : AbstractAttributes("PhysicsManagerAttributes", handle) {
  setSimulator("none");
  setTimestep(0.01);
  setMaxSubsteps(10);
}  // PhysicsManagerAttributes ctor

CapsulePrimitiveAttributes::CapsulePrimitiveAttributes(
    bool isWireframe,
    int primObjType,
    const std::string& primObjClassName)
    : AbstractPrimitiveAttributes(isWireframe,
                                  primObjType,
                                  primObjClassName,
                                  "CapsulePrimitiveAttributes") {
  setCylinderRings(1);
  if (!isWireframe) {  // solid
    setHemisphereRings(4);
    setNumSegments(12);
    setHalfLength(0.75);
  } else {  // wireframe
    setHemisphereRings(8);
    setNumSegments(16);
    setHalfLength(1.0);
  }
  buildHandle();  // build handle based on config
}  // PhysicsCapsulePrimAttributes

ConePrimitiveAttributes::ConePrimitiveAttributes(
    bool isWireframe,
    int primObjType,
    const std::string& primObjClassName)
    : AbstractPrimitiveAttributes(isWireframe,
                                  primObjType,
                                  primObjClassName,
                                  "ConePrimitiveAttributes") {
  setHalfLength(1.25);

  if (!isWireframe) {  // solid
    setNumRings(1);
    setNumSegments(12);
    setCapEnd(true);
  } else {  // wireframe
    setNumSegments(32);
  }
  buildHandle();  // build handle based on config
}  // PhysicsConePrimAttributes

CylinderPrimitiveAttributes::CylinderPrimitiveAttributes(
    bool isWireframe,
    int primObjType,
    const std::string& primObjClassName)
    : AbstractPrimitiveAttributes(isWireframe,
                                  primObjType,
                                  primObjClassName,
                                  "CylinderPrimitiveAttributes") {
  setNumRings(1);
  setHalfLength(1.0);

  if (!isWireframe) {  // solid
    setNumSegments(12);
    setCapEnds(true);
  } else {  // wireframe
    setNumSegments(32);
  }
  buildHandle();  // build handle based on config
}  // PhysicsCylinderPrimAttributes

UVSpherePrimitiveAttributes::UVSpherePrimitiveAttributes(
    bool isWireframe,
    int primObjType,
    const std::string& primObjClassName)
    : AbstractPrimitiveAttributes(isWireframe,
                                  primObjType,
                                  primObjClassName,
                                  "UVSpherePrimitiveAttributes") {
  if (!isWireframe) {  // solid
    setNumRings(8);
    setNumSegments(16);
  } else {  // wireframe
    setNumRings(16);
    setNumSegments(32);
  }
  buildHandle();  // build handle based on config
}  // PhysicsUVSpherePrimAttributes

}  // namespace assets
}  // namespace esp
