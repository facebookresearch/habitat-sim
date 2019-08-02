// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

/* Bullet Physics Integration */
#include <Magnum/BulletIntegration/DebugDraw.h>
#include <Magnum/BulletIntegration/Integration.h>
#include <Magnum/BulletIntegration/MotionState.h>
#include <btBulletDynamicsCommon.h>

#include "BulletRigidObject.h"
#include "esp/physics/PhysicsManager.h"

namespace esp {
namespace physics {

class BulletPhysicsManager : public PhysicsManager {
 public:
  explicit BulletPhysicsManager(assets::ResourceManager* _resourceManager)
      : PhysicsManager(_resourceManager){};
  virtual ~BulletPhysicsManager();

  //============ Initialization =============
  // load physical properties and setup the world
  // do_profile indicates timing for FPS
  bool initPhysics(scene::SceneNode* node,
                   assets::PhysicsSceneMetaData sceneMetaData);

  //============ Object/Scene Instantiation =============
  //! Initialize scene given mesh data
  //! Only one scene per simulation
  //! The scene could contain several components
  bool addScene(const assets::AssetInfo& info,
                assets::PhysicsSceneMetaData& sceneMetaData,
                std::vector<assets::CollisionMeshData> meshGroup);

  //============ Simulator functions =============
  void stepPhysics(double dt = -1.0);

  void setGravity(const Magnum::Vector3d gravity);

  //============ Interact with objects =============
  // Alex NOTE: engine specifics handled by objects themselves...

 protected:
  //! The world has to live longer than the scene because RigidBody
  //! instances have to remove themselves from it on destruction
  btDbvtBroadphase bBroadphase_;
  btDefaultCollisionConfiguration bCollisionConfig_;
  btSequentialImpulseConstraintSolver bSolver_;
  btCollisionDispatcher bDispatcher_{&bCollisionConfig_};

  //! The following are made ptr because we need to intialize them in
  //! constructor, potentially with different world configurations
  std::shared_ptr<btDiscreteDynamicsWorld> bWorld_;

 private:
  bool isMeshPrimitiveValid(assets::CollisionMeshData& meshData);

  //! Create and initialize rigid object
  int makeRigidObject(std::vector<assets::CollisionMeshData> meshGroup,
                      assets::PhysicsObjectMetaData metaData);

};  // end class BulletPhysicsManager
}  // end namespace physics
}  // end namespace esp
