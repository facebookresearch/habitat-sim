// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

/* Bullet Physics Integration */
#include <Magnum/Trade/MeshData3D.h>
#include <Magnum/BulletIntegration/Integration.h>
#include <Magnum/BulletIntegration/MotionState.h>
#include <Magnum/BulletIntegration/DebugDraw.h>
#include <Magnum/Timeline.h>
#include <btBulletDynamicsCommon.h>

#include "esp/physics/PhysicsManager.h"
#include "BulletRigidObject.h"

namespace esp {
namespace physics {

class BulletPhysicsManager : public PhysicsManager {
 public:
  explicit BulletPhysicsManager(assets::ResourceManager* _resourceManager) : PhysicsManager(_resourceManager) {};
  virtual ~BulletPhysicsManager();

  //============ Initialization =============
  //load physical properties and setup the world
  //do_profile indicates timing for FPS
  bool initPhysics(scene::SceneNode* node,
                   Magnum::Vector3d gravity,
                   bool do_profile            = false);

  //============ Object/Scene Instantiation =============
  //! Initialize scene given mesh data
  //! Only one scene per simulation
  //! The scene could contain several components
  bool addScene(
      const assets::AssetInfo& info,
      std::vector<assets::CollisionMeshData> meshGroup);

  //! Initialize object given mesh data
  //! The object could contain several parts
  int addObject(
      const std::string configFile,
      physics::PhysicalObjectType objectType,
      DrawableGroup* drawables);

  //============ Simulator functions =============
  void stepPhysics();

  //============ Interact with objects =============
  //Alex NOTE: engine specifics handled by objects themselves...

protected:

  //! The world has to live longer than the scene because RigidBody
  //! instances have to remove themselves from it on destruction
  Magnum::BulletIntegration::DebugDraw    debugDraw_{Magnum::NoCreate};
  btDbvtBroadphase                        bBroadphase_;
  btDefaultCollisionConfiguration         bCollisionConfig_;
  btSequentialImpulseConstraintSolver     bSolver_;
  btCollisionDispatcher                   bDispatcher_{&bCollisionConfig_};
  
  //! The following are made ptr because we need to intialize them in constructor,
  //! potentially with different world configurations
  std::shared_ptr<btDiscreteDynamicsWorld>   bWorld_;


};//end class BulletPhysicsManager
}//end namespace physics
}//end namespace esp

