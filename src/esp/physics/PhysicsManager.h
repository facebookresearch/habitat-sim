// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

/* Bullet Physics Integration */
#include <Magnum/Trade/MeshData3D.h>
#include <Magnum/BulletIntegration/Integration.h>
#include <Magnum/BulletIntegration/MotionState.h>
#include <Magnum/BulletIntegration/DebugDraw.h>
#include <Magnum/Timeline.h>
#include <btBulletDynamicsCommon.h>

#include "esp/assets/Asset.h"
#include "esp/assets/BaseMesh.h"
#include "esp/assets/GenericInstanceMeshData.h"
#include "esp/assets/MeshMetaData.h"
#include "esp/assets/CollisionMeshData.h"
#include "esp/assets/MeshData.h"
#include "esp/assets/ResourceManager.h"
#include "esp/scene/SceneNode.h"
#include "RigidObject.h"

//#include <Magnum/Math/Angle.h>
// Debug draw
#include <Magnum/DebugTools/ForceRenderer.h>
#include <Magnum/DebugTools/ResourceManager.h>


namespace esp {
namespace physics {

class PhysicsManager {
 public:
  explicit PhysicsManager(assets::ResourceManager& _resourceManager) : resourceManager(_resourceManager) {};
  virtual ~PhysicsManager();

  //load physical properties and setup the world
  //do_profile indicates timing for FPS
  bool initPhysics(scene::SceneNode* node,
                   Magnum::Vector3d gravity,
                   std::string simulator="bullet",
                   bool do_profile=false);
  
  //! Initialize object given mesh data
  //! The object could contain several parts
  int initObject(
      const assets::AssetInfo& info,
      const assets::MeshMetaData& metaData,
      std::vector<assets::CollisionMeshData> meshGroup,
      physics::RigidObject* physObject);

  //! Initialize scene given mesh data
  //! The scene could contain several components
  bool initScene(
      const assets::AssetInfo& info,
      const assets::MeshMetaData& metaData,
      std::vector<assets::CollisionMeshData> meshGroup,
      physics::RigidObject* physObject);

  void debugSceneGraph(const MagnumObject* root);

  void stepPhysics();
  void nextFrame();

  void checkActiveObjects();

  void applyForce(const int objectID,
      Magnum::Vector3 force,
      Magnum::Vector3 relPos);

  void applyImpulse(const int objectID,
      Magnum::Vector3 impulse,
      Magnum::Vector3 relPos);

  void setTimestep(double dt);

  double getTimestep(){return fixedTimeStep_;};

  Magnum::SceneGraph::DrawableGroup3D& getDrawables() { return debugDrawables; }
  const Magnum::SceneGraph::DrawableGroup3D& getDrawables() const {
    return debugDrawables;
  }

 protected:

  void getPhysicsEngine();

  //! Check if mesh primitive type is valid for bullet physics engine
  bool isMeshPrimitiveValid(assets::CollisionMeshData& meshData);

  //use this to instantiate physics objects from the physicsObjectLibrary_
  assets::ResourceManager& resourceManager;

  //! ==== physics engines ====
  enum PhysicsSimulationLibrary {BULLET};
  enum PhysicsSimulationLibrary activePhysSimLib_ = BULLET; //default

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

  //! Used to keep track of all sceneNodes that have physical properties
  scene::SceneNode* physicsNode = nullptr;

  //! ==== dynamic object resources ===
  std::map<int, physics::RigidObject*> dynamicObjects_;
  int nextObjectID_ = 0;

  bool initialized_ = false;
  bool do_profile_ = false;
  float total_time_ = 0.0f;
  int total_frames_ = 0;

  Magnum::Timeline timeline_;
  int maxSubSteps_ = 10;
  float fixedTimeStep_ = 1.0f / 240.0f;


  /* Debug Draw */
  Magnum::DebugTools::ResourceManager debugManager;
  Magnum::SceneGraph::DrawableGroup3D debugDrawables;
};

}  // namespace physics

}  // namespace esp
