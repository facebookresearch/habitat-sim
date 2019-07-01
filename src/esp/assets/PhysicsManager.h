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

#include "Asset.h"
#include "BaseMesh.h"
#include "GenericInstanceMeshData.h"
#include "MeshMetaData.h"
#include "MeshData.h"
#include "esp/scene/SceneNode.h"
#include "esp/physics/BulletObject.h"


namespace esp {
namespace assets {

class PhysicsManager {
 public:
  explicit PhysicsManager(){};
  virtual ~PhysicsManager();

  bool initPhysics(scene::SceneNode* node,
                   bool do_profile);
  
  //! Initialize object given mesh data
  //! The object could contain several parts
  bool initObject(
      const AssetInfo& info,
      const MeshMetaData& metaData,
      std::vector<CollisionMeshData> meshGroup,
      physics::BulletRigidObject* physObject);

  //! Initialize scene given mesh data
  //! The scene could contain several components
  bool initScene(
      const AssetInfo& info,
      const MeshMetaData& metaData,
      std::vector<CollisionMeshData> meshGroup,
      physics::BulletRigidObject* physObject);

  void debugSceneGraph(const MagnumObject* root);

  void stepPhysics();
  void nextFrame();

  void checkActiveObjects();

 protected:

  void getPhysicsEngine();

  //! Check if mesh primitive type is valid for bullet physics engine
  bool isMeshPrimitiveValid(CollisionMeshData& meshData);

  //! ==== physics engines ====
  //! The world has to live longer than the scene because RigidBody
  //! instances have to remove themselves from it on destruction
  Magnum::BulletIntegration::DebugDraw      _debugDraw{Magnum::NoCreate};
  btDbvtBroadphase*                         _bBroadphase;
  btDefaultCollisionConfiguration*          _bCollisionConfig;
  btCollisionDispatcher*                    _bDispatcher;
  //! btCollisionDispatcher*                    _bDispatcher;
  btSequentialImpulseConstraintSolver*      _bSolver;
  //! btDiscreteDynamicsWorld                   
  //      _bWorld{&_bDispatcher, &_bBroadphase, &_bSolver, &_bCollisionConfig};
  btDiscreteDynamicsWorld*                  _bWorld;

  scene::SceneNode* physicsNode = nullptr;

  bool _initialized = false;
  bool _do_profile = false;
  float _total_time = 0.0f;
  int _total_frames = 0;

  Magnum::Timeline _timeline;
  int _maxSubSteps = 10;
  float _fixedTimeStep = 1.0f / 240.0f;
};

}  // namespace assets

}  // namespace esp
