// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Timeline.h>

/* Bullet Physics Integration */
#include <Corrade/Containers/Optional.h>
#include <Corrade/Containers/Pointer.h>
#include <Magnum/BulletIntegration/Integration.h>
#include <Magnum/BulletIntegration/MotionState.h>
#include <Magnum/BulletIntegration/DebugDraw.h>
#include <btBulletDynamicsCommon.h>

//#include <Magnum/GL/DefaultFramebuffer.h>
//#include <Magnum/GL/Mesh.h>
//#include <Magnum/GL/Renderer.h>
//#include <Magnum/Math/Constants.h>
//#include <Magnum/MeshTools/Compile.h>
//#include <Magnum/MeshTools/Transform.h>
//#include <Magnum/Primitives/Cube.h>
//#include <Magnum/Primitives/UVSphere.h>
//#include <Magnum/SceneGraph/Camera.h>
//#include <Magnum/SceneGraph/Drawable.h>
//#include <Magnum/SceneGraph/MatrixTransformation3D.h>
//#include <Magnum/SceneGraph/Scene.h>
//#include <Magnum/Shaders/Phong.h>
//#include <Magnum/Trade/MeshData3D.h>


#include "Asset.h"
#include "BaseMesh.h"
//#include "magnum.h"
#include "MeshMetaData.h"
#include "esp/scene/SceneNode.h"
#include "esp/physics/BulletObject.h"


namespace esp {
namespace assets {

class PhysicsManager {
 public:
  explicit PhysicsManager(){};
  ~PhysicsManager() { LOG(INFO) << "Deconstructing PhysicsManager"; }

  bool initPhysics();
  
  void initObject(const AssetInfo& info,
                  const MeshMetaData& metaData,
                  Magnum::Trade::MeshData3D& meshData,
                  physics::BulletRigidObject* physObject,
                  const std::string& shapeType="TriangleMeshShape");

  void debugSceneGraph(const MagnumObject* root);

  void stepPhysics();
  void nextFrame();

 protected:

  void getPhysicsEngine();
  // ==== physics engines ====
  // The world has to live longer than the scene because RigidBody
  // instances have to remove themselves from it on destruction
  scene::SceneNode*                         _scene;
  Magnum::BulletIntegration::DebugDraw      _debugDraw{Magnum::NoCreate};
  btDbvtBroadphase                          _bBroadphase;
  btDefaultCollisionConfiguration           _bCollisionConfig;
  btCollisionDispatcher                     _bDispatcher{&_bCollisionConfig};
  btSequentialImpulseConstraintSolver       _bSolver;
  btDiscreteDynamicsWorld                   _bWorld{&_bDispatcher, &_bBroadphase, &_bSolver, &_bCollisionConfig};

  bool _drawCubes{true}, _drawDebug{true}, _shootBox{true};

  Magnum::Timeline _timeline;
  float _maxSubSteps = 1.0f;
  float _fixedTimeStep = 1.0f / 200.0f;
};


class RigidBody: public scene::SceneNode {
 public:
  RigidBody(scene::SceneNode* parent, Magnum::Float mass, btCollisionShape* bShape, btDynamicsWorld& bWorld);

  ~RigidBody();
  btRigidBody& rigidBody();
  /* needed after changing the pose from Magnum side */
  void syncPose();

 private:
  btDynamicsWorld& _bWorld;
  Magnum::Containers::Pointer<btRigidBody> _bRigidBody;
};



}  // namespace assets

}  // namespace esp
