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



namespace esp {
namespace assets {

class PhysicsManager {
 public:
  explicit PhysicsManager(){};
  ~PhysicsManager() { LOG(INFO) << "Deconstructing PhysicsManager"; }

  // Stores references to a set of drawable elements
  using DrawableGroup = Magnum::SceneGraph::DrawableGroup3D;
  // Convenience typedef for Importer class
  using Importer = Magnum::Trade::AbstractImporter;

  bool initPhysics(scene::SceneNode* scene);
  
  void initObject(Importer& importer,
                  const AssetInfo& info,
                  const MeshMetaData& metaData,
                  Magnum::Trade::MeshData3D& meshData,
                  const std::string& shapeType="TriangleMeshShape");

  void debugSceneGraph(const MagnumObject* root);

  void stepPhysics();
  void nextFrame();

 protected:
  // ==== Inherited from resource manager ===
  std::vector<std::shared_ptr<BaseMesh>> meshes_;
  std::vector<std::shared_ptr<Magnum::GL::Texture2D>> textures_;
  std::vector<std::shared_ptr<Magnum::Trade::PhongMaterialData>> materials_;

  // a dictionary to check if a mesh has been loaded
  std::map<std::string, MeshMetaData> resourceDict_;

  //! Types of supported Shader programs
  enum ShaderType {
    INSTANCE_MESH_SHADER = 0,
    PTEX_MESH_SHADER = 1,
    COLORED_SHADER = 2,
    VERTEX_COLORED_SHADER = 3,
    TEXTURED_SHADER = 4,
  };
  // maps a name to the shader program
  std::map<ShaderType, std::shared_ptr<Magnum::GL::AbstractShaderProgram>>
      shaderPrograms_;
  Magnum::GL::AbstractShaderProgram* getPhysicsEngine(ShaderType type);


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
