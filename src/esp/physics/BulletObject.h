#pragma once

#include <Magnum/Trade/MeshData3D.h>
#include <Magnum/Trade/MeshObjectData3D.h>
#include <Corrade/Containers/Optional.h>
#include <Corrade/Containers/Reference.h>
#include <btBulletDynamicsCommon.h>
#include "esp/assets/Asset.h"
#include "esp/assets/BaseMesh.h"
#include "esp/assets/MeshData.h"
#include "esp/assets/GenericInstanceMeshData.h"
#include "esp/assets/FRLInstanceMeshData.h"
#include "esp/core/esp.h"
#include <Magnum/DebugTools/ForceRenderer.h>

namespace esp {
namespace physics {

class BulletRigidObject : public scene::SceneNode {
 public:
  BulletRigidObject(scene::SceneNode* parent);

  // TODO (JH) Currently a BulletRigidObject is either a scene
  // or an object, but cannot be both (tracked by _isScene/_isObject_)
  // there is probably a better way to abstract this
  bool initializeScene(
      const assets::AssetInfo& info,
      Magnum::Float mass,
      std::vector<assets::CollisionMeshData> meshGroup,
      btDynamicsWorld& bWorld);

  bool initializeObject(
      const assets::AssetInfo& info,
      Magnum::Float mass,
      std::vector<assets::CollisionMeshData> meshGroup,
      btDynamicsWorld& bWorld);

  ~BulletRigidObject();
  btRigidBody& rigidBody();
  /* needed after changing the pose from Magnum side */
  void syncPose();

  bool isActive();

  void debugForce(Magnum::SceneGraph::DrawableGroup3D& debugDrawables);

  void setDebugForce(Magnum::Vector3 force);

 private:
  bool initialized_ = false;
  bool isScene_  = false;
  bool isObject_ = false;

  //! Physical scene
  //! Scene data: triangular mesh shape
  std::unique_ptr<btTriangleIndexVertexArray> tivArray_;
  std::vector<std::unique_ptr<btBvhTriangleMeshShape>> bSceneShapes_;
  std::vector<std::unique_ptr<btCollisionObject>> bCollisionBodies_;

  // Physical object
  //! Object data: Convex collision shape
  std::unique_ptr<btCollisionObject> bCollisionBody_;
  std::vector<std::unique_ptr<btConvexHullShape>> bConvexShapes_;
  std::unique_ptr<btCompoundShape> bObjectShape_;
  std::unique_ptr<btRigidBody> rigidBody_;

  //! Magnum Physics Binding
  Magnum::BulletIntegration::MotionState* motionState_;

  float mass_;
  float defaultRestitution_ = 0.1f;
  float defaultMargin_ = 0.01f;
  float defaultLinDamping_ = 0.2f;
  float defaultAngDamping_ = 0.2f;

  //! Debugging visualization
  bool debugForce_;
  Magnum::DebugTools::ForceRenderer3D* debugRender_;
  Magnum::Vector3 debugExternalForce_ = Magnum::Vector3(0.0f, 0.0f, 0.0f);

  void getDimensions(assets::CollisionMeshData& meshData, 
      float* x, float* y, float* z);
};

}  // namespace physics
}  // namespace esp
