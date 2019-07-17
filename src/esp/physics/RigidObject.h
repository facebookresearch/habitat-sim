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
#include "esp/assets/PhysicsObjectMetaData.h"
#include "esp/assets/FRLInstanceMeshData.h"
#include "esp/core/esp.h"
#include "esp/scene/SceneNode.h"
#include <Magnum/DebugTools/ForceRenderer.h>


namespace esp {
namespace physics {

class RigidObject : public scene::SceneNode {
 public:
  RigidObject(scene::SceneNode* parent);

  // TODO (JH) Currently a RigidObject is either a scene
  // or an object, but cannot be both (tracked by _isScene/_isObject_)
  // there is probably a better way to abstract this
  bool initializeScene(
      std::vector<assets::CollisionMeshData> meshGroup,
      btDynamicsWorld& bWorld);

  bool initializeObject(
      assets::PhysicsObjectMetaData& metaData,
      physics::PhysicalObjectType objectType,
      std::vector<assets::CollisionMeshData> meshGroup,
      btDynamicsWorld& bWorld);

  ~RigidObject();

  //! Check whether object is being actively simulated, or sleeping
  bool isActive();

  //! Force interaction
  void applyForce(Magnum::Vector3 force,
                  Magnum::Vector3 relPos);

  // Impulse interaction
  void applyImpulse(Magnum::Vector3 impulse,
                    Magnum::Vector3 relPos);

  //! (Prototype) For visualizing & debugging
  void debugForce(Magnum::SceneGraph::DrawableGroup3D& debugDrawables);

  //! (Prototype) For visualizing & debugging
  void setDebugForce(Magnum::Vector3 force);

  // ==== Transformations ===
  //! Need to overwrite a bunch of functions to update physical states 
  virtual SceneNode& setTransformation(
      const Magnum::Math::Matrix4<float> transformation);
  virtual SceneNode& setTranslation(const Magnum::Math::Vector3<float> vector);
  virtual SceneNode& setRotation(const Magnum::Math::Quaternion<float>& quaternion);

  virtual SceneNode& resetTransformation();
  virtual SceneNode& translate(const Magnum::Math::Vector3<float> vector);
  virtual SceneNode& translateLocal(const Magnum::Math::Vector3<float> vector);

  virtual SceneNode& rotate(const Magnum::Math::Rad<float> angleInRad,
                            const Magnum::Math::Vector3<float> normalizedAxis);
  virtual SceneNode& rotateLocal(const Magnum::Math::Rad<float> angleInRad,
                                 const Magnum::Math::Vector3<float> normalizedAxis);

  virtual SceneNode& rotateX(const Magnum::Math::Rad<float> angleInRad);
  virtual SceneNode& rotateY(const Magnum::Math::Rad<float> angleInRad);
  virtual SceneNode& rotateZ(const Magnum::Math::Rad<float> angleInRad);
  virtual SceneNode& rotateXLocal(const Magnum::Math::Rad<float> angleInRad);
  virtual SceneNode& rotateYLocal(const Magnum::Math::Rad<float> angleInRad);
  virtual SceneNode& rotateZLocal(const Magnum::Math::Rad<float> angleInRad);

 private:
  bool initialized_ = false;
  bool isScene_  = false;
  bool isObject_ = false;

  //! Physical scene
  //! Scene data: triangular mesh shape
  //! All components are stored as a vector of bCollisionBody_
  std::unique_ptr<btTriangleIndexVertexArray>          bSceneArray_;
  std::vector<std::unique_ptr<btBvhTriangleMeshShape>> bSceneShapes_;
  std::vector<std::unique_ptr<btCollisionObject>>      bSceneCollisionObjects_;

  // Physical object
  //! Object data: Composite convex collision shape
  //! All components are wrapped into one rigidBody_
  std::vector<std::unique_ptr<btConvexHullShape>>      bObjectConvexShapes_;
  std::unique_ptr<btCompoundShape>                     bObjectShape_;
  std::unique_ptr<btRigidBody>                         bObjectRigidBody_;
  Magnum::BulletIntegration::MotionState*              bObjectMotionState_;

  //! Debugging visualization
  bool debugForce_;
  Magnum::DebugTools::ForceRenderer3D* debugRender_;
  Magnum::Vector3 debugExternalForce_ = Magnum::Vector3(0.0f, 0.0f, 0.0f);

  void getDimensions(assets::CollisionMeshData& meshData, 
      float* x, float* y, float* z);

  //! Needed after changing the pose from Magnum side
  //! Not exposed to end user
  void syncPose();
};

}  // namespace physics
}  // namespace esp
