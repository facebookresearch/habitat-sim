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
      const assets::AssetInfo& info,
      Magnum::Float mass,
      std::vector<assets::CollisionMeshData> meshGroup,
      btDynamicsWorld& bWorld);

  bool initializeObject(
      const assets::AssetInfo& info,
      Magnum::Float mass,
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
      const Eigen::Ref<const mat4f> transformation) override;
  virtual SceneNode& setTransformation(const Eigen::Ref<const vec3f> position,
                                       const Eigen::Ref<const vec3f> target,
                                       const Eigen::Ref<const vec3f> up) override;
  virtual SceneNode& setTranslation(const Eigen::Ref<const vec3f> vector) override;
  virtual SceneNode& setRotation(const quatf& quaternion) override;

  virtual SceneNode& resetTransformation() override;
  virtual SceneNode& translate(const Eigen::Ref<const vec3f> vector) override;
  virtual SceneNode& translateLocal(const Eigen::Ref<const vec3f> vector) override;

  virtual SceneNode& rotate(float angleInRad,
                            const Eigen::Ref<const vec3f> normalizedAxis) override;
  virtual SceneNode& rotateLocal(float angleInRad,
                                 const Eigen::Ref<const vec3f> normalizedAxis) override;

  virtual SceneNode& rotateX(float angleInRad) override;
  virtual SceneNode& rotateXInDegree(float angleInDeg) override;
  virtual SceneNode& rotateXLocal(float angleInRad) override;
  virtual SceneNode& rotateY(float angleInRad) override;
  virtual SceneNode& rotateYLocal(float angleInRad) override;
  virtual SceneNode& rotateZ(float angleInRad) override;
  virtual SceneNode& rotateZLocal(float angleInRad) override;

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

  //! Needed after changing the pose from Magnum side
  //! Not exposed to end user
  void syncPose();
};

}  // namespace physics
}  // namespace esp
