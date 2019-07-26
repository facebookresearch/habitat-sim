// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <Magnum/Trade/MeshData3D.h>
#include <Magnum/Trade/MeshObjectData3D.h>
#include <Corrade/Containers/Optional.h>
#include <Corrade/Containers/Reference.h>
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

enum MotionType {
    STATIC,
    KINEMATIC,
    DYNAMIC
};

class RigidObject : public scene::SceneNode {
 public:
  RigidObject(scene::SceneNode* parent);

  // TODO (JH) Currently a RigidObject is either a scene
  // or an object, but cannot be both (tracked by _isScene/_isObject_)
  // there is probably a better way to abstract this
  virtual bool initializeScene(std::vector<assets::CollisionMeshData> meshGroup);

  virtual bool initializeObject(
      assets::PhysicsObjectMetaData& metaData,
      physics::PhysicalObjectType objectType,
      std::vector<assets::CollisionMeshData> meshGroup);

  ~RigidObject();

  //! Check whether object is being actively simulated, or sleeping
  virtual bool isActive();

  //! Force interaction
  virtual void applyForce(Magnum::Vector3 force,
                  Magnum::Vector3 relPos);

  // Impulse interaction
  virtual void applyImpulse(Magnum::Vector3 impulse,
                    Magnum::Vector3 relPos);

  //! (Prototype) For visualizing & debugging
  void debugForce(Magnum::SceneGraph::DrawableGroup3D& debugDrawables);

  //! (Prototype) For visualizing & debugging
  void setDebugForce(Magnum::Vector3 force);

  // ==== Transformations ===
  //! Need to overwrite a bunch of functions to update physical states 
  virtual SceneNode& setTransformation(const Magnum::Math::Matrix4<float> transformation);
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

 protected:
  bool initialized_ = false;
  bool isScene_  = false;
  bool isObject_ = false;
  MotionType objectMotionType;

  //! Debugging visualization
  bool debugForce_;
  Magnum::DebugTools::ForceRenderer3D* debugRender_;
  Magnum::Vector3 debugExternalForce_ = Magnum::Vector3(0.0f, 0.0f, 0.0f);

  void getDimensions(assets::CollisionMeshData& meshData, 
      float* x, float* y, float* z);

  //! Needed after changing the pose from Magnum side
  //! Not exposed to end user
  virtual void syncPose();
};

}  // namespace physics
}  // namespace esp
