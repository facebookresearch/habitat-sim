// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <Corrade/Containers/Optional.h>
#include <Corrade/Containers/Reference.h>
#include <Magnum/DebugTools/ForceRenderer.h>
#include "esp/assets/Asset.h"
#include "esp/assets/Attributes.h"
#include "esp/assets/BaseMesh.h"
#include "esp/assets/FRLInstanceMeshData.h"
#include "esp/assets/GenericInstanceMeshData.h"
#include "esp/assets/MeshData.h"
#include "esp/core/esp.h"
#include "esp/scene/SceneNode.h"

namespace esp {
namespace physics {

enum MotionType { STATIC, KINEMATIC, DYNAMIC };

class RigidObject : public scene::SceneNode {
 public:
  RigidObject(scene::SceneNode* parent);

  // TODO (JH) Currently a RigidObject is either a scene
  // or an object, but cannot be both (tracked by _isScene/_isObject_)
  // there is probably a better way to abstract this
  virtual bool initializeScene(
      assets::PhysicsSceneAttributes& physicsSceneAttributes,
      std::vector<assets::CollisionMeshData> meshGroup);

  virtual bool initializeObject(
      assets::PhysicsObjectAttributes& physicsObjectAttributes,
      std::vector<assets::CollisionMeshData> meshGroup);

  ~RigidObject();

  //! Check whether object is being actively simulated, or sleeping
  virtual bool isActive();
  virtual void setActive(){};

  //! Force interaction
  virtual void applyForce(Magnum::Vector3 force, Magnum::Vector3 relPos);
  // Impulse interaction
  virtual void applyImpulse(Magnum::Vector3 impulse, Magnum::Vector3 relPos);

  //! (Prototype) For visualizing & debugging
  void debugForce(Magnum::SceneGraph::DrawableGroup3D& debugDrawables);
  //! (Prototype) For visualizing & debugging
  void setDebugForce(Magnum::Vector3 force);

  virtual bool removeObject();

  // ==== Transformations ===
  //! Need to overwrite a bunch of functions to update physical states
  virtual SceneNode& setTransformation(
      const Magnum::Math::Matrix4<float> transformation);
  virtual SceneNode& setTranslation(const Magnum::Math::Vector3<float> vector);
  virtual SceneNode& setRotation(
      const Magnum::Math::Quaternion<float>& quaternion);

  virtual SceneNode& resetTransformation();
  virtual SceneNode& translate(const Magnum::Math::Vector3<float> vector);
  virtual SceneNode& translateLocal(const Magnum::Math::Vector3<float> vector);

  virtual SceneNode& rotate(const Magnum::Math::Rad<float> angleInRad,
                            const Magnum::Math::Vector3<float> normalizedAxis);
  virtual SceneNode& rotateLocal(
      const Magnum::Math::Rad<float> angleInRad,
      const Magnum::Math::Vector3<float> normalizedAxis);

  virtual SceneNode& rotateX(const Magnum::Math::Rad<float> angleInRad);
  virtual SceneNode& rotateY(const Magnum::Math::Rad<float> angleInRad);
  virtual SceneNode& rotateZ(const Magnum::Math::Rad<float> angleInRad);
  virtual SceneNode& rotateXLocal(const Magnum::Math::Rad<float> angleInRad);
  virtual SceneNode& rotateYLocal(const Magnum::Math::Rad<float> angleInRad);
  virtual SceneNode& rotateZLocal(const Magnum::Math::Rad<float> angleInRad);

  // ==== Getter/Setter functions ===
  //! For kinematic objects they are dummies, for dynamic objects
  //! implemented in physics-engine specific ways
  virtual const double getMass() { return 0.0; }
  virtual const Magnum::Vector3 getCOM() { return Magnum::Vector3(); }
  virtual const Magnum::Vector3 getInertia() { return Magnum::Vector3(); }
  virtual const double getScale() { return 0.0; }
  virtual const double getFrictionCoefficient() { return 0.0; }
  virtual const double getRestitutionCoefficient() { return 0.0; }
  virtual const double getLinearDamping() { return 0.0; }
  virtual const double getAngularDamping() { return 0.0; }

  virtual void setMass(const double mass){};
  virtual void setCOM(const Magnum::Vector3 COM){};
  virtual void setInertia(const Magnum::Vector3 inertia){};
  virtual void setScale(const double scale){};
  virtual void setFrictionCoefficient(const double frictionCoefficient){};
  virtual void setRestitutionCoefficient(const double restitutionCoefficient){};
  virtual void setLinearDamping(const double linearDamping){};
  virtual void setAngularDamping(const double angularDamping){};

  // public Attributes object for user convenience.
  assets::Attributes attributes_;

 protected:
  bool initialized_ = false;
  bool isScene_ = false;
  bool isObject_ = false;
  MotionType objectMotionType_;

  // used only if isObject_
  // assets::PhysicsObjectMetaData physicsObjectMetaData_;

  //! Needed after changing the pose from Magnum side
  //! Not exposed to end user
  virtual void syncPose();
};

}  // namespace physics
}  // namespace esp
