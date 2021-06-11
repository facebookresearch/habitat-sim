// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_PHYSICSOBJECTBASE_H_
#define ESP_PHYSICS_PHYSICSOBJECTBASE_H_

#include <Corrade/Containers/Optional.h>
#include <Corrade/Containers/Reference.h>
#include "esp/assets/ResourceManager.h"
#include "esp/core/RigidState.h"
#include "esp/physics/CollisionGroupHelper.h"

/** @file
 * @brief Class @ref physics::PhysicsObjectBase is the base class for any
 * physics-based construct, and holds basic accounting info and accessors, along
 * with scene node access.
 */

namespace esp {

namespace physics {

/**
 * @brief Motion type of a @ref RigidObject.
 * Defines its treatment by the simulator and operations which can be performed
 * on it.
 */
enum class MotionType {
  /**
   * Refers to an error (such as a query to non-existing object) or an
   * unknown/unspecified value.
   */
  UNDEFINED = -1,

  /**
   * The object is not expected to move and should not allow kinematic updates.
   * Likely treated as static collision geometry. See @ref
   * RigidObjectType::SCENE.
   */
  STATIC,

  /**
   * The object is expected to move kinematically, but is not simulated. Default
   * behavior of @ref RigidObject with no physics simulator defined.
   */
  KINEMATIC,

  /**
   * The object is simulated and can, but should not be, updated kinematically .
   * Default behavior of @ref RigidObject with a physics simulator defined. See
   * @ref BulletRigidObject.
   */
  DYNAMIC

};

class PhysicsObjectBase : public Magnum::SceneGraph::AbstractFeature3D {
 public:
  PhysicsObjectBase(scene::SceneNode* bodyNode,
                    int objectId,
                    const assets::ResourceManager& resMgr)
      : Magnum::SceneGraph::AbstractFeature3D(*bodyNode),
        objectId_(objectId),
        resMgr_(resMgr) {}

  ~PhysicsObjectBase() override = default;

  /**
   * @brief Get the scene node being attached to.
   */
  scene::SceneNode& node() { return object(); }
  const scene::SceneNode& node() const { return object(); }

  // Overloads to avoid confusion
  scene::SceneNode& object() {
    return static_cast<scene::SceneNode&>(
        Magnum::SceneGraph::AbstractFeature3D::object());
  }
  const scene::SceneNode& object() const {
    return static_cast<const scene::SceneNode&>(
        Magnum::SceneGraph::AbstractFeature3D::object());
  }
  /**
   * @brief Get the @ref physics::MotionType of the object. See @ref
   * setMotionType.
   * @return The object's current @ref MotionType.
   */
  MotionType getMotionType() const { return objectMotionType_; }

  /**
   * @brief Set the @ref MotionType of the object. If the construct is a @ref
   * physics::RigidStage, it can only be @ref
   * physics::MotionType::STATIC. If the object is
   * @ref physics::RigidObject it can also be set to @ref
   * physics::MotionType::KINEMATIC. Only if a dervied @ref
   * physics::PhysicsManager implementing dynamics is in use can the object
   * be set to @ref physics::MotionType::DYNAMIC.
   * @param mt The desirved @ref MotionType.
   */
  virtual void setMotionType(MotionType mt) = 0;

  /**
   * @brief Get object's ID
   */
  int getObjectID() const { return objectId_; }

  /**
   * @brief Object name, to faciliate access.
   */
  std::string getObjectName() const { return objectName_; }
  void setObjectName(const std::string& name) { objectName_ = name; }

  /**
   * @brief Get a const reference to this physica object's root SceneNode for
   * info query purposes.
   * @return Const reference to the object's scene node.
   */
  const scene::SceneNode& getSceneNode() const { return node(); }

  /**
   * @brief Check whether object is being actively simulated, or sleeping.
   * Kinematic objects are always active, but derived dynamics implementations
   * may not be.  NOTE: no active objects without a physics engine...
   * (kinematics don't count)
   * @return true if active, false otherwise.
   */
  virtual bool isActive() const { return false; }

  /**
   * @brief Set an object as being actively simulated or sleeping.
   * Kinematic objects are always active, but derived dynamics implementations
   * may not be.
   *
   * @param active Whether to activate or sleep the object
   */
  virtual void setActive(CORRADE_UNUSED bool active) {}

  /**
   * @brief Return result of a discrete contact test between the object and
   * collision world.
   *
   * See @ref SimulationContactResultCallback
   * @return Whether or not the object is in contact with any other collision
   * enabled objects.
   */
  virtual bool contactTest() { return false; }

  /**
   * @brief Manually set the collision group for an object.
   * @param group The desired CollisionGroup for the object.
   */
  virtual void overrideCollisionGroup(CORRADE_UNUSED CollisionGroup group) {}

  /**
   * @brief Set the light setup of this rigid.
   * @param lightSetupKey @ref gfx::LightSetup key
   */
  void setLightSetup(const std::string& lightSetupKey) {
    gfx::setLightSetupForSubTree(node(), lightSetupKey);
  }

  // ==== Transformations ===

  /** @brief Set the 4x4 transformation matrix of the object kinematically.
   * Calling this during simulation of a @ref physics::MotionType::DYNAMIC
   * object is not recommended.
   * @param transformation The desired 4x4 transform of the object.
   */
  virtual void setTransformation(const Magnum::Matrix4& transformation) {
    if (objectMotionType_ != MotionType::STATIC) {
      node().setTransformation(transformation);
      syncPose();
    }
  }

  /**
   * @brief Get the 4x4 transformation matrix of the object
   */
  virtual Magnum::Matrix4 getTransformation() const {
    return node().transformation();
  }

  /**
   * @brief Set the 3D position of the object kinematically.
   * Calling this during simulation of a @ref physics::MotionType::DYNAMIC
   * object is not recommended.
   * @param vector The desired 3D position of the object.
   */
  virtual void setTranslation(const Magnum::Vector3& vector) {
    if (objectMotionType_ != MotionType::STATIC) {
      node().setTranslation(vector);
      syncPose();
    }
  }

  /**
   * @brief Get the 3D position of the object.
   */
  virtual Magnum::Vector3 getTranslation() const {
    return node().translation();
  }

  /**
   * @brief Set the orientation of the object kinematically.
   * Calling this during simulation of a @ref physics::MotionType::DYNAMIC
   * object is not recommended.
   * @param quaternion The desired orientation of the object.
   */
  virtual void setRotation(const Magnum::Quaternion& quaternion) {
    if (objectMotionType_ != MotionType::STATIC) {
      node().setRotation(quaternion);
      syncPose();
    }
  }

  /**
   * @brief Get the orientation of the object.
   */
  virtual Magnum::Quaternion getRotation() const { return node().rotation(); }

  /**
   * @brief Set the rotation and translation of the object.
   */
  virtual void setRigidState(const core::RigidState& rigidState) {
    setTranslation(rigidState.translation);
    setRotation(rigidState.rotation);
  }

  /**
   * @brief Get the rotation and translation of the object.
   */
  virtual core::RigidState getRigidState() {
    return core::RigidState(node().rotation(), node().translation());
  }

  /**
   * @brief Reset the transformation of the object.
   * !!NOT IMPLEMENTED!!
   */
  virtual void resetTransformation() {
    if (objectMotionType_ != MotionType::STATIC) {
      node().resetTransformation();
      syncPose();
    }
  }

  /** @brief Modify the 3D position of the object kinematically by translation.
   * Calling this during simulation of a @ref physics::MotionType::DYNAMIC
   * object is not recommended.
   * @param vector The desired 3D vector by which to translate the object.
   */
  virtual void translate(const Magnum::Vector3& vector) {
    if (objectMotionType_ != MotionType::STATIC) {
      node().translate(vector);
      syncPose();
    }
  }

  /**
   * @brief Modify the 3D position of the object kinematically by translation
   * with a vector defined in the object's local coordinate system. Calling this
   * during simulation of a @ref physics::MotionType::DYNAMIC object is not
   * recommended.
   * @param vector The desired 3D vector in the object's ocal coordiante system
   * by which to translate the object.
   */
  virtual void translateLocal(const Magnum::Vector3& vector) {
    if (objectMotionType_ != MotionType::STATIC) {
      node().translateLocal(vector);
      syncPose();
    }
  }

  /**
   * @brief Modify the orientation of the object kinematically by applying an
   * axis-angle rotation to it. Calling this during simulation of a @ref
   * MotionType::DYNAMIC object is not recommended.
   * @param angleInRad The angle of rotation in radians.
   * @param normalizedAxis The desired unit vector axis of rotation.
   */
  virtual void rotate(const Magnum::Rad angleInRad,
                      const Magnum::Vector3& normalizedAxis) {
    if (objectMotionType_ != MotionType::STATIC) {
      node().rotate(angleInRad, normalizedAxis);
      syncPose();
    }
  }

  /**
   * @brief Modify the orientation of the object kinematically by applying an
   * axis-angle rotation to it in the local coordinate system. Calling this
   * during simulation of a @ref physics::MotionType::DYNAMIC object is not
   * recommended.
   * @param angleInRad The angle of rotation in radians.
   * @param normalizedAxis The desired unit vector axis of rotation in the local
   * coordinate system.
   */
  virtual void rotateLocal(const Magnum::Rad angleInRad,
                           const Magnum::Vector3& normalizedAxis) {
    if (objectMotionType_ != MotionType::STATIC) {
      node().rotateLocal(angleInRad, normalizedAxis);
      syncPose();
    }
  }

  /**
   * @brief Modify the orientation of the object kinematically by applying a
   * rotation to it about the global X axis. Calling this during simulation of a
   * @ref physics::MotionType::DYNAMIC object is not recommended.
   * @param angleInRad The angle of rotation in radians.
   */
  virtual void rotateX(const Magnum::Rad angleInRad) {
    if (objectMotionType_ != MotionType::STATIC) {
      node().rotateX(angleInRad);
      syncPose();
    }
  }

  /**
   * @brief Modify the orientation of the object kinematically by applying a
   * rotation to it about the global Y axis. Calling this during simulation of a
   * @ref physics::MotionType::DYNAMIC object is not recommended.
   * @param angleInRad The angle of rotation in radians.
   */
  virtual void rotateY(const Magnum::Rad angleInRad) {
    if (objectMotionType_ != MotionType::STATIC) {
      node().rotateY(angleInRad);
      syncPose();
    }
  }

  /**
   * @brief Modify the orientation of the object kinematically by applying a
   * rotation to it about the global Z axis. Calling this during simulation of a
   * @ref physics::MotionType::DYNAMIC object is not recommended.
   * @param angleInRad The angle of rotation in radians.
   */
  virtual void rotateZ(const Magnum::Rad angleInRad) {
    if (objectMotionType_ != MotionType::STATIC) {
      node().rotateZ(angleInRad);
      syncPose();
    }
  }

  /**
   * @brief Modify the orientation of the object kinematically by applying a
   * rotation to it about the local X axis. Calling this during simulation of a
   * @ref physics::MotionType::DYNAMIC object is not recommended.
   * @param angleInRad The angle of rotation in radians.
   */
  virtual void rotateXLocal(const Magnum::Rad angleInRad) {
    if (objectMotionType_ != MotionType::STATIC) {
      node().rotateXLocal(angleInRad);
      syncPose();
    }
  }

  /**
   * @brief Modify the orientation of the object kinematically by applying a
   * rotation to it about the local Y axis. Calling this during simulation of a
   * @ref physics::MotionType::DYNAMIC object is not recommended.
   * @param angleInRad The angle of rotation in radians.
   */
  virtual void rotateYLocal(const Magnum::Rad angleInRad) {
    if (objectMotionType_ != MotionType::STATIC) {
      node().rotateYLocal(angleInRad);
      syncPose();
    }
  }

  /**
   * @brief Modify the orientation of the object kinematically by applying a
   * rotation to it about the local Z axis. Calling this during simulation of a
   * @ref physics::MotionType::DYNAMIC object is not recommended.
   * @param angleInRad The angle of rotation in radians.
   */
  virtual void rotateZLocal(const Magnum::Rad angleInRad) {
    if (objectMotionType_ != MotionType::STATIC) {
      node().rotateZLocal(angleInRad);
      syncPose();
    }
  }

  virtual void deferUpdate() { isDeferringUpdate_ = true; }

  /**
   * @brief update the SceneNode state to match the simulation state
   */
  virtual void updateNodes(CORRADE_UNUSED bool force = false) {
    isDeferringUpdate_ = false;
  }

  /**
   * @brief Set or reset the object's state using the object's specified @p
   * sceneInstanceAttributes_.
   * @param defaultCOMCorrection The default value of whether COM-based
   * translation correction needs to occur.
   */
  virtual void resetStateFromSceneInstanceAttr(
      CORRADE_UNUSED bool defaultCOMCorrection = false) = 0;

  /**
   * @brief Set this object's @ref
   * metadata::attributes::SceneObjectInstanceAttributes used to place the
   * object within the scene.
   * @param instanceAttr The @ref
   * metadata::attributes::SceneObjectInstanceAttributes used to place this
   * object in the scene.
   */

  template <class U>
  void setSceneInstanceAttr(std::shared_ptr<U> instanceAttr) {
    _sceneInstanceAttributes = std::move(instanceAttr);
  }  // setSceneInstanceAttr

  /**
   * @brief Get pointers to this physics object's visual scene nodes
   * @return vector of pointers to the object's visual scene nodes.
   */
  virtual std::vector<scene::SceneNode*> getVisualSceneNodes() const = 0;

  core::Configuration::ptr getUserAttributes() const { return userAttributes_; }
  void setUserAttributes(core::Configuration::ptr attr) {
    userAttributes_ = std::move(attr);
  }

 protected:
  /** @brief Accessed internally. Get an appropriately cast copy of the @ref
   * metadata::attributes::SceneObjectInstanceAttributes used to place the
   * object within the scene.
   * @return A copy of the initialization template used to create this object
   * instance or nullptr if no template exists.
   */
  template <class T>
  std::shared_ptr<T> getSceneInstanceAttrInternal() const {
    if (!_sceneInstanceAttributes) {
      return nullptr;
    }
    return T::create(*(static_cast<T*>(_sceneInstanceAttributes.get())));
  }

  /**
   * @brief Used to synchronize other simulator's notion of the object state
   * after it was changed kinematically. Must be called automatically on
   * kinematic updates.
   */
  virtual void syncPose() { return; }

  /**
   * @brief if true visual nodes are not updated from physics simulation such
   * that the SceneGraph is not polluted during render
   */
  bool isDeferringUpdate_ = false;

  /**
   * @brief An assignable name for this object.
   */
  std::string objectName_;

  /**
   * @brief The @ref MotionType of the object. Determines what operations can
   * be performed on this object. */
  MotionType objectMotionType_{MotionType::UNDEFINED};

  /**
   * @brief Access for the object to its own PhysicsManager id.
   */
  int objectId_;

  /**
   * @brief Reference to the ResourceManager for internal access to the
   * object's asset data.
   */
  const assets::ResourceManager& resMgr_;

  /**
   * @brief Stores user-defined attributes for this object, held as a smart
   * pointer to a @ref esp::core::Configuration. These attributes are not
   * internally processed by habitat, but provide a "scratch pad" for the user
   * to access and save important information and metadata.
   */
  core::Configuration::ptr userAttributes_{};

 private:
  /**
   * @brief This object's instancing attributes, if any were used during its
   * creation.
   */
  std::shared_ptr<metadata::attributes::SceneObjectInstanceAttributes>
      _sceneInstanceAttributes = nullptr;

 public:
  ESP_SMART_POINTERS(PhysicsObjectBase)
};  // class PhysicsObjectBase

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_PHYSICSOBJECTBASE_H_
