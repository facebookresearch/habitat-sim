// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_RIGIDBASE_H_
#define ESP_PHYSICS_RIGIDBASE_H_

#include "esp/assets/Asset.h"
#include "esp/assets/BaseMesh.h"
#include "esp/assets/GenericSemanticMeshData.h"
#include "esp/core/Esp.h"
#include "esp/metadata/attributes/AbstractObjectAttributes.h"
#include "esp/metadata/attributes/AttributesBase.h"
#include "esp/physics/PhysicsObjectBase.h"

/** @file
 * @brief Class @ref Rigidbase
 */

namespace esp {
namespace assets {
class ResourceManager;
}

namespace physics {

/**
 * @brief This class specifies the functionality expected of rigid objects and
 * stages, particularly with regard to dynamic simulation, if such a library,
 * such as bullet, is available.
 */
class RigidBase : public esp::physics::PhysicsObjectBase {
 public:
  /**
   * @brief Constructor for a @ref RigidBase.
   * @param rigidBodyNode Pointer to the node to be used for this rigid.
   * @param objectId the desired ID for this rigid construct.
   * @param resMgr a reference to @ref esp::assets::ResourceManager
   */
  RigidBase(scene::SceneNode* rigidBodyNode,
            int objectId,
            const assets::ResourceManager& resMgr)
      : PhysicsObjectBase(rigidBodyNode, objectId, resMgr),
        visualNode_(&rigidBodyNode->createChild()) {}

  /**
   * @brief Virtual destructor for a @ref RigidBase.
   */
  ~RigidBase() override = default;

  /**
   * @brief Initializes the @ref RigidObject or @ref
   * esp::physics::RigidStage that inherits from this class.  This is overridden
   * @param initAttributes The template structure defining relevant physical
   * parameters for this object
   * @return true if initialized successfully, false otherwise.
   */
  virtual bool initialize(
      std::shared_ptr<metadata::attributes::AbstractObjectAttributes>
          initAttributes) = 0;

  /**
   * @brief Finalize the creation of @ref RigidObject or @ref
   * esp::physics::RigidStage that inherits from this class.
   * @return whether successful finalization.
   */
  virtual bool finalizeObject() = 0;

 private:
  /**
   * @brief Finalize the initialization of this @ref RigidBase.
   * This is overridden by inheriting objects
   * @return true if initialized successfully, false otherwise.
   */
  virtual bool initialization_LibSpecific() = 0;
  /**
   * @brief any physics-lib-specific finalization code that needs to be run
   * after @ref RigidObject or @ref RigidStage is
   * created.
   * @return whether successful finalization.
   */
  virtual bool finalizeObject_LibSpecific() = 0;

 public:
  bool getCollidable() const { return isCollidable_; }
  /**
   * @brief Set a rigid as collidable or not. Derived implementations handle the
   * specifics of modifying the collision properties.
   */
  virtual void setCollidable(CORRADE_UNUSED bool collidable){};

  /**
   * @brief Apply a force to an object through a dervied dynamics
   * implementation. Does nothing for @ref esp::physics::MotionType::STATIC and
   * @ref esp::physics::MotionType::KINEMATIC objects.
   * @param force The desired force on the object in the global coordinate
   * system.
   * @param relPos The desired location of force application in the global
   * coordinate system relative to the object's center of mass.
   */
  virtual void applyForce(CORRADE_UNUSED const Magnum::Vector3& force,
                          CORRADE_UNUSED const Magnum::Vector3& relPos) {
    ESP_ERROR()
        << "Not implemented. Install with --bullet to use this feature.";
  }

  /**
   * @brief Apply an impulse to an object through a dervied dynamics
   * implementation. Directly modifies the object's velocity without requiring
   * integration through simulation. Does nothing for @ref
   * esp::physics::MotionType::STATIC and @ref
   * esp::physics::MotionType::KINEMATIC objects.
   * @param impulse The desired impulse on the object in the global coordinate
   * system.
   * @param relPos The desired location of impulse application in the global
   * coordinate system relative to the object's center of mass.
   */
  virtual void applyImpulse(CORRADE_UNUSED const Magnum::Vector3& impulse,
                            CORRADE_UNUSED const Magnum::Vector3& relPos) {
    ESP_ERROR()
        << "Not implemented. Install with --bullet to use this feature.";
  }

  /**
   * @brief Apply an internal torque to an object through a dervied dynamics
   * implementation. Does nothing for @ref esp::physics::MotionType::STATIC and
   * @ref esp::physics::MotionType::KINEMATIC objects.
   * @param torque The desired torque on the object in the local coordinate
   * system.
   */
  virtual void applyTorque(CORRADE_UNUSED const Magnum::Vector3& torque) {
    ESP_ERROR()
        << "Not implemented. Install with --bullet to use this feature.";
  }
  /**
   * @brief Apply an internal impulse torque to an object through a dervied
   * dynamics implementation. Does nothing for @ref
   * esp::physics::MotionType::STATIC and @ref
   * esp::physics::MotionType::KINEMATIC objects.
   * @param impulse The desired impulse torque on the object in the local
   * coordinate system. Directly modifies the object's angular velocity without
   * requiring integration through simulation.
   */
  virtual void applyImpulseTorque(
      CORRADE_UNUSED const Magnum::Vector3& impulse) {
    ESP_ERROR()
        << "Not implemented. Install with --bullet to use this feature.";
  }

  // ==== Getter/Setter functions ===

  //! For kinematic objects they are dummies, for dynamic objects
  //! implemented in physics-engine specific ways

  /** @brief Get the scalar angular damping coefficient of the object. Only used
   * for dervied dynamic implementations of @ref RigidObject.
   * @return The scalar angular damping coefficient of the object.
   */
  virtual double getAngularDamping() const { return 0.0; }

  /** @brief Set the scalar angular damping coefficient for the object. Only
   * used for dervied dynamic implementations of @ref RigidObject.
   * @param angDamping The new scalar angular damping coefficient for the
   * object.
   */
  virtual void setAngularDamping(CORRADE_UNUSED const double angDamping) {}

  /**
   * @brief Virtual angular velocity getter for an object.
   *
   * Returns zero for default @ref esp::physics::MotionType::KINEMATIC or @ref
   * esp::physics::MotionType::STATIC objects.
   * @return Angular velocity vector corresponding to world unit axis angles.
   */
  virtual Magnum::Vector3 getAngularVelocity() const {
    return Magnum::Vector3();
  };

  /** @brief Virtual angular velocity setter for an object.
   *
   * Does nothing for default @ref esp::physics::MotionType::KINEMATIC or @ref
   * esp::physics::MotionType::STATIC objects.
   * @param angVel Angular velocity vector corresponding to world unit axis
   * angles.
   */
  virtual void setAngularVelocity(
      CORRADE_UNUSED const Magnum::Vector3& angVel) {}

  /** @brief Get the center of mass (COM) of the object.
   * @return Object 3D center of mass in the global coordinate system.
   * @todo necessary for @ref esp::physics::MotionType::KINEMATIC?
   */
  virtual Magnum::Vector3 getCOM() const {
    const Magnum::Vector3 com = Magnum::Vector3();
    return com;
  }
  /** @brief Set the center of mass (COM) of the object.
   * @param COM Object 3D center of mass in the local coordinate system.
   * @todo necessary for @ref esp::physics::MotionType::KINEMATIC?
   */
  virtual void setCOM(CORRADE_UNUSED const Magnum::Vector3& COM) {}

  /** @brief Get the scalar friction coefficient of the object. Only used for
   * dervied dynamic implementations of @ref RigidObject.
   * @return The scalar friction coefficient of the object.
   */
  virtual double getFrictionCoefficient() const { return 0.0; }

  /** @brief Set the scalar friction coefficient of the object. Only used for
   * dervied dynamic implementations of @ref RigidObject.
   * @param frictionCoefficient The new scalar friction coefficient of the
   * object.
   */
  virtual void setFrictionCoefficient(
      CORRADE_UNUSED const double frictionCoefficient) {}

  /** @brief Get the scalar rolling friction coefficient of the object. Only
   * used for dervied dynamic implementations of @ref RigidObject.
   * @return The scalar rolling friction coefficient of the object. Damps
   * angular velocity about axis orthogonal to the contact normal to prevent
   * rounded shapes from rolling forever.
   */
  virtual double getRollingFrictionCoefficient() const { return 0.0; }

  /** @brief Set the scalar rolling friction coefficient of the object. Only
   * used for dervied dynamic implementations of @ref RigidObject.
   * @param rollingFrictionCoefficient The new scalar rolling friction
   * coefficient of the object. Damps angular velocity about axis orthogonal to
   * the contact normal to prevent rounded shapes from rolling forever.
   */
  virtual void setRollingFrictionCoefficient(
      CORRADE_UNUSED const double rollingFrictionCoefficient) {}

  /** @brief Get the scalar spinning friction coefficient of the object. Only
   * used for dervied dynamic implementations of @ref RigidObject.
   * @return The scalar spinning friction coefficient of the object. Damps
   * angular velocity about the contact normal.
   */
  virtual double getSpinningFrictionCoefficient() const { return 0.0; }

  /** @brief Set the scalar spinning friction coefficient of the object. Only
   * used for dervied dynamic implementations of @ref RigidObject.
   * @param spinningFrictionCoefficient The new scalar friction coefficient of
   * the object. Damps angular velocity about the contact normal.
   */
  virtual void setSpinningFrictionCoefficient(
      CORRADE_UNUSED const double spinningFrictionCoefficient) {}

  /** @brief Get the 3x3 inertia matrix for an object.
   * @return The object's 3x3 inertia matrix.
   * @todo provide a setter for the full 3x3 inertia matrix. Not all
   * implementations will provide this option.
   */
  virtual Magnum::Matrix3 getInertiaMatrix() const {
    const Magnum::Matrix3 inertia = Magnum::Matrix3();
    return inertia;
  }

  /** @brief Get the diagonal of the inertia matrix for an object.
   * If an object is aligned with its principle axii of inertia, the 3x3 inertia
   * matrix can be reduced to a diagonal. See @ref
   * RigidObject::setInertiaVector.
   * @return The diagonal of the object's inertia matrix.
   */
  virtual Magnum::Vector3 getInertiaVector() const {
    const Magnum::Vector3 inertia = Magnum::Vector3();
    return inertia;
  }

  /** @brief Set the diagonal of the inertia matrix for the object.
   * If an object is aligned with its principle axii of inertia, the 3x3 inertia
   * matrix can be reduced to a diagonal.
   * @param inertia The new diagonal for the object's inertia matrix.
   */
  virtual void setInertiaVector(CORRADE_UNUSED const Magnum::Vector3& inertia) {
  }

  /**
   * @brief Returns the @ref metadata::attributes::SceneObjectInstanceAttributes
   * used to place this rigid object in the scene.
   * @return a read-only copy of the scene instance attributes used to place
   * this object in the scene.
   */
  std::shared_ptr<const metadata::attributes::SceneObjectInstanceAttributes>
  getInitObjectInstanceAttr() const {
    return PhysicsObjectBase::getInitObjectInstanceAttrInternal<
        const metadata::attributes::SceneObjectInstanceAttributes>();
  }

  /**
   * @brief Return a @ref
   * metadata::attributes::SceneObjectInstanceAttributes reflecting the current
   * state of this Rigid.
   * @return a @ref metadata::attributes::SceneObjectInstanceAttributes
   * reflecting this rigid
   */
  std::shared_ptr<metadata::attributes::SceneObjectInstanceAttributes>
  getCurrentStateInstanceAttr() {
    // retrieve copy of initial SceneObjectInstanceAttributes with appropriate
    // fields updated based on current state
    return PhysicsObjectBase::getCurrentObjectInstanceAttrInternal<
        metadata::attributes::SceneObjectInstanceAttributes>();
  }

  /** @brief Get a copy of the template used to initialize this object
   * or scene.
   * @return A copy of the initialization template used to create this object
   * instance or nullptr if no template exists.
   */
  template <class T>
  std::shared_ptr<T> getInitializationAttributes() const {
    if (!initializationAttributes_) {
      return nullptr;
    }
    return T::create(*(static_cast<T*>(initializationAttributes_.get())));
  }

  /** @brief Get the scalar linear damping coefficient of the object. Only used
   * for dervied dynamic implementations of @ref RigidObject.
   * @return The scalar linear damping coefficient of the object.
   */
  virtual double getLinearDamping() const { return 0.0; }

  /** @brief Set the scalar linear damping coefficient of the object. Only used
   * for dervied dynamic implementations of @ref RigidObject.
   * @param linDamping The new scalar linear damping coefficient of the object.
   */
  virtual void setLinearDamping(CORRADE_UNUSED const double linDamping) {}

  /**
   * @brief Virtual linear velocity getter for an object.
   *
   * Returns zero for default @ref esp::physics::MotionType::KINEMATIC or @ref
   * esp::physics::MotionType::STATIC objects.
   * @return Linear velocity of the object.
   */
  virtual Magnum::Vector3 getLinearVelocity() const {
    return Magnum::Vector3();
  };

  /**
   * @brief Virtual linear velocity setter for an object.
   *
   * Does nothing for default @ref esp::physics::MotionType::KINEMATIC or @ref
   * esp::physics::MotionType::STATIC objects.
   * @param linVel Linear velocity to set.
   */
  virtual void setLinearVelocity(CORRADE_UNUSED const Magnum::Vector3& linVel) {
  }

  /** @brief Get the mass of the object. Only used for dervied dynamic
   * implementations of @ref RigidObject.
   * @return The mass of the object.
   */
  virtual double getMass() const { return 0.0; }

  /** @brief Set the mass of the object. Only used for dervied dynamic
   * implementations of @ref RigidObject.
   * @param mass The new mass of the object.
   */
  virtual void setMass(CORRADE_UNUSED const double mass) {}

  /** @brief Get the scalar coefficient of restitution  of the object. Only used
   * for dervied dynamic implementations of @ref RigidObject.
   * @return The scalar coefficient of restitution  of the object.
   */
  virtual double getRestitutionCoefficient() const { return 0.0; }

  /** @brief Set the scalar coefficient of restitution of the object. Only used
   * for dervied dynamic implementations of @ref RigidObject.
   * @param restitutionCoefficient The new scalar coefficient of restitution of
   * the object.
   */
  virtual void setRestitutionCoefficient(
      CORRADE_UNUSED const double restitutionCoefficient) {}

  /** @brief Get the scale of the object set during initialization.
   * @return The scaling for the object relative to its initially loaded meshes.
   */
  virtual Magnum::Vector3 getScale() const {
    return initializationAttributes_->getScale();
  }

  /**
   * @brief Get the semantic ID for this object.
   */
  int getSemanticId() const { return visualNode_->getSemanticId(); }

  /**
   * @brief Set the @ref esp::scene::SceneNode::semanticId_ for all visual nodes
   * belonging to the object.
   * @param semanticId The desired semantic id for the object.
   */
  void setSemanticId(uint32_t semanticId) {
    for (auto* node : visualNodes_) {
      node->setSemanticId(semanticId);
    }
  }

  /**
   * @brief Get pointers to this rigid's visual SceneNodes.
   * @return vector of pointers to the rigid's visual scene nodes.
   */
  std::vector<scene::SceneNode*> getVisualSceneNodes() const override {
    return visualNodes_;
  }

  /**
   * @brief The @ref SceneNode of a bounding box debug drawable. If nullptr, BB
   * drawing is off. See @ref setObjectBBDraw().
   */
  scene::SceneNode* BBNode_ = nullptr;

  /**
   * @brief All Drawable components are children of this node.
   *
   * Note that the transformation of this node is a composition of rotation and
   * translation as scaling is applied to a child of this node.
   */
  scene::SceneNode* visualNode_ = nullptr;

  /**
   * @brief all nodes created when this object's render asset was added to the
   * SceneGraph
   */
  std::vector<esp::scene::SceneNode*> visualNodes_;

 protected:
  /**
   * @brief Shift the object's local origin by translating all children of this
   * object's SceneNode.
   * @param shift The translation to apply to object's children.
   */
  virtual void shiftOrigin(const Magnum::Vector3& shift) {
    // shift visual components
    if (visualNode_)
      visualNode_->translate(shift);
    node().computeCumulativeBB();
  }

  /**
   * @brief Shift the object's local origin to be coincident with the center of
   * it's bounding box, @ref cumulativeBB_. See @ref shiftOrigin.
   */
  void shiftOriginToBBCenter() {
    shiftOrigin(-node().getCumulativeBB().center());
  }

  /** @brief Flag sepcifying whether or not the object has an active collision
   * shape.
   */
  bool isCollidable_ = false;

  /**
   * @brief Saved attributes when the object was initialized.
   */
  metadata::attributes::AbstractObjectAttributes::ptr
      initializationAttributes_ = nullptr;

 public:
  ESP_SMART_POINTERS(RigidBase)
};  // class RigidBase

}  // namespace physics
}  // namespace esp
#endif  // ESP_PHYSICS_RIGIDBASE_H_
