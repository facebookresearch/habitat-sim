// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_MANAGEDPHYSICSOBJECTBASE_H_
#define ESP_PHYSICS_MANAGEDPHYSICSOBJECTBASE_H_

#include <Corrade/Utility/Macros.h>

#include "esp/core/managedContainers/AbstractManagedObject.h"
#include "esp/physics/PhysicsObjectBase.h"

namespace esp {
namespace physics {

/**
 * @brief Base class template for wrapper for physics objects of all kinds to
 * enable Managed Container access.
 */
template <class T>
class AbstractManagedPhysicsObject
    : public esp::core::managedContainers::AbstractManagedObject {
 public:
  static_assert(
      std::is_base_of<esp::physics::PhysicsObjectBase, T>::value,
      "AbstractManagedPhysicsObject :: Managed physics object type must be "
      "derived from esp::physics::PhysicsObjectBase");

  typedef std::weak_ptr<T> WeakObjRef;

  explicit AbstractManagedPhysicsObject(const std::string& classKey) {
    AbstractManagedPhysicsObject::setClassKey(classKey);
  }

  void setObjectRef(const std::shared_ptr<T>& objRef) { weakObjRef_ = objRef; }

  ~AbstractManagedPhysicsObject() override = default;

  /**
   * @brief Test whether this wrapper's object still exists.
   */
  bool isAlive() { return !weakObjRef_.expired(); }

  /**
   * @brief Get this managed object's class.  Should only be set from
   * constructor. Used as key in constructor function pointer maps in Managed
   * Container.
   */
  std::string getClassKey() const override { return classKey_; }

  std::string getHandle() const override {
    if (auto sp = getObjectReference()) {
      return sp->getObjectName();
    }
    return "";
  }

  /**
   *  @brief Managed Physics objects manage their own handles, so this is
   * currently unsettable.
   */
  void setHandle(CORRADE_UNUSED const std::string& name) override {}

  /**
   * @brief return the object's ID or nullptr if doesn't exist.
   */
  int getID() const override {
    if (auto sp = getObjectReference()) {
      return sp->getObjectID();
    }
    return ID_UNDEFINED;
  }  // getID()

  /**
   *  @brief Managed Physics objects manage their own IDs, so this is
   * unsettable.
   */
  void setID(CORRADE_UNUSED int ID) override {}

  MotionType getMotionType() const {
    if (auto sp = getObjectReference()) {
      return sp->getMotionType();
    }
    return MotionType::UNDEFINED;
  }

  void setMotionType(MotionType mt) {
    if (auto sp = getObjectReference()) {
      sp->setMotionType(mt);
    }
  }

  bool isActive() const {
    if (auto sp = this->getObjectReference()) {
      return sp->isActive();
    }
    return false;
  }  // isActive()

  void setActive(const bool active) {
    if (auto sp = this->getObjectReference()) {
      sp->setActive(active);
    }
  }  // setActive()

  void setLightSetup(const std::string& lightSetupKey) {
    if (auto sp = this->getObjectReference()) {
      sp->setLightSetup(lightSetupKey);
    }
  }  // setLightSetup

  bool contactTest() {
    if (auto sp = this->getObjectReference()) {
      return sp->contactTest();
    }
    return false;
  }  // contactTest

  void overrideCollisionGroup(CollisionGroup group) {
    if (auto sp = this->getObjectReference()) {
      return sp->overrideCollisionGroup(group);
    }
  }  // overrideCollisionGroup

  scene::SceneNode* getSceneNode() {
    if (auto sp = this->getObjectReference()) {
      return &const_cast<scene::SceneNode&>(sp->getSceneNode());
    }
    return nullptr;
  }  // getSceneNode

  core::config::Configuration::ptr getUserAttributes() const {
    if (auto sp = this->getObjectReference()) {
      return sp->getUserAttributes();
    }
    return nullptr;
  }
  // ==== Transformations ===

  Magnum::Matrix4 getTransformation() const {
    if (auto sp = this->getObjectReference()) {
      return sp->getTransformation();
    }
    return Magnum::Matrix4{};
  }  // getTransformation

  void setTransformation(const Magnum::Matrix4& transformation) {
    if (auto sp = this->getObjectReference()) {
      sp->setTransformation(transformation);
    }
  }  // setTransformation

  Magnum::Vector3 getTranslation() const {
    if (auto sp = this->getObjectReference()) {
      return sp->getTranslation();
    }
    return Magnum::Vector3{};
  }  // getTranslation

  void setTranslation(const Magnum::Vector3& vector) {
    if (auto sp = this->getObjectReference()) {
      sp->setTranslation(vector);
    }
  }  // setTranslation

  Magnum::Quaternion getRotation() const {
    if (auto sp = this->getObjectReference()) {
      return sp->getRotation();
    }
    return Magnum::Quaternion{};
  }  // getTranslation
  void setRotation(const Magnum::Quaternion& quaternion) {
    if (auto sp = this->getObjectReference()) {
      sp->setRotation(quaternion);
    }
  }  // setRotation

  core::RigidState getRigidState() {
    if (auto sp = this->getObjectReference()) {
      return sp->getRigidState();
    }
    return core::RigidState{};
  }  // getRigidState()

  void setRigidState(const core::RigidState& rigidState) {
    if (auto sp = this->getObjectReference()) {
      sp->setRigidState(rigidState);
    }
  }  // setRigidState

  void resetTransformation() {
    if (auto sp = this->getObjectReference()) {
      sp->resetTransformation();
    }
  }  // resetTransformation

  void translate(const Magnum::Vector3& vector) {
    if (auto sp = this->getObjectReference()) {
      sp->translate(vector);
    }
  }  // translate

  void translateLocal(const Magnum::Vector3& vector) {
    if (auto sp = this->getObjectReference()) {
      sp->translateLocal(vector);
    }
  }  // translateLocal

  void rotate(const Magnum::Rad angleInRad,
              const Magnum::Vector3& normalizedAxis) {
    if (auto sp = this->getObjectReference()) {
      sp->rotate(angleInRad, normalizedAxis);
    }
  }  // rotate

  void rotateLocal(const Magnum::Rad angleInRad,
                   const Magnum::Vector3& normalizedAxis) {
    if (auto sp = this->getObjectReference()) {
      sp->rotateLocal(angleInRad, normalizedAxis);
    }
  }  // rotateLocal

  void rotateX(const Magnum::Rad angleInRad) {
    if (auto sp = this->getObjectReference()) {
      sp->rotateX(angleInRad);
    }
  }  // rotateX

  void rotateY(const Magnum::Rad angleInRad) {
    if (auto sp = this->getObjectReference()) {
      sp->rotateY(angleInRad);
    }
  }  // rotateY

  void rotateZ(const Magnum::Rad angleInRad) {
    if (auto sp = this->getObjectReference()) {
      sp->rotateZ(angleInRad);
    }
  }  // rotateZ

  void rotateXLocal(const Magnum::Rad angleInRad) {
    if (auto sp = this->getObjectReference()) {
      sp->rotateXLocal(angleInRad);
    }
  }  // rotateXLocal

  void rotateYLocal(const Magnum::Rad angleInRad) {
    if (auto sp = this->getObjectReference()) {
      sp->rotateYLocal(angleInRad);
    }
  }  // rotateYLocal

  void rotateZLocal(const Magnum::Rad angleInRad) {
    if (auto sp = this->getObjectReference()) {
      sp->rotateZLocal(angleInRad);
    }
  }  // rotateZLocal

  std::vector<scene::SceneNode*> getVisualSceneNodes() const {
    if (auto sp = this->getObjectReference()) {
      return sp->getVisualSceneNodes();
    }
    return std::vector<scene::SceneNode*>();
  }  // getVisualSceneNodes

  /**
   * @brief Retrieve a comma-separated string holding the header values for the
   * info returned for this managed object.
   */
  std::string getObjectInfoHeader() const override {
    return "Type, Name, ID, Translation XYZ, Rotation W[XYZ], " +
           getPhyObjInfoHeaderInternal();
  }

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  std::string getObjectInfo() const override {
    if (auto sp = this->getObjectReference()) {
      namespace CrUt = Corrade::Utility;
      return Cr::Utility::formatString(
          "{},{},{},{},{},{},", classKey_, sp->getObjectName(),
          std::to_string(sp->getObjectID()),
          CrUt::ConfigurationValue<Mn::Vector3>::toString(sp->getTranslation(),
                                                          {}),
          CrUt::ConfigurationValue<Magnum::Quaternion>::toString(
              sp->getRotation(), {}),
          getPhysObjInfoInternal(sp));
    }
    return Cr::Utility::formatString("Unknown classkey {},", classKey_);
  }

 protected:
  /**
   * @brief Retrieve a comma-separated string holding the header values for
   * the info returned for this managed object, type-specific.
   * TODO : once Magnum supports retrieving key-values of configurations, use
   * that to build this data.
   */

  virtual std::string getPhyObjInfoHeaderInternal() const = 0;
  /**
   * @brief Specialization-specific extension of getObjectInfo, comma
   * separated info ideal for saving to csv
   */
  virtual std::string getPhysObjInfoInternal(std::shared_ptr<T>& sp) const = 0;

  /**
   * @brief This function accesses the underlying shared pointer of this
   * object's @p weakObjRef_ if it exists; if not, it provides a message.
   * @return Either a shared pointer of this wrapper's object, or nullptr if
   * dne.
   */
  std::shared_ptr<T> inline getObjectReference() const {
    std::shared_ptr<T> sp = weakObjRef_.lock();
    if (!sp) {
      // TODO: Verify object is removed from manager here?
      ESP_ERROR()
          << "This object no longer exists.  Please delete any variable "
             "references.";
    }
    return sp;
  }  // getObjectReference

  /**
   * @brief Set this managed object's class.  Should only be set from
   * constructor. Used as key in constructor function pointer maps in Managed
   * Container.
   * @param classKey the string handle corresponding to the
   * constructors used to make copies of this object in copy constructor map.
   */
  void setClassKey(const std::string& classKey) override {
    classKey_ = classKey;
  }

  /**
   * @brief Weak ref to object. If user has copy of this wrapper but object
   * has been deleted, this will be nullptr.
   */
  WeakObjRef weakObjRef_{};

  /**
   * @brief Name of instancing class responsible for this managed object
   */
  std::string classKey_;

 public:
  ESP_SMART_POINTERS(AbstractManagedPhysicsObject<T>)
};  // namespace physics

}  // namespace physics
}  // namespace esp
#endif  // ESP_PHYSICS_MANAGEDPHYSICSOBJECTBASE_H_
