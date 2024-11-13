// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_ARTICULATEDLINK_H_
#define ESP_PHYSICS_ARTICULATEDLINK_H_

/** @file
 * @brief Class @ref esp::physics::ArticulatedLink
 */

#include "RigidBase.h"
#include "esp/scene/SceneNode.h"

namespace esp {
namespace physics {

////////////////////////////////////
// Link
////////////////////////////////////

/**
 * @brief A single rigid link in a kinematic chain. Abstract class. Feature
 * attaches to a SceneNode.
 */
class ArticulatedLink : public RigidBase {
 public:
  ArticulatedLink(scene::SceneNode* bodyNode,
                  int index,
                  const assets::ResourceManager& resMgr)
      : RigidBase(bodyNode,
                  0,  // TODO: pass an actual object ID. This is currently
                      // assigned AFTER creation.
                  resMgr),
        mbIndex_(index) {
    setIsArticulated(true);
  }

  ~ArticulatedLink() override = default;

  //! Get the link's index within its multibody
  int getIndex() const { return mbIndex_; }

  //! List of visual components attached to this link. Used for NavMesh
  //! recomputation. Each entry is a child node of this link's node and a string
  //! key to reference the asset in ResourceManager.
  std::vector<std::pair<esp::scene::SceneNode*, std::string>>
      visualAttachments_;

  // RigidBase overrides

  /**
   * @brief Initializes the link.
   * @param resMgr a reference to ResourceManager object
   * @param handle The handle for the template structure defining relevant
   * physical parameters for this object
   * @return true if initialized successfully, false otherwise.
   */
  bool initialize(
      CORRADE_UNUSED metadata::attributes::AbstractObjectAttributes::ptr
          initAttributes) override {
    return true;
  }

  void initializeArticulatedLink(const std::string& _linkName,
                                 const Mn::Vector3& _scale) {
    linkName = _linkName;
    setScale(_scale);
  }

  /**
   * @brief Finalize the creation of the link.
   * @return whether successful finalization.
   */
  bool finalizeObject() override { return true; }

  void setTransformation(
      CORRADE_UNUSED const Magnum::Matrix4& transformation) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void setTranslation(CORRADE_UNUSED const Magnum::Vector3& vector) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void setRotation(
      CORRADE_UNUSED const Magnum::Quaternion& quaternion) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void setRigidState(
      CORRADE_UNUSED const core::RigidState& rigidState) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void resetTransformation() override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void translate(CORRADE_UNUSED const Magnum::Vector3& vector) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void translateLocal(CORRADE_UNUSED const Magnum::Vector3& vector) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void rotate(CORRADE_UNUSED const Magnum::Rad angleInRad,
              CORRADE_UNUSED const Magnum::Vector3& normalizedAxis) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void rotateLocal(
      CORRADE_UNUSED const Magnum::Rad angleInRad,
      CORRADE_UNUSED const Magnum::Vector3& normalizedAxis) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void rotateX(CORRADE_UNUSED const Magnum::Rad angleInRad) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void rotateY(CORRADE_UNUSED const Magnum::Rad angleInRad) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void rotateZ(CORRADE_UNUSED const Magnum::Rad angleInRad) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void rotateXLocal(CORRADE_UNUSED const Magnum::Rad angleInRad) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void rotateYLocal(CORRADE_UNUSED const Magnum::Rad angleInRad) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void rotateZLocal(CORRADE_UNUSED const Magnum::Rad angleInRad) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  /**
   * @brief Not used for articulated links.  Set or reset the object's state
   * using the object's specified @p sceneInstanceAttributes_.
   */
  void resetStateFromSceneInstanceAttr() override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  std::string linkName = "";
  std::string linkJointName = "";

  /**
   * @brief Set the ManagedArticulatedObject encapsulating this link's owning
   * Articulated Object
   */
  template <class T>
  void setOwningManagedAO(const std::shared_ptr<T>& owningManagedAO) {
    _owningManagedAO = owningManagedAO;
  }

 protected:
  /**
   * @brief Get the ManagedArticulatedObject encapsulating this link's owning
   * Articulated Object
   */
  template <class T>
  std::shared_ptr<T> getOwningManagedAOInternal() const {
    if (!_owningManagedAO) {
      return nullptr;
    }
    static_assert(
        std::is_base_of<core::managedContainers::AbstractManagedObject,
                        T>::value,
        "ManagedArticulatedObject must be base class of desired "
        "ArticulatedLink's Owning AO's Managed wrapper class.");
    return std::static_pointer_cast<T>(_owningManagedAO);
  }

 private:
  /**
   * @brief Finalize the initialization of this link.
   * @return true if initialized successfully, false otherwise.
   */
  bool initialization_LibSpecific() override { return true; }
  /**
   * @brief any physics-lib-specific finalization code that needs to be run
   * after creation.
   * @return whether successful finalization.
   */
  bool finalizeObject_LibSpecific() override { return true; }

  // end RigidBase overrides

  /**
   * @brief The managed wrapper object for the AO that owns this link.
   */
  core::managedContainers::AbstractManagedObject::ptr _owningManagedAO =
      nullptr;

 protected:
  int mbIndex_;

 public:
  ESP_SMART_POINTERS(ArticulatedLink)
};

}  // namespace physics
}  // namespace esp
#endif  // ESP_PHYSICS_ARTICULATEDLINK_H_
