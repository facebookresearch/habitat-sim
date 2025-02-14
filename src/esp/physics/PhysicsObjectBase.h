// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_PHYSICSOBJECTBASE_H_
#define ESP_PHYSICS_PHYSICSOBJECTBASE_H_

#include <Corrade/Containers/Optional.h>
#include <Corrade/Containers/Reference.h>
#include "esp/core/RigidState.h"
#include "esp/gfx/ShaderManager.h"
#include "esp/metadata/attributes/MarkerSets.h"
#include "esp/metadata/attributes/SceneInstanceAttributes.h"
#include "esp/physics/CollisionGroupHelper.h"

/** @file
 * @brief Class @ref physics::PhysicsObjectBase is the base class for any
 * physics-based construct, and holds basic accounting info and accessors, along
 * with scene node access.
 */

namespace esp {
namespace assets {
class ResourceManager;
}

namespace physics {

/**
 * @brief Motion type of a @ref esp::physics::RigidObject.
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
   * esp::physics::RigidStage.
   */
  STATIC,

  /**
   * The object is expected to move kinematically, but is not simulated. Default
   * behavior of @ref esp::physics::RigidObject with no physics simulator defined.
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
        resMgr_(resMgr),
        userAttributes_(std::make_shared<core::config::Configuration>()) {
    bodyNode->setBaseObjectId(objectId);
  }

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

  /** @brief Get a copy of the template used to initialize this object
   * or scene.
   * @return A copy of the initialization template used to create this object
   * instance or nullptr if no template exists.
   */
  template <class T>
  std::shared_ptr<T> getInitializationAttributes() const {
    if (!objInitAttributes_) {
      return nullptr;
    }
    return T::create(*(static_cast<const T*>(objInitAttributes_.get())));
  }

  /**
   * @brief Get the @ref MotionType of the object. See @ref
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
   * @param mt The desired @ref MotionType.
   */
  virtual void setMotionType(MotionType mt) = 0;

  /**
   * @brief Get object's ID
   */
  int getObjectID() const { return objectId_; }

  /**
   * @brief Object name, to facilitate access.
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
  virtual bool contactTest() {
    ESP_ERROR()
        << "Not implemented. Install with --bullet to use this feature.";
    return false;
  }

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
   * @brief Given the list of passed points in this object's local space, return
   * those points transformed to world space.
   * @param points vector of points in object local space
   * @param linkID Unused for rigids.
   * @return vector of points transformed into world space
   */
  virtual std::vector<Mn::Vector3> transformLocalPointsToWorld(
      const std::vector<Mn::Vector3>& points,
      CORRADE_UNUSED int linkID = ID_UNDEFINED) const {
    std::vector<Mn::Vector3> wsPoints;
    wsPoints.reserve(points.size());
    Mn::Vector3 objScale = getScale();
    Mn::Matrix4 worldTransform = node().absoluteTransformation();
    for (const auto& lsPoint : points) {
      wsPoints.emplace_back(worldTransform.transformPoint(lsPoint * objScale));
    }
    return wsPoints;
  }

  /**
   * @brief Given the list of passed points in world space, return
   * those points transformed to this object's local space.
   * @param points vector of points in world space
   * @param linkID Unused for rigids.
   * @return vector of points transformed to be in local space
   */
  virtual std::vector<Mn::Vector3> transformWorldPointsToLocal(
      const std::vector<Mn::Vector3>& points,
      CORRADE_UNUSED int linkID = ID_UNDEFINED) const {
    std::vector<Mn::Vector3> lsPoints;
    lsPoints.reserve(points.size());
    Mn::Vector3 objScale = getScale();
    Mn::Matrix4 worldTransform = node().absoluteTransformation();
    for (const auto& wsPoint : points) {
      lsPoints.emplace_back(worldTransform.inverted().transformPoint(wsPoint) /
                            objScale);
    }
    return lsPoints;
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
   * @param vector The desired 3D vector in the object's ocal coordinate system
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

  /** @brief Activate deferred updates, preventing SceneNode state changes until
   * updateNodes is called to prevent SceneGraph pollution during render.
   */
  virtual void deferUpdate() { isDeferringUpdate_ = true; }

  /** @brief Disable deferred updates if active and sets SceneNode states from
   * internal object physics states.
   * @param force If set, update sleeping nodes as well as active nodes.
   */
  virtual void updateNodes(CORRADE_UNUSED bool force = false) {
    isDeferringUpdate_ = false;
  }

  /**
   * @brief Set or reset the object's state using the object's specified @p
   * sceneInstanceAttributes_.
   */
  virtual void resetStateFromSceneInstanceAttr() = 0;

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
    _objInstanceInitAttributes = std::move(instanceAttr);
  }  // setSceneInstanceAttr

  /**
   * @brief Get pointers to this physics object's visual scene nodes
   * @return vector of pointers to the object's visual scene nodes.
   */
  virtual std::vector<scene::SceneNode*> getVisualSceneNodes() const = 0;

  core::config::Configuration::ptr getUserAttributes() const {
    return userAttributes_;
  }

  /**
   * @brief This function will overwrite this object's existing user-defined
   * attributes with @p attr.
   * @param attr A ptr to the user defined attributes specified for this object.
   */
  void setUserAttributes(core::config::Configuration::ptr attr) {
    userAttributes_ = std::move(attr);
  }

  /**
   * @brief This function will merge this object's existing user-defined
   * attributes with @p attr by overwriting it with @p attr.
   * @param attr A ptr to the user defined attributes that are to be merged into
   * this object's existing user-defined attributes.
   */
  void mergeUserAttributes(const core::config::Configuration::ptr& attr) {
    userAttributes_->overwriteWithConfig(attr);
  }

  /**
   * @brief Get a reference to the existing MarkerSets for this object.
   */
  metadata::attributes::MarkerSets::ptr getMarkerSets() const {
    return markerSets_;
  }

  /**
   * @brief This function will overwrite this object's existing MarkerSets
   * attributes with @p attr.
   * @param attr A ptr to the MarkerSets attributes specified for this object.
   */
  void setMarkerSets(metadata::attributes::MarkerSets::ptr attr) {
    markerSets_ = std::move(attr);
  }

  /**
   * @brief This function will merge this object's existing MarkerSets
   * attributes with @p attr by overwriting it with @p attr.
   * @param attr A ptr to the user defined attributes specified for this object.
   * with mergee into them.
   */
  void mergeMarkerSets(const metadata::attributes::MarkerSets::ptr& attr) {
    markerSets_->overwriteWithConfig(attr);
  }

  /**
   * @brief Retrieves the hierarchical map-of-map-of-maps containing
   * the @ref MarkerSets constituent marker points, in local space
   * (which is the space they are given in).
   */
  std::unordered_map<
      std::string,
      std::unordered_map<
          std::string,
          std::unordered_map<std::string, std::vector<Mn::Vector3>>>>
  getMarkerPointsLocal() const {
    return markerSets_->getAllMarkerPoints();
  }

  /**
   * @brief Retrieves the hierarchical map-of-map-of-maps containing
   * the @ref MarkerSets constituent marker points, in local space
   * (which is the space they are given in).
   */
  virtual std::unordered_map<
      std::string,
      std::unordered_map<
          std::string,
          std::unordered_map<std::string, std::vector<Mn::Vector3>>>>
  getMarkerPointsGlobal() const {
    const auto lclPoints = markerSets_->getAllMarkerPoints();
    std::unordered_map<
        std::string,
        std::unordered_map<
            std::string,
            std::unordered_map<std::string, std::vector<Mn::Vector3>>>>
        res{};
    // for each task
    for (const auto& taskEntry : lclPoints) {
      const std::string taskName = taskEntry.first;
      std::unordered_map<
          std::string,
          std::unordered_map<std::string, std::vector<Mn::Vector3>>>
          perTaskMap;
      // for each link - should only have 1 link in rigids
      for (const auto& linkEntry : taskEntry.second) {
        const std::string linkName = linkEntry.first;
        std::unordered_map<std::string, std::vector<Mn::Vector3>> perLinkMap;
        // for each set in link
        for (const auto& markersEntry : linkEntry.second) {
          const std::string markersName = markersEntry.first;
          perLinkMap[markersName] =
              transformLocalPointsToWorld(markersEntry.second, ID_UNDEFINED);
        }
        perTaskMap[linkName] = perLinkMap;
      }
      res[taskName] = perTaskMap;
    }
    return res;
  }  // getMarkerPointsGlobal

  /**
   * @brief Get the scale of the object set during initialization.
   * @return The scaling for the object relative to its initially loaded meshes.
   */
  virtual Magnum::Vector3 getScale() const { return _creationScale; }

  /**
   * @brief Return whether or not this object is articulated. Override in
   * ArticulatedObject
   */
  bool isArticulated() const { return _isArticulated; }

  /** @brief Return the local axis-aligned bounding box of the this object.*/
  virtual const Mn::Range3D& getAabb() { return node().getCumulativeBB(); }

  /** @brief Set the managed object used to reference this object externally
   * (i.e. via python)*/
  template <class T>
  void setManagedObjectPtr(std::shared_ptr<T> managedObjPtr) {
    _managedObject = std::move(managedObjPtr);
  }

  /** @brief Return this object's mesh volume. */
  virtual double getVolume() const { return node().getMeshVolume(); }

  /** @brief Return this object's mesh surface area. */
  virtual double getSurfaceArea() const { return node().getMeshSurfaceArea(); }

 protected:
  /**
   * @brief Accessed Internally. Get the Managed Object that references this
   * object.
   */
  template <class T>
  std::shared_ptr<T> getManagedObjectPtrInternal() const {
    if (!_managedObject) {
      return nullptr;
    }
    static_assert(
        std::is_base_of<core::managedContainers::AbstractManagedObject,
                        T>::value,
        "AbstractManagedObject must be base class of desired Managed Object "
        "class.");

    return std::static_pointer_cast<T>(_managedObject);
  }

  /**
   * @brief Used Internally on object creation. Set whether or not this object
   * is articulated.
   */
  void setIsArticulated(bool isArticulated) { _isArticulated = isArticulated; }

  /**
   * @brief Accessed internally. Get an appropriately cast copy of the @ref
   * metadata::attributes::SceneObjectInstanceAttributes used to place the
   * object within the scene.
   * @return A copy of the initialization template used to create this object
   * instance or nullptr if no template exists.
   */
  template <class T>
  std::shared_ptr<T> getInitObjectInstanceAttrCopyInternal() const {
    if (!_objInstanceInitAttributes) {
      return nullptr;
    }
    static_assert(
        std::is_base_of<metadata::attributes::SceneObjectInstanceAttributes,
                        T>::value,
        "SceneObjectInstanceAttributes must be base class of desired instance "
        "attributes class.");
    return T::create(
        *(static_cast<const T*>(_objInstanceInitAttributes.get())));
  }

  /**
   * @brief Accessed internally. Get the
   * @ref metadata::attributes::SceneObjectInstanceAttributes used to
   * create and place the object within the scene, appropriately cast for
   * object type.
   * @return The initialization template used to create this object
   * instance or nullptr if no template exists.
   */
  template <class T>
  std::shared_ptr<const T> getInitObjectInstanceAttrInternal() const {
    if (!_objInstanceInitAttributes) {
      return nullptr;
    }
    static_assert(
        std::is_base_of<metadata::attributes::SceneObjectInstanceAttributes,
                        T>::value,
        "SceneObjectInstanceAttributes must be base class of desired instance "
        "attributes class.");
    return std::static_pointer_cast<const T>(_objInstanceInitAttributes);
  }

  /**
   * @brief Reverses the COM correction transformation for objects that require
   * it. Currently a simple passthrough for stages and articulated objects.
   */
  virtual Magnum::Vector3 getUncorrectedTranslation() const {
    return getTranslation();
  }

  /** @brief Accessed internally. Get an appropriately cast copy of the @ref
   * metadata::attributes::SceneObjectInstanceAttributes used to place the
   * object within the scene, updated to have the current transformation and
   * status of the object.
   * @return A copy of the initialization template used to create this object
   * instance or nullptr if no template exists.
   */
  template <class T>
  std::shared_ptr<T> getCurrentObjectInstanceAttrInternal() {
    if (!_objInstanceInitAttributes) {
      return nullptr;
    }
    static_assert(
        std::is_base_of<metadata::attributes::SceneObjectInstanceAttributes,
                        T>::value,
        "PhysicsObjectBase : Cast of SceneObjectInstanceAttributes must be to "
        "class that inherits from SceneObjectInstanceAttributes");

    std::shared_ptr<T> initObjInstAttrsCopy = std::const_pointer_cast<T>(
        T::create(*(static_cast<const T*>(_objInstanceInitAttributes.get()))));
    // set values
    const auto translation = getUncorrectedTranslation();
    if (initObjInstAttrsCopy->getTranslation() != translation) {
      initObjInstAttrsCopy->setTranslation(translation);
    }
    const auto rotation = getRotation();
    if (initObjInstAttrsCopy->getRotation() != rotation) {
      initObjInstAttrsCopy->setRotation(rotation);
    }
    // only change if different
    if (initObjInstAttrsCopy->getMotionType() != objectMotionType_) {
      initObjInstAttrsCopy->setMotionType(
          metadata::attributes::getMotionTypeName(objectMotionType_));
    }

    // temp copy of object's user attributes. Treated as ground truth for user
    // attributes.
    core::config::Configuration::ptr tmpUserAttrs =
        core::config::Configuration::create(*userAttributes_);

    // now filter this by the creation attributes' copy.  NOTE if the creation
    // attributes themselves are different than the same-named versions on disk,
    // these values may be out of sync.
    tmpUserAttrs->filterFromConfig(
        objInitAttributes_->getUserConfigurationView());
    // copy these over the existing user defined fields
    // in the instance
    initObjInstAttrsCopy->setUserConfiguration(tmpUserAttrs);
    return initObjInstAttrsCopy;
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
   * pointer to a @ref esp::core::config::Configuration. These attributes are
   * not internally processed by habitat, but provide a "scratch pad" for the
   * user to access and save important information and metadata.
   */
  core::config::Configuration::ptr userAttributes_ = nullptr;

  /**
   * @brief Stores a reference to the markersets for this object, held as a
   * smart pointer to a MarkerSets construct, which is an alias for
   * a @ref esp::core::config::Configuration.
   */
  metadata::attributes::MarkerSets::ptr markerSets_ = nullptr;

  /**
   * @brief Saved template attributes when the object was initialized.
   */
  metadata::attributes::AbstractAttributes::cptr objInitAttributes_ = nullptr;

  /**
   * @brief Set the object's creation scale
   */
  void setScale(const Magnum::Vector3& creationScale) {
    _creationScale = creationScale;
  }

 private:
  /**
   * @brief The managed wrapper-based object referencing this object.
   */
  core::managedContainers::AbstractManagedObject::ptr _managedObject = nullptr;

  /**
   * @brief This object's instancing attributes, if any were used during its
   * creation.
   */
  std::shared_ptr<const metadata::attributes::SceneObjectInstanceAttributes>
      _objInstanceInitAttributes = nullptr;

  /**
   * @brief The scale applied to this object on creation
   */
  Mn::Vector3 _creationScale{1.0f, 1.0f, 1.0f};

  /**
   * @brief Whether or not this is an articulated object. Set to true in
   * articulated object constructors.
   */
  bool _isArticulated = false;

 public:
  ESP_SMART_POINTERS(PhysicsObjectBase)
};  // class PhysicsObjectBase

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_PHYSICSOBJECTBASE_H_
