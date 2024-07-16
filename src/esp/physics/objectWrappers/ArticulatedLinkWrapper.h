// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_ARTICULATEDLINKWRAPPER_H_
#define ESP_PHYSICS_ARTICULATEDLINKWRAPPER_H_

#include "ManagedArticulatedObject.h"

namespace esp {
namespace physics {

/**
 * @brief Class providing an access abstraction API for ArticulatedLinks which
 * are not directly managed by Configurations and therefore should not be
 * exposed through the ManageObject API. Provides bindings for all
 * ArticulatedLink-specific functionality through the ManagedArticulatedObject
 * API.
 */
template <class T>
class ArticulatedLinkWrapper {
 public:
  static_assert(
      std::is_base_of<esp::physics::ArticulatedObject, T>::value,
      "ArticulatedObject :: ArticulatedLinkWrapper object parent type must be "
      "derived from esp::physics::ArticulatedObject");

  typedef std::weak_ptr<T> WeakParentRef;

  ArticulatedLinkWrapper(std::shared_ptr<T> parentAO, int linkId) {
    _linkIndex = linkId;
    weakParentRef_ = parentAO;
  }

  //! Get the link's index within its multibody
  int getLinkIndex() const { return _linkIndex; }

  //! Get the link's Simulator objectId
  int getObjectId() const {
    if (auto sp = getParentReference()) {
      return sp->getLinkIdsToObjectIds()[_linkIndex];
    }
    return ID_UNDEFINED;
  }

  /**
   * @brief Get the SceneNode for this ArticulatedLink.
   * @return pointer to the SceneNode.
   */
  scene::SceneNode* getSceneNode() const {
    if (auto sp = getParentReference()) {
      return &const_cast<scene::SceneNode&>(sp->getLinkSceneNode(_linkIndex));
    }
    return nullptr;
  }

  /**
   * @brief Get a list of the visual scene nodes (those with render assets
   * attached) from this ArticulatedLink.
   * @return pointer to the visual SceneNodes.
   */
  std::vector<scene::SceneNode*> getVisualSceneNodes() const {
    if (auto sp = getParentReference()) {
      return sp->getLinkVisualSceneNodes(_linkIndex);
    }
    return {};
  }

  /**
   * @brief Get a list of the visual scene nodes (those with render assets
   * attached) from this ArticulatedLink.
   * @return pointer to the visual SceneNodes.
   */
  int getParentArticulatedObjectId() {
    if (auto sp = getParentReference()) {
      return sp->getObjectID();
    }
    return {};
  }

  // AABB
  // transform
  // translation
  // rotation
  // is_alive

  // TODO: how to do this without a wrapper/manager
  //  /**
  //   * @brief Get a list of the visual scene nodes (those with render assets
  //   attached) from this ArticulatedLink.
  //   * @return pointer to the visual SceneNodes.
  //   */
  //  String getParentArticulatedObjectHandle() {
  //    if (auto sp = getParentReference()) {
  //      return sp->getLinkVisualSceneNodes(_linkIndex);
  //    }
  //    return {};
  //  }

  /* TODO: how to get a ManagedObject from inside a wrapper?
  esp::physics::ManagedArticulatedObject::ptr getParentArticulatedObject() {
    //TODO: how?
    if (auto sp = getParentReference()) {
      return sp->;
    }
    return nullptr;
   }
  */

 protected:
  /**
   * @brief This function accesses the underlying shared pointer of this
   * object's @p weakParentRef_ if it exists; if not, it provides a message.
   * @return Either a shared pointer of this wrapper's parent object, or nullptr
   * if dne.
   */
  std::shared_ptr<T> inline getParentReference() const {
    std::shared_ptr<T> sp = weakParentRef_.lock();
    if (!sp) {
      ESP_ERROR() << "This link's parent object no longer exists.  Please "
                     "delete any variable "
                     "references.";
    }
    return sp;
  }  // getParentReference

  int _linkIndex;

  /**
   * @brief Weak ref to parent object. If user has copy of this wrapper but
   * object has been deleted, this will be nullptr.
   */
  WeakObjRef weakParentRef_{};

 public:
  ESP_SMART_POINTERS(ArticulatedLinkWrapper)
};  // class ArticulatedLinkWrapper

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_ARTICULATEDLINKWRAPPER_H_
