// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "AttachedObject.h"
#include "SceneNode.h"

namespace esp {
namespace scene {

AttachedObject::AttachedObject(AttachedObjectType type) : objectType_(type) {}
AttachedObject::AttachedObject(SceneNode& node, AttachedObjectType type)
    : objectType_(type) {
  // since a node is passed, attach it
  attach(node);
}

// detach the object from the scene graph
void AttachedObject::detach() {
  // if the attaching scene node is destroyed earlier, it will inform the
  // attachedObject before it was deconstructed. This must be guaranteed in the
  // destructor of "SceneNode";

  // the above statement also implies that as long as node_ is NOT nullptr,
  // it points to an address that is guaranteed accessible.

  // so if the attached object is destroyed first, it is *safe* to call
  // node_->attachedObject_

  if (node_ != nullptr) {
    node_->attachedObject_ =
        nullptr;      // break the connection (SceneNode --> AttachedObject)
    node_ = nullptr;  // break the connection (SceneNode <-- AttachedObject)
  }
}

void AttachedObject::attach(SceneNode& node) {
  node_ = &node;  // build the connection (SceneNode <-- AttachedObject)
  node_->attachedObject_ =
      this;  // build the connection (SceneNode --> AttachedObject)
}

SceneNode& AttachedObject::object() {
  ASSERT(isValid());
  return *node_;
}

const SceneNode& AttachedObject::object() const {
  ASSERT(isValid());
  return *node_;
}

}  // namespace scene
}  // namespace esp
