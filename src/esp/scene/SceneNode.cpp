// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneNode.h"
#include "AttachedObject.h"

using namespace Magnum;

namespace esp {
namespace scene {

SceneNode::SceneNode(SceneNode& parent) {
  setParent(&parent);
  setId(parent.getId());
}

SceneNode::SceneNode(MagnumScene& parentNode) {
  setParent(&parentNode);
}

SceneNode::~SceneNode() {
  if (attachedObject_ != nullptr) {
    attachedObject_->detach();
  }
}

SceneNode& SceneNode::createChild() {
  // will set the parent to *this
  SceneNode* node = new SceneNode(*this);
  node->setId(this->getId());
  return *node;
}

}  // namespace scene
}  // namespace esp
