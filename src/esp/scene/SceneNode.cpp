// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneNode.h"

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

SceneNode& SceneNode::createChild() {
  // will set the parent to *this
  SceneNode* node = new SceneNode(*this);
  node->setId(this->getId());
  return *node;
}

//! @brief recursivles compute the cumulative bounding box of this node's tree.
const Magnum::Range3D& SceneNode::computeCumulativeBB() {
  // first copy from your precomputed mesh bb
  cumulativeBB_ = Magnum::Range3D(meshBB_);
  auto* child = children().first();

  while (child != nullptr) {
    // try catch before cast incase a non scene node child is attached here.
    // Better way?
    try {
      SceneNode* child_node = static_cast<SceneNode*>(child);
      cumulativeBB_ =
          Magnum::Math::join(cumulativeBB_, child_node->computeCumulativeBB());
    } catch (...) {
      // TODO: need a warning here?
    }
    child = child->nextSibling();
  }
  return cumulativeBB_;
}

}  // namespace scene
}  // namespace esp
