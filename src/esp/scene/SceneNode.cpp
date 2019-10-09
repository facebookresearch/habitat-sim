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

//! @brief recursively compute the cumulative bounding box of this node's tree.
const Magnum::Range3D& SceneNode::computeCumulativeBB() {
  // first copy from your precomputed mesh bb
  cumulativeBB_ = Magnum::Range3D(meshBB_);
  auto* child = children().first();

  while (child != nullptr) {
    SceneNode* child_node = dynamic_cast<SceneNode*>(child);
    if (child_node != nullptr) {
      child_node->computeCumulativeBB();
      std::vector<Magnum::Vector3> corners;
      corners.push_back(child_node->transformation().transformPoint(
          child_node->cumulativeBB_.frontBottomLeft()));
      corners.push_back(child_node->transformation().transformPoint(
          child_node->cumulativeBB_.frontBottomRight()));
      corners.push_back(child_node->transformation().transformPoint(
          child_node->cumulativeBB_.frontTopLeft()));
      corners.push_back(child_node->transformation().transformPoint(
          child_node->cumulativeBB_.frontTopRight()));

      corners.push_back(child_node->transformation().transformPoint(
          child_node->cumulativeBB_.backTopLeft()));
      corners.push_back(child_node->transformation().transformPoint(
          child_node->cumulativeBB_.backTopRight()));
      corners.push_back(child_node->transformation().transformPoint(
          child_node->cumulativeBB_.backBottomLeft()));
      corners.push_back(child_node->transformation().transformPoint(
          child_node->cumulativeBB_.backBottomRight()));

      Magnum::Range3D transformedBB{
          Magnum::Math::minmax<Magnum::Vector3>(corners)};

      cumulativeBB_ = Magnum::Math::join(cumulativeBB_, transformedBB);
    }
    child = child->nextSibling();
  }
  return cumulativeBB_;
}

}  // namespace scene
}  // namespace esp
