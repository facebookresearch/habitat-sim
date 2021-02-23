// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneNode.h"
#include "esp/geo/geo.h"

namespace Mn = Magnum;

namespace esp {
namespace scene {

SceneNode::SceneNode(SceneNode& parent)
    : Mn::SceneGraph::AbstractFeature3D{*this} {
  setParent(&parent);
  setId(parent.getId());
  setCachedTransformations(Mn::SceneGraph::CachedTransformation::Absolute);
  absoluteTransformation_ = absoluteTransformation();
}

SceneNode::SceneNode(MagnumScene& parentNode)
    : Mn::SceneGraph::AbstractFeature3D{*this} {
  setParent(&parentNode);
  setCachedTransformations(Mn::SceneGraph::CachedTransformation::Absolute);
  absoluteTransformation_ = absoluteTransformation();
}

SceneNode& SceneNode::createChild() {
  // will set the parent to *this
  SceneNode* node = new SceneNode(*this);
  node->setId(this->getId());
  return *node;
}

//! @brief recursively compute the cumulative bounding box of this node's tree.
const Mn::Range3D& SceneNode::computeCumulativeBB() {
  // first copy from your precomputed mesh bb
  cumulativeBB_ = Mn::Range3D(meshBB_);
  auto* child = children().first();

  while (child != nullptr) {
    SceneNode* child_node = dynamic_cast<SceneNode*>(child);
    if (child_node != nullptr) {
      child_node->computeCumulativeBB();

      Mn::Range3D transformedBB = esp::geo::getTransformedBB(
          child_node->cumulativeBB_, child_node->transformation());

      cumulativeBB_ = Mn::Math::join(cumulativeBB_, transformedBB);
    }
    child = child->nextSibling();
  }
  return cumulativeBB_;
}

void SceneNode::clean(const Magnum::Matrix4& absoluteTransformation) {
  worldCumulativeBB_ = Cr::Containers::NullOpt;

  absoluteTransformation_ = absoluteTransformation;
}

Mn::Vector3 SceneNode::absoluteTranslation() const {
  if (isDirty())
    return absoluteTransformation().translation();
  else
    return absoluteTransformation_.translation();
}

Mn::Vector3 SceneNode::absoluteTranslation() {
  setClean();
  return absoluteTransformation_.translation();
}

const Mn::Range3D& SceneNode::getAbsoluteAABB() const {
  if (aabb_)
    return *aabb_;
  else {
    if (!worldCumulativeBB_)
      worldCumulativeBB_ = {
          geo::getTransformedBB(getCumulativeBB(), absoluteTransformation_)};
    return *worldCumulativeBB_;
  }
}

}  // namespace scene
}  // namespace esp
