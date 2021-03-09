// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneNode.h"
#include "SceneGraph.h"
#include "esp/geo/geo.h"
#include "esp/sensor/Sensor.h"

namespace Mn = Magnum;

namespace esp {
namespace scene {

SceneNode::SceneNode(SceneNode& parent)
    : Mn::SceneGraph::AbstractFeature3D{*this} {
  MagnumObject::setParent(&parent);
  setId(parent.getId());
  setCachedTransformations(Mn::SceneGraph::CachedTransformation::Absolute);
  absoluteTransformation_ = absoluteTransformation();
  nodeSensorSuite_ = new esp::sensor::SensorSuite(*this);
  subtreeSensorSuite_ = new esp::sensor::SensorSuite(*this);
}

SceneNode::SceneNode(MagnumScene& parentNode)
    : Mn::SceneGraph::AbstractFeature3D{*this} {
  MagnumObject::setParent(&parentNode);
  setCachedTransformations(Mn::SceneGraph::CachedTransformation::Absolute);
  absoluteTransformation_ = absoluteTransformation();
  nodeSensorSuite_ = new esp::sensor::SensorSuite(*this);
  subtreeSensorSuite_ = new esp::sensor::SensorSuite(*this);
}

SceneNode::~SceneNode() {
  // if the entire scene graph is being deleted, no need to update anything
  if (SceneGraph::isRootNode(*this)) {
    return;
  }
  // if it is to delete a subtree
  // then two cases:
  // 1) current node is the root node (of this subtree).
  // In case 1, you do not have to do anything since its parent node is nullptr
  // already!! This is because magnum destroys the nodes in a top-down manner
  // recursively, root node first, and then its child nodes
  if (!this->parent()) {
    return;
  }

  // 2) current node is NOT the root node.
  // In case 2, you need update the sensorSuites stored in each ancestor node in
  // a bottom-up manner.

  // First remove all sensors in nodeSensorSuite from its parent's
  // nodeSensorSuite if node is leaf node
  if (getSceneNodeTags() & SceneNodeTag::Leaf) {
    removeSensorsFromParentNodeSensorSuite();
  }

  // go bottom to the top, and erase sensors and all subtreeSensors from its
  // ancestors' subtreeSensorSuites
  removeSubtreeSensorsFromAncestors();
}

std::map<std::string, std::reference_wrapper<esp::sensor::Sensor>>&
SceneNode::getNodeSensors() {
  return nodeSensorSuite_->getSensors();
}

std::map<std::string, std::reference_wrapper<esp::sensor::Sensor>>&
SceneNode::getSubtreeSensors() {
  return subtreeSensorSuite_->getSensors();
}

SceneNode& SceneNode::createChild(SceneNodeTags childNodeTags) {
  CORRADE_ASSERT(
      !(sceneNodeTags_ & SceneNodeTag::Leaf),
      "SceneNode::createChild(): Can not create child from leaf node", *this);
  // will set the parent to *this
  SceneNode* node = new SceneNode(*this);
  node->setId(this->getId());
  node->setSceneNodeTags(childNodeTags);
  return *node;
}

SceneNode& SceneNode::setParent(SceneNode* newParent) {
  // Parent can not be leaf node
  CORRADE_ASSERT(!(newParent->getSceneNodeTags() &= SceneNodeTag::Leaf),
                 "SceneNode::setParent(): New parent node can not be leaf node",
                 *this);

  // Skip if this is scene or RootNode (which cannot have parent), or if
  // newParent is already parent
  if (isScene() || this->parent() == newParent) {
    return *this;
  }

  // Skip if this is parent of newParent
  auto p = newParent->parent();
  while (p) {
    if (p == this)
      return *this;
    p = p->parent();
  }

  // Update old ancestors' SensorSuites
  removeSubtreeSensorsFromAncestors();

  // If current node is a leaf node, it may contain sensors
  if (getSceneNodeTags() &= SceneNodeTag::Leaf) {
    // Remove sensors from old parent node's nodeSensorSuite
    removeSensorsFromParentNodeSensorSuite();
  }

  MagnumObject::setParent(newParent);

  // Update new ancestors' SensorSuites
  addSubtreeSensorsToAncestors();

  // Add sensors to newParent's nodeSensorSuite
  if (getSceneNodeTags() &= SceneNodeTag::Leaf) {
    for (const auto& sensor : nodeSensorSuite_->getSensors()) {
      newParent->getNodeSensorSuite().add(sensor.second);
    }
  }
  return *this;
}

void SceneNode::removeSensorsFromParentNodeSensorSuite() {
  for (const auto& sensor : nodeSensorSuite_->getSensors()) {
    static_cast<SceneNode*>(this->parent())
        ->getNodeSensorSuite()
        .getSensors()
        .erase(sensor.first);
  }
}

void SceneNode::removeSubtreeSensorsFromAncestors() {
  for (const auto& sensor : subtreeSensorSuite_->getSensors()) {
    SceneNode* currentNode = this;
    do {
      currentNode = dynamic_cast<SceneNode*>(currentNode->parent());
      // no need to worry that currentNode could be nullptr, no chance
      currentNode->getSubtreeSensorSuite().getSensors().erase(sensor.first);
    } while (!SceneGraph::isRootNode(*currentNode));
  }
}

void SceneNode::addSubtreeSensorsToAncestors() {
  for (const auto& sensor : subtreeSensorSuite_->getSensors()) {
    SceneNode* currentNode = this;
    do {
      currentNode = dynamic_cast<SceneNode*>(currentNode->parent());
      // no need to worry that currentNode could be nullptr, no chance
      currentNode->getSubtreeSensorSuite().add(sensor.second);
    } while (!SceneGraph::isRootNode(*currentNode));
  }
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
