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
  // nodeSensorSuite_ and subtreeSensorSuite_ will be released upon SceneNode's
  // deletion
  nodeSensorSuite_ = new esp::sensor::SensorSuite(*this);
  subtreeSensorSuite_ = new esp::sensor::SensorSuite(*this);
}

SceneNode::~SceneNode() {
  // If the entire scene graph is being deleted no need to update anything
  if (SceneGraph::isRootNode(*this)) {
    return;
  }
  // If a subtree is being deleted
  // If parent node is nullptr already, there is no need to do anything.
  // This is because magnum destroys the nodes in a top-down manner
  // recursively, root node first, and then its child nodes
  if (!this->parent()) {
    return;
  }

  // If parent node is not nullptr, update the sensorSuites stored in each
  // ancestor node in a bottom-up manner.

  // First remove Sensor from parent's nodeSensorSuite if node is leaf node
  removeSensorFromParentNodeSensorSuite();

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
  // Perform same internal checks as magnum to ensure newParent is a valid new
  // parent before updating any SensorSuites
  // Parent can not be leaf node
  CORRADE_ASSERT(!(newParent->getSceneNodeTags() & SceneNodeTag::Leaf),
                 "SceneNode::setParent(): New parent node can not be leaf node",
                 *this);

  // Skip if this is scene or RootNode (which cannot have parent), or if
  // newParent is already parent
  if (isScene() || SceneGraph::isRootNode(*this) ||
      this->parent() == newParent) {
    return *this;
  }

  // Skip if this is an ancestor of newParent
  auto p = newParent->parent();
  while (p) {
    if (p == this)
      return *this;
    p = p->parent();
  }

  // Remove sensors from old parent node's nodeSensorSuite
  removeSensorFromParentNodeSensorSuite();

  // Update old ancestors' SubtreeSensorSuites
  removeSubtreeSensorsFromAncestors();

  MagnumObject::setParent(newParent);

  // Update new ancestors'SubtreeSensorSuites
  addSubtreeSensorsToAncestors();
  // Add sensors to newParent's nodeSensorSuite
  addSensorToParentNodeSensorSuite();
  return *this;
}

void SceneNode::addSensorToParentNodeSensorSuite() {
  // Only add a sensor if this is a leaf node and sensor exists
  if (!(getSceneNodeTags() & SceneNodeTag::Leaf) ||
      nodeSensorSuite_->getSensors().empty()) {
    return;
  }
  // There only exists 0 or 1 sensors in a leaf nodeSensorSuite
  CORRADE_INTERNAL_ASSERT(nodeSensorSuite_->getSensors().size() == 1);
  // Get the first one, and if it is valid, add it to parent's nodeSensorSuite
  std::map<std::string, std::reference_wrapper<sensor::Sensor>>::iterator it =
      nodeSensorSuite_->getSensors().begin();
  if (it != nodeSensorSuite_->getSensors().end()) {
    SceneNode* parentNode = dynamic_cast<SceneNode*>(this->parent());
    // SceneNode::addSensorToParentNodeSensorSuite() is only called when
    // constructing a sensor or setting the new parent of a node, so parentNode
    // should never be nullptr
    CORRADE_ASSERT(parentNode != nullptr,
                   "SceneNode::addSensorToParentNodeSensorSuite(): Parent node "
                   "is nullptr", );
    parentNode->getNodeSensorSuite().add(it->second);
  }
}

void SceneNode::removeSensorFromParentNodeSensorSuite() {
  // Only remove a sensor if this is a leaf node and sensor exists
  if (!(getSceneNodeTags() & SceneNodeTag::Leaf) ||
      nodeSensorSuite_->getSensors().empty()) {
    return;
  }
  // There only exists 0 or 1 sensors in a leaf nodeSensorSuite
  CORRADE_INTERNAL_ASSERT(nodeSensorSuite_->getSensors().size() == 1);
  // Get the first one, and if it is valid, remove it from parent's
  // nodeSensorSuite
  std::map<std::string, std::reference_wrapper<sensor::Sensor>>::iterator it =
      nodeSensorSuite_->getSensors().begin();
  if (it != nodeSensorSuite_->getSensors().end()) {
    SceneNode* parentNode = dynamic_cast<SceneNode*>(this->parent());
    // If parentNode has not been deconstructed, remove leaf node's sensor from
    // parentNode's nodeSensorSuite
    if (parentNode != nullptr) {
      parentNode->getNodeSensorSuite().remove(it->first);
    }
  }
}

void SceneNode::addSubtreeSensorsToAncestors() {
  for (const auto& sensor : subtreeSensorSuite_->getSensors()) {
    SceneNode* currentNode = dynamic_cast<SceneNode*>(this->parent());
    while (currentNode && !SceneGraph::isRootNode(*currentNode)) {
      currentNode->getSubtreeSensorSuite().add(sensor.second);
      currentNode = dynamic_cast<SceneNode*>(currentNode->parent());
    }
  }
}

void SceneNode::removeSubtreeSensorsFromAncestors() {
  for (const auto& sensor : subtreeSensorSuite_->getSensors()) {
    SceneNode* currentNode = dynamic_cast<SceneNode*>(this->parent());
    while (currentNode && !SceneGraph::isRootNode(*currentNode)) {
      currentNode->getSubtreeSensorSuite().remove(sensor.first);
      currentNode = dynamic_cast<SceneNode*>(currentNode->parent());
    }
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
