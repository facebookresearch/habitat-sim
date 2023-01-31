// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneNode.h"
#include "SceneGraph.h"
#include "esp/core/Check.h"
#include "esp/geo/Geo.h"
#include "esp/sensor/Sensor.h"

namespace Mn = Magnum;

namespace esp {
namespace scene {

SceneNode::SceneNode()
    : Mn::SceneGraph::AbstractFeature3D{*this},
      nodeSensorSuite_(new esp::sensor::SensorSuite(*this)),
      subtreeSensorSuite_(new esp::sensor::SensorSuite(*this)) {
  setCachedTransformations(Mn::SceneGraph::CachedTransformation::Absolute);
  absoluteTransformation_ = absoluteTransformation();
  // Once created, nodeSensorSuite_ and subtreeSensorSuite_ are features owned
  // by the SceneNode. No need to release them in the destructor since the
  // magnum SceneGraph will handle it. (constructed as member initializers)
}

SceneNode::SceneNode(SceneNode& parent) : SceneNode() {
  MagnumObject::setParent(&parent);
  setId(parent.getId());
}
SceneNode::SceneNode(MagnumScene& parentNode) : SceneNode() {
  MagnumObject::setParent(&parentNode);
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
  ESP_CHECK(!(sceneNodeTags_ & SceneNodeTag::Leaf),
            "SceneNode::createChild(): Can not create child from leaf node");
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
  ESP_CHECK(!(newParent->getSceneNodeTags() & SceneNodeTag::Leaf),
            "SceneNode::setParent(): New parent node can not be leaf node");
  // Skip if this is scene or RootNode (which cannot have parent), or if
  // newParent is already parent
  if (isScene() || SceneGraph::isRootNode(*this) ||
      this->parent() == newParent) {
    return *this;
  }

  // Skip if this is an ancestor of newParent
  auto* p = newParent->parent();
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
  // Only add a sensor if this is a leaf node, a sensor exists to be added, and
  // this is not the root node
  if (!(getSceneNodeTags() & SceneNodeTag::Leaf) ||
      nodeSensorSuite_->getSensors().empty() || SceneGraph::isRootNode(*this)) {
    return;
  }
  // There only exists 0 or 1 sensors in a leaf nodeSensorSuite
  CORRADE_INTERNAL_ASSERT(nodeSensorSuite_->getSensors().size() == 1);
  // Get the first one, and if it is valid, add it to parent's nodeSensorSuite
  std::map<std::string, std::reference_wrapper<sensor::Sensor>>::iterator it =
      nodeSensorSuite_->getSensors().begin();
  SceneNode* parentNode = dynamic_cast<SceneNode*>(this->parent());
  // SceneNode::addSensorToParentNodeSensorSuite() is only called when
  // constructing a sensor or setting the new parent of a node, so parentNode
  // should never be nullptr
  CORRADE_ASSERT(parentNode != nullptr,
                 "SceneNode::addSensorToParentNodeSensorSuite(): Cannot cast "
                 "the parent node to a SceneNode", );
  parentNode->getNodeSensorSuite().add(it->second);
}

void SceneNode::removeSensorFromParentNodeSensorSuite() {
  // Only remove a sensor if this is a leaf node, a sensor exists to be removed,
  // and this is not the root node
  if (!(getSceneNodeTags() & SceneNodeTag::Leaf) ||
      nodeSensorSuite_->getSensors().empty() || SceneGraph::isRootNode(*this)) {
    return;
  }
  // There only exists 0 or 1 sensors in a leaf nodeSensorSuite
  CORRADE_INTERNAL_ASSERT(nodeSensorSuite_->getSensors().size() == 1);
  // Get the first one, and if it is valid, remove it from parent's
  // nodeSensorSuite
  std::map<std::string, std::reference_wrapper<sensor::Sensor>>::iterator it =
      nodeSensorSuite_->getSensors().begin();
  SceneNode* parentNode = dynamic_cast<SceneNode*>(this->parent());
  // If parentNode has not been deconstructed, remove leaf node's sensor from
  // parentNode's nodeSensorSuite
  if (parentNode != nullptr) {
    parentNode->getNodeSensorSuite().remove(it->first);
  }
}

void SceneNode::addSubtreeSensorsToAncestors() {
  // Do nothing if this is the root node, as the root node has no ancestors with
  // SensorSuites
  if (SceneGraph::isRootNode(*this)) {
    return;
  }
  for (const auto& entry : subtreeSensorSuite_->getSensors()) {
    SceneNode* currentNode = this;
    do {
      currentNode = dynamic_cast<SceneNode*>(currentNode->parent());
      if (currentNode != nullptr) {
        currentNode->getSubtreeSensorSuite().add(entry.second);
      }
    } while ((currentNode != nullptr) && !SceneGraph::isRootNode(*currentNode));
  }
}

void SceneNode::removeSubtreeSensorsFromAncestors() {
  // Do nothing if this is the root node, as the root node has no ancestors with
  // SensorSuites
  if (SceneGraph::isRootNode(*this)) {
    return;
  }
  for (const auto& entry : subtreeSensorSuite_->getSensors()) {
    SceneNode* currentNode = this;
    do {
      currentNode = dynamic_cast<SceneNode*>(currentNode->parent());
      if (currentNode != nullptr) {
        currentNode->getSubtreeSensorSuite().remove(entry.first);
      }
    } while ((currentNode != nullptr) && !SceneGraph::isRootNode(*currentNode));
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

void setSemanticIdForSubtree(SceneNode* node, int semanticId) {
  if (node->getSemanticId() == semanticId) {
    // We assume the entire subtree's semanticId matches the root's, so we can
    // early out here.
    return;
  }

  // See also RigidBase setSemanticId. That function uses a prepared container
  // of visual nodes, whereas this function traverses the subtree to touch all
  // nodes (including visual nodes). The results should be the same.
  auto cb = [&](SceneNode& node) { node.setSemanticId(semanticId); };
  preOrderTraversalWithCallback(*node, cb);
}

}  // namespace scene
}  // namespace esp
