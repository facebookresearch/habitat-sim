// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Agent.h"

#include "esp/scene/ObjectControls.h"
#include "esp/sensor/Sensor.h"

namespace esp {
namespace agent {

namespace Mn = Magnum;

const std::set<std::string> Agent::BodyActions = {"moveRight",   "moveLeft",
                                                  "moveForward", "moveBackward",
                                                  "turnLeft",    "turnRight"};

Agent::Agent(scene::SceneNode& agentNode, const AgentConfiguration& cfg)
    : Mn::SceneGraph::AbstractFeature3D(agentNode),
      configuration_(cfg),
      controls_(scene::ObjectControls::create()) {
  agentNode.setType(scene::SceneNodeType::AGENT);
}  // Agent::Agent

Agent::~Agent() {
  ESP_DEBUG() << "Deconstructing Agent";
}

bool Agent::act(const std::string& actionName) {
  if (hasAction(actionName)) {
    const ActionSpec& actionSpec = *configuration_.actionSpace.at(actionName);
    if (BodyActions.find(actionSpec.name) != BodyActions.end()) {
      controls_->action(object(), actionSpec.name,
                        actionSpec.actuation.at("amount"),
                        /*applyFilter=*/true);
    } else {
      for (const auto& p : node().getNodeSensors()) {
        controls_->action(p.second.get().object(), actionSpec.name,
                          actionSpec.actuation.at("amount"),
                          /*applyFilter=*/false);
      }
    }
    return true;
  }
  return false;
}

bool Agent::hasAction(const std::string& actionName) const {
  auto actionSpace = configuration_.actionSpace;
  return !(actionSpace.find(actionName) == actionSpace.end());
}

void Agent::reset() {
  setState(initialState_);
}

void Agent::getState(const AgentState::ptr& state) const {
  // TODO this should be done less hackishly
  state->position = node().absoluteTransformation().translation();
  // doing this for x,y,z,w format of state's rotation.
  auto rot = node().rotation();
  state->rotation = Mn::Vector4(rot.vector(), rot.scalar());
  // TODO other state members when implemented
}

void Agent::setState(const AgentState& state,
                     const bool resetSensors /*= true*/) {
  node().setTranslation(Mn::Vector3(state.position));

  const Mn::Quaternion rot =
      Mn::Quaternion(state.rotation.xyz(), state.rotation.w());
  CORRADE_ASSERT(rot.isNormalized(),
                 state.rotation << " not a valid rotation", );
  node().setRotation(rot);

  if (resetSensors) {
    for (auto& p : node().getNodeSensors()) {
      p.second.get().setTransformationFromSpec();
    }
  }
  // TODO other state members when implemented
}

bool operator==(const ActionSpec& a, const ActionSpec& b) {
  return a.name == b.name && a.actuation == b.actuation;
}
bool operator!=(const ActionSpec& a, const ActionSpec& b) {
  return !(a == b);
}

bool operator==(const AgentConfiguration& a, const AgentConfiguration& b) {
  return a.height == b.height && a.radius == b.radius &&
         esp::equal(a.sensorSpecifications, b.sensorSpecifications) &&
         esp::equal(a.actionSpace, b.actionSpace) && a.bodyType == b.bodyType;
}
bool operator!=(const AgentConfiguration& a, const AgentConfiguration& b) {
  return !(a == b);
}

}  // namespace agent
}  // namespace esp
