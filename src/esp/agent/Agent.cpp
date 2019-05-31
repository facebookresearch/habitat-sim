// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Agent.h"

#include <Magnum/EigenIntegration/GeometryIntegration.h>
#include <Magnum/EigenIntegration/Integration.h>

#include "esp/scene/ObjectControls.h"
#include "esp/sensor/PinholeCamera.h"
#include "esp/sensor/Sensor.h"

using Magnum::EigenIntegration::cast;

namespace esp {
namespace agent {

const std::set<std::string> Agent::BodyActions = {
    "moveRight", "moveLeft", "moveForward", "moveBackward", "turnLeft",
    "turnRight",
    // TODO lookLeft and lookRight should not be body actions
    // turnLeft and turnRight will take their place
    "lookLeft", "lookRight"};

// Warning!!
// agent, as well as the attached sensors, are all in "invalid" status after the
// creation
// user have to attach them to the scene nodes (one node for each agent, each
// sensor) to make them enabled
Agent::Agent(const AgentConfiguration& cfg)
    : scene::AttachedObject(scene::AttachedObjectType::AGENT),
      configuration_(cfg),
      sensors_(),
      controls_(scene::ObjectControls::create()) {
  for (sensor::SensorSpec::ptr spec : cfg.sensorSpecifications) {
    // TODO: this should take type into account to create appropriate
    // sensor

    // CAREFUL: the sensor remains "invalid" until it is attached to a
    // scene node
    sensors_.add(sensor::PinholeCamera::create(spec));
  }
}

Agent::Agent(const AgentConfiguration& cfg, scene::SceneNode& agentNode)
    : Agent(cfg) {
  // attach the agent to the agent scene node
  attach(agentNode);

  // now, setup (attach, transform) the sensors_
  for (auto& sensor : sensors_.getSensors()) {
    auto& sensorNode = agentNode.createChild();
    sensor.second->attach(sensorNode);  // transformed within
  }
}

Agent::~Agent() {
  LOG(INFO) << "Deconstructing Agent";
  sensors_.clear();
}

void Agent::detach() {
  AttachedObject::detach();

  // traverse all the sensors, and detach them as well
  auto& sensors = sensors_.getSensors();
  for (auto& sensor : sensors) {
    sensor.second->detach();
  }
}

void Agent::act(const std::string& actionName) {
  ASSERT(isValid());
  const ActionSpec& actionSpec = *configuration_.actionSpace.at(actionName);
  if (BodyActions.find(actionSpec.name) != BodyActions.end()) {
    controls_->action(*node_, actionSpec.name,
                      actionSpec.actuation.at("amount"),
                      /*applyFilter=*/true);
  } else {
    for (auto p : sensors_.getSensors()) {
      controls_->action(p.second->object(), actionSpec.name,
                        actionSpec.actuation.at("amount"),
                        /*applyFilter=*/false);
    }
  }
}

void Agent::getState(AgentState::ptr state) const {
  ASSERT(isValid());
  // TODO this should be done less hackishly
  state->position =
      cast<vec3f>(object().absoluteTransformation().translation());
  state->rotation = quatf(object().rotation()).coeffs();
  // TODO other state members when implemented
}

void Agent::setState(const AgentState& state,
                     const bool resetSensors /*= true*/) {
  ASSERT(isValid());

  object().setTranslation(Magnum::Vector3(state.position));

  const Eigen::Map<const quatf> rot(state.rotation.data());
  CHECK_LT(std::abs(rot.norm() - 1.0),
           2.0 * Magnum::Math::TypeTraits<float>::epsilon())
      << state.rotation << " not a valid rotation";
  object().setRotation(Magnum::Quaternion(quatf(rot)).normalized());

  if (resetSensors) {
    for (auto p : sensors_.getSensors()) {
      p.second->setTransformationFromSpec();
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
  return a.height == b.height && a.radius == b.radius && a.mass == b.mass &&
         a.linearAcceleration == b.linearAcceleration &&
         a.angularAcceleration == b.angularAcceleration &&
         a.linearFriction == b.linearFriction &&
         a.angularFriction == b.angularFriction &&
         a.coefficientOfRestitution == b.coefficientOfRestitution &&
         esp::equal(a.sensorSpecifications, b.sensorSpecifications) &&
         esp::equal(a.actionSpace, b.actionSpace) && a.bodyType == b.bodyType;
}
bool operator!=(const AgentConfiguration& a, const AgentConfiguration& b) {
  return !(a == b);
}

}  // namespace agent
}  // namespace esp
