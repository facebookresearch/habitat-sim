// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SimulatorWithAgents.h"
#include "esp/gfx/Renderer.h"
#include "esp/io/io.h"

namespace esp {
namespace sim {

SimulatorWithAgents::SimulatorWithAgents(const gfx::SimulatorConfiguration& cfg)
    : gfx::Simulator() {
  // NOTE: NOT SO GREAT NOW THAT WE HAVE virtual functions
  //       Maybe better not to do this reconfigure
  reconfigure(cfg);
}

SimulatorWithAgents::~SimulatorWithAgents() {}

void SimulatorWithAgents::seed(uint32_t newSeed) {
  gfx::Simulator::seed(newSeed);
  pathfinder_->seed(newSeed);
}

void SimulatorWithAgents::reset() {
  // connect controls to navmesh if loaded
  gfx::Simulator::reset();

  for (int iAgent = 0; iAgent < agents_.size(); ++iAgent) {
    auto& agent = agents_[iAgent];
    agent::AgentState::ptr state = agent::AgentState::create();
    sampleRandomAgentState(state);
    agent->setState(*state);
    LOG(INFO) << "Reset agent i=" << iAgent
              << " position=" << state->position.transpose()
              << " rotation=" << state->rotation.transpose();
  }
}

void SimulatorWithAgents::reconfigure(const gfx::SimulatorConfiguration& cfg) {
  LOG(INFO) << "SimulatorWithAgents::reconfigure";
  if (cfg == config_) {
    reset();
    return;
  }

  std::string sceneFilename = cfg.scene.id;
  if (cfg.scene.filepaths.count("mesh")) {
    sceneFilename = cfg.scene.filepaths.at("mesh");
  }

  // create pathfinder and load navmesh if available
  pathfinder_ = nav::PathFinder::create();
  std::string navmeshFilename = io::changeExtension(sceneFilename, ".navmesh");
  if (cfg.scene.filepaths.count("navmesh")) {
    navmeshFilename = cfg.scene.filepaths.at("navmesh");
  }
  if (io::exists(navmeshFilename)) {
    LOG(INFO) << "Loading navmesh from " << navmeshFilename;
    pathfinder_->loadNavMesh(navmeshFilename);
    LOG(INFO) << "Loaded.";
  } else {
    LOG(WARNING) << "Navmesh file not found, checked at " << navmeshFilename;
  }

  gfx::Simulator::reconfigure(cfg);
}

// Agents
void SimulatorWithAgents::sampleRandomAgentState(
    agent::AgentState::ptr agentState) {
  agentState->position = pathfinder_->getRandomNavigablePoint();
  const float randomAngleRad = random_.uniform_float_01() * M_PI;
  quatf rotation(Eigen::AngleAxisf(randomAngleRad, vec3f::UnitY()));
  agentState->rotation = rotation.coeffs();
  // TODO: any other AgentState members should be randomized?
}

agent::Agent::ptr SimulatorWithAgents::addAgent(
    const agent::AgentConfiguration& agentConfig,
    scene::SceneNode& agentParentNode) {
  // initialize the agent, as well as all the sensors on it.

  // attach each agent, each sensor to a scene node, set the local
  // transformation of the sensor w.r.t. the agent (done internally in the
  // constructor of Agent)

  auto& agentNode = agentParentNode.createChild();
  agent::Agent::ptr ag = agent::Agent::create(agentNode, agentConfig);

  // Add a RenderTarget to each of the agent's sensors
  for (auto& it : ag->getSensorSuite().getSensors()) {
    renderer_->bindRenderTarget(it.second);
  }

  agents_.push_back(ag);
  // TODO: just do this once
  if (pathfinder_->isLoaded()) {
    ag->getControls()->setMoveFilterFunction(
        [&](const vec3f& start, const vec3f& end) {
          return pathfinder_->tryStep(start, end);
        });
  }

  return ag;
}

agent::Agent::ptr SimulatorWithAgents::addAgent(
    const agent::AgentConfiguration& agentConfig) {
  return addAgent(agentConfig, getActiveSceneGraph().getRootNode());
}

agent::Agent::ptr SimulatorWithAgents::getAgent(int agentId) {
  ASSERT(0 <= agentId && agentId < agents_.size());
  return agents_[agentId];
}

nav::PathFinder::ptr SimulatorWithAgents::getPathFinder() {
  return pathfinder_;
}

bool SimulatorWithAgents::displayObservation(int agentId,
                                             const std::string& sensorId) {
  agent::Agent::ptr ag = getAgent(agentId);

  if (ag != nullptr) {
    sensor::Sensor::ptr sensor = ag->getSensorSuite().get(sensorId);
    if (sensor != nullptr) {
      return sensor->displayObservation(*this);
    }
  }
  return false;
}

bool SimulatorWithAgents::getAgentObservation(
    int agentId,
    const std::string& sensorId,
    sensor::Observation& observation) {
  agent::Agent::ptr ag = getAgent(agentId);
  if (ag != nullptr) {
    sensor::Sensor::ptr sensor = ag->getSensorSuite().get(sensorId);
    if (sensor != nullptr) {
      return sensor->getObservation(*this, observation);
    }
  }
  return false;
}

int SimulatorWithAgents::getAgentObservations(
    int agentId,
    std::map<std::string, sensor::Observation>& observations) {
  observations.clear();
  agent::Agent::ptr ag = getAgent(agentId);
  if (ag != nullptr) {
    const std::map<std::string, sensor::Sensor::ptr>& sensors =
        ag->getSensorSuite().getSensors();
    for (std::pair<std::string, sensor::Sensor::ptr> s : sensors) {
      sensor::Observation obs;
      if (s.second->getObservation(*this, obs)) {
        observations[s.first] = obs;
      }
    }
  }
  return observations.size();
}

bool SimulatorWithAgents::getAgentObservationSpace(
    int agentId,
    const std::string& sensorId,
    sensor::ObservationSpace& space) {
  agent::Agent::ptr ag = getAgent(agentId);
  if (ag != nullptr) {
    sensor::Sensor::ptr sensor = ag->getSensorSuite().get(sensorId);
    if (sensor != nullptr) {
      return sensor->getObservationSpace(space);
    }
  }
  return false;
}

int SimulatorWithAgents::getAgentObservationSpaces(
    int agentId,
    std::map<std::string, sensor::ObservationSpace>& spaces) {
  spaces.clear();
  agent::Agent::ptr ag = getAgent(agentId);
  if (ag != nullptr) {
    const std::map<std::string, sensor::Sensor::ptr>& sensors =
        ag->getSensorSuite().getSensors();
    for (std::pair<std::string, sensor::Sensor::ptr> s : sensors) {
      sensor::ObservationSpace space;
      if (s.second->getObservationSpace(space)) {
        spaces[s.first] = space;
      }
    }
  }
  return spaces.size();
}

}  // namespace sim
}  // namespace esp
