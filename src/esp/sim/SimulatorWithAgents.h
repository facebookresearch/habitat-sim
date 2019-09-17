// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "esp/gfx/Simulator.h"

#include "esp/agent/Agent.h"
#include "esp/nav/PathFinder.h"

namespace esp {
namespace sim {

class SimulatorWithAgents : public gfx::Simulator {
 public:
  SimulatorWithAgents(const gfx::SimulatorConfiguration& cfg);
  virtual ~SimulatorWithAgents();

  virtual void reconfigure(const gfx::SimulatorConfiguration& cfg) override;
  virtual void reset() override;
  virtual void seed(uint32_t newSeed) override;

  //! sample a random valid AgentState in passed agentState
  void sampleRandomAgentState(agent::AgentState::ptr agentState);
  agent::Agent::ptr getAgent(int agentId);

  agent::Agent::ptr addAgent(const agent::AgentConfiguration& agentConfig,
                             scene::SceneNode& agentParentNode);
  agent::Agent::ptr addAgent(const agent::AgentConfiguration& agentConfig);

  /**
   * @brief Displays observations on default frame buffer for a
   * particular sensor of an agent
   * @param agentId    Id of the agent for which the observation is to
   *                   be returned
   * @param sensorId   Id of the sensor for which the observation is to
   *                   be returned
   */
  bool displayObservation(int agentId, const std::string& sensorId);
  bool getAgentObservation(int agentId,
                           const std::string& sensorId,
                           sensor::Observation& observation);
  int getAgentObservations(
      int agentId,
      std::map<std::string, sensor::Observation>& observations);

  bool getAgentObservationSpace(int agentId,
                                const std::string& sensorId,
                                sensor::ObservationSpace& space);
  int getAgentObservationSpaces(
      int agentId,
      std::map<std::string, sensor::ObservationSpace>& spaces);

  nav::PathFinder::ptr getPathFinder();

 protected:
  std::vector<agent::Agent::ptr> agents_;
  nav::PathFinder::ptr pathfinder_;
  ESP_SMART_POINTERS(SimulatorWithAgents)
};

}  // namespace sim
}  // namespace esp
