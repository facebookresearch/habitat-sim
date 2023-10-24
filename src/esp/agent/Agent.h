// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#ifndef ESP_AGENT_AGENT_H_
#define ESP_AGENT_AGENT_H_

#include <Magnum/SceneGraph/Object.h>
#include <map>
#include <set>
#include <string>

#include "esp/core/Esp.h"
#include "esp/core/EspEigen.h"
#include "esp/scene/SceneNode.h"

namespace esp {
namespace scene {
class ObjectControls;
}
namespace sensor {
class Sensor;
struct SensorSpec;
class SensorSuite;
}  // namespace sensor
namespace agent {

/**
 * @brief Struct describing the physical state of an agent
 */
struct AgentState {
  /**
   * @brief the position of the agent
   */
  vec3f position = {0, 0, 0};
  /**
   * @brief the agent's rotation. TODO : This exposes the rotation quaternion
   * x,y,z,w as vec4f for pybind11 interop, replace with quatf when we have
   * custom pybind11 type conversion for quaternions
   */
  vec4f rotation = {0, 0, 0, 1};
  ESP_SMART_POINTERS(AgentState)
};

/**
 * @brief Type to describe the characteristics of an action (i.e. angle to turn
 * for a turn action)
 */
typedef std::map<std::string, float> ActuationMap;

/**
 * @brief Struct describing an action (i.e. name -> agent actuation).
 */
struct ActionSpec {
  /**
   * @brief Constructor
   */
  explicit ActionSpec(const std::string& _name, const ActuationMap& _actuation)
      : name(_name), actuation(_actuation) {}
  /**
   * @brief action name
   */
  std::string name;
  /**
   * @brief linear, angular forces, joint torques, sensor actuation
   */
  ActuationMap actuation;
  ESP_SMART_POINTERS(ActionSpec)
};

/**
 * @brief Verify @ref ActionSpec equality.
 */
bool operator==(const ActionSpec& a, const ActionSpec& b);

/**
 * @brief Verify @ref ActionSpec inequality.
 */
bool operator!=(const ActionSpec& a, const ActionSpec& b);

/**
 * @brief Represents a set of possible agent actions.
 */
typedef std::map<std::string, ActionSpec::ptr> ActionSpace;

/**
 * @brief Struct describing a configuration for an embodied Agent
 */
struct AgentConfiguration {
  /**
   * @brief The agent's height
   */
  float height = 1.5;

  /**
   * @brief The radius of the colliding capsule around the agent
   */
  float radius = 0.1;

  /**
   * @brief A vector of @ref esp::sensor::SensorSpec that describe the sensors attached to this agent.
   */
  std::vector<std::shared_ptr<sensor::SensorSpec>> sensorSpecifications = {};

  /**
   * @brief The default ActionSpace for this agent
   */
  ActionSpace actionSpace = {
      {"moveForward",
       ActionSpec::create("moveForward", ActuationMap{{"amount", 0.25f}})},
      {"lookUp", ActionSpec::create("lookUp", ActuationMap{{"amount", 10.0f}})},
      {"lookDown",
       ActionSpec::create("lookDown", ActuationMap{{"amount", 10.0f}})},
      {"turnLeft",
       ActionSpec::create("turnLeft", ActuationMap{{"amount", 10.0f}})},
      {"turnRight",
       ActionSpec::create("turnRight", ActuationMap{{"amount", 10.0f}})}};
  /**
   * @brief The body type of this agent.
   */
  std::string bodyType = "cylinder";

  ESP_SMART_POINTERS(AgentConfiguration)
};

/**
 *  @brief Verify @ref AgentConfiguration equality.
 */
bool operator==(const AgentConfiguration& a, const AgentConfiguration& b);

/**
 * @brief Verify @ref AgentConfiguration inequality.
 */
bool operator!=(const AgentConfiguration& a, const AgentConfiguration& b);

/**
 * @brief Class that represents an agent that can act within an environment
 */
class Agent : public Magnum::SceneGraph::AbstractFeature3D {
 public:
  /**
   * @brief constructor: the status of the agent, sensors is "valid" after
   * construction; user can use them immediately
   */
  explicit Agent(scene::SceneNode& agentNode, const AgentConfiguration& cfg);

  ~Agent() override;

  /**
   * @brief Get the scene node being attached to.
   */
  scene::SceneNode& node() { return object(); }

  /**
   * @brief Get the scene node being attached to.
   */
  const scene::SceneNode& node() const { return object(); }

  /**
   * @brief Overloads to avoid confusion
   */
  scene::SceneNode& object() {
    return static_cast<scene::SceneNode&>(
        Magnum::SceneGraph::AbstractFeature3D::object());
  }

  /**
   * @brief Overloads to avoid confusion
   */
  const scene::SceneNode& object() const {
    return static_cast<const scene::SceneNode&>(
        Magnum::SceneGraph::AbstractFeature3D::object());
  }

  /**
   * @brief Perform an action.
   * @param actionName the name of the action to perform
   * @return Whether the named action is available to the agent
   */
  bool act(const std::string& actionName);

  /**
   * @brief Verify whether the named action is available to the agent.
   * @param actionName the name of the action to perform
   * @return Whether the named action is available to the agent
   */
  bool hasAction(const std::string& actionName) const;

  /**
   * @brief Return the agent to the saved @p initialState_
   */
  void reset();

  /**
   * @brief Populate the passed state with the agent's current positon and
   * rotation.
   * @param state The @ref AgentState variable to populate with the current state of this agent.
   */
  void getState(const AgentState::ptr& state) const;

  /**
   * @brief Set the agent to the passsed state.
   * @param state The state to set the agent to
   * @param resetSensors Whether to reset the agent's sensors or not.
   */
  void setState(const AgentState& state, bool resetSensors = true);

  /**
   * @brief Set the agent to the passsed state and set that state to be the @p
   * initialState_
   * @param state The state to set to, and save as initial state
   * @param resetSensors Whether to reset the agent's sensors or not.
   */
  void setInitialState(const AgentState& state,
                       const bool resetSensors = true) {
    initialState_ = state;
    setState(state, resetSensors);
  }

  /**
   * @brief Retrieve the @ref esp::scene::ObjectControls specified for this agent.
   */
  std::shared_ptr<scene::ObjectControls> getControls() { return controls_; }

  /**
   * @brief Return SensorSuite containing references to superset of all Sensors
   * held by this Agent's SceneNode and its children
   */
  sensor::SensorSuite& getSubtreeSensorSuite() {
    return node().getSubtreeSensorSuite();
  }

  /**
   * @brief Return map containing references to superset of all Sensors held by
   * this Agent's SceneNode and its children values.
   * Keys of map are uuid strings, values are references to Sensors with that
   * uuid
   */
  std::map<std::string, std::reference_wrapper<sensor::Sensor>>&
  getSubtreeSensors() {
    return node().getSubtreeSensors();
  }

  /**
   * @brief Retrieve a const reference to this agent's configuration.
   */
  const AgentConfiguration& getConfig() const { return configuration_; }

  /**
   * @brief Retrieve a reference to this agent's configuration.
   */
  AgentConfiguration& getConfig() { return configuration_; }

  /**
   * @brief Set of actions that are applied to the body of the agent.  These
   * actions update both the absolute position/rotation of the agent and the
   * sensor. Non-body actions only effect the absolute position and rotation of
   * the sensors (only effects their position/rotation relative to the agent's
   * body)
   */
  static const std::set<std::string> BodyActions;

 private:
  /**
   * @brief The configuration of this agent.
   */
  AgentConfiguration configuration_;

  std::shared_ptr<scene::ObjectControls> controls_;
  AgentState initialState_;

  ESP_SMART_POINTERS(Agent)
};

}  // namespace agent
}  // namespace esp

#endif  // ESP_AGENT_AGENT_H_
