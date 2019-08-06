// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <emscripten/bind.h>

namespace em = emscripten;

#include "esp/sim/SimulatorWithAgents.h"

using namespace esp;
using namespace esp::agent;
using namespace esp::core;
using namespace esp::geo;
using namespace esp::gfx;
using namespace esp::nav;
using namespace esp::scene;
using namespace esp::sensor;
using namespace esp::sim;

SceneConfiguration* SimulatorConfiguration_getScene(SimulatorConfiguration& config) 
  { return &config.scene; }

Agent* Simulator_addAgent(SimulatorWithAgents& sim, const AgentConfiguration& agentConfig) 
  { return sim.addAgent(agentConfig).get(); }

Agent* Simulator_addAgentToNode(SimulatorWithAgents& sim, const AgentConfiguration& agentConfig, SceneNode& sceneNode) 
  { return sim.addAgent(agentConfig, sceneNode).get(); }

Agent* Simulator_getAgent(SimulatorWithAgents& sim, int agentId) 
  { return sim.getAgent(agentId).get(); }

ActionSpace Agent_getActionSpace(Agent& agent) { 
  return agent.getConfig().actionSpace;
}

std::map<std::string, Sensor::ptr> Agent_getSensors(Agent& agent) { 
  return agent.getSensorSuite().getSensors();
}

std::map<std::string, SensorSpec::ptr> Agent_getSensorSpecs(Agent& agent) { 
  auto sensors = agent.getSensorSuite().getSensors();
  std::map<std::string, SensorSpec::ptr> specs;
  for (std::pair<std::string, Sensor::ptr> s : sensors) {
    specs[s.first] = s.second->specification();
  }
  return specs;
}

bool Agent_act(Agent& agent, const std::string& action) {
  if (agent.hasAction(action)) {
    agent.act(action);
    return true;
  } else {
    return false;
  }
}

// Consider https://becominghuman.ai/passing-and-returning-webassembly-array-parameters-a0f572c65d97
em::val Buffer_getBytes(Buffer::ptr buffer) {
  if (buffer != nullptr) {
    return em::val(em::typed_memory_view(buffer->totalBytes, (uint8_t*) buffer->data));
  } else {
    return em::val::undefined();
  }
}

em::val Buffer_getDataType(Buffer::ptr buffer) {
  if (buffer != nullptr) {
    return em::val(buffer->dataType);
  } else {
    return em::val::undefined();
  }
}

em::val Buffer_getShape(Buffer::ptr buffer) {
  if (buffer != nullptr) {
    return em::val(buffer->shape);
  } else {
    return em::val::undefined();
  }
}

em::val Observation_getBytes(Observation& obs) {
  return Buffer_getBytes(obs.buffer);
}

em::val Observation_getShape(Observation& obs) {
  return Buffer_getShape(obs.buffer);
}

em::val Observation_getDataType(Observation& obs) {
  return Buffer_getDataType(obs.buffer);
}

em::val ObservationSpace_getShape(ObservationSpace& space) {
  return em::val(space.shape);
}

em::val ObservationSpace_getDataType(ObservationSpace& space) {
  return em::val(space.dataType);
}

ObservationSpace Simulator_getAgentObservationSpace(SimulatorWithAgents& sim, int agentId, std::string sensorId) {
  ObservationSpace space;
  sim.getAgentObservationSpace(agentId, sensorId, space);
  return space;
}

std::map<std::string,ObservationSpace> Simulator_getAgentObservationSpaces(SimulatorWithAgents& sim, int agentId) {
  std::map<std::string,ObservationSpace> spaces;
  sim.getAgentObservationSpaces(agentId, spaces);
  return spaces;
}

EMSCRIPTEN_BINDINGS(habitat_sim_bindings_js) {
  em::value_array<vec2f>("vec2f")
        .element(em::index<0>())
        .element(em::index<1>());

  em::value_array<vec3f>("vec3f")
        .element(em::index<0>())
        .element(em::index<1>())
        .element(em::index<2>());

  em::value_array<vec4f>("vec4f")
        .element(em::index<0>())
        .element(em::index<1>())
        .element(em::index<2>())
        .element(em::index<3>());

  em::value_array<vec2i>("vec2i")
        .element(em::index<0>())
        .element(em::index<1>());

  em::value_array<vec3i>("vec3i")
        .element(em::index<0>())
        .element(em::index<1>())
        .element(em::index<2>());

  em::value_array<vec4i>("vec4i")
        .element(em::index<0>())
        .element(em::index<1>())
        .element(em::index<2>())
        .element(em::index<3>());

  em::class_<AgentConfiguration>("AgentConfiguration")
      .smart_ptr_constructor("AgentConfiguration",
                             &AgentConfiguration::create<>)
      .property("height", &AgentConfiguration::height)
      .property("radius", &AgentConfiguration::radius)
      .property("mass", &AgentConfiguration::mass)
      .property("linearAcceleration", &AgentConfiguration::linearAcceleration)
      .property("angularAcceleration", &AgentConfiguration::angularAcceleration)
      .property("linearFriction", &AgentConfiguration::linearFriction)
      .property("angularFriction", &AgentConfiguration::angularFriction)
      .property("coefficientOfRestitution", &AgentConfiguration::coefficientOfRestitution)
      .property("sensorSpecifications", &AgentConfiguration::sensorSpecifications);

  em::class_<ActionSpec>("ActionSpec")
      .smart_ptr_constructor("ActionSpec",
                             &ActionSpec::create<const std::string&, const ActuationMap&>)
      .property("name", &ActionSpec::name)
      .property("actuation", &ActionSpec::actuation);

  em::class_<SensorSpec>("SensorSpec")
      .smart_ptr_constructor("SensorSpec",
                             &SensorSpec::create<>)
      .property("uuid", &SensorSpec::uuid)
      .property("sensorType", &SensorSpec::sensorType)
      .property("sensorSubtype", &SensorSpec::sensorSubtype)
      .property("position", &SensorSpec::position)
      .property("orientation", &SensorSpec::orientation)
      .property("resolution", &SensorSpec::resolution)
      .property("parameters", &SensorSpec::parameters);

  em::class_<Sensor>("Sensor")
      .smart_ptr_constructor("Sensor",
                             &Sensor::create<SensorSpec::ptr>)
      .function("specification", &Sensor::specification);

  em::class_<SceneConfiguration>("SceneConfiguration")
      .smart_ptr_constructor("SceneConfiguration",
                             &SceneConfiguration::create<>)
      .property("dataset", &SceneConfiguration::dataset)
      .property("id", &SceneConfiguration::id)
      .property("filepaths", &SceneConfiguration::filepaths)
      .property("sceneUpDir", &SceneConfiguration::sceneUpDir)
      .property("sceneFrontDir", &SceneConfiguration::sceneFrontDir)
      .property("sceneScaleUnit", &SceneConfiguration::sceneScaleUnit);

  em::class_<SimulatorConfiguration>("SimulatorConfiguration")
      .smart_ptr_constructor("SimulatorConfiguration",
                             &SimulatorConfiguration::create<>)
      .function("getScene", &SimulatorConfiguration_getScene, em::allow_raw_pointers())
      .property("scene", &SimulatorConfiguration::scene)
      .property("defaultAgentId", &SimulatorConfiguration::defaultAgentId)
      .property("defaultCameraUuid",
                &SimulatorConfiguration::defaultCameraUuid)
      .property("gpuDeviceId", &SimulatorConfiguration::gpuDeviceId)
      .property("width", &SimulatorConfiguration::width)
      .property("height", &SimulatorConfiguration::height)
      .property("compressTextures", &SimulatorConfiguration::compressTextures);

  em::class_<AgentState>("AgentState")
      .smart_ptr_constructor("AgentState",
                             &AgentState::create<>)
      .property("position", &AgentState::position)
      .property("rotation", &AgentState::rotation)
      .property("velocity", &AgentState::velocity)
      .property("angularVelocity", &AgentState::angularVelocity)
      .property("force", &AgentState::force)
      .property("torque", &AgentState::torque);

  em::class_<Agent>("Agent")
      .smart_ptr_constructor("Agent",
                             &Agent::create<const AgentConfiguration&>)
      .function("getSensors", &Agent_getSensors) 
      .function("getSensorSpecs", &Agent_getSensorSpecs) 
      .function("getState", &Agent::getState)
      .function("getActionSpace", &Agent_getActionSpace)
      .function("hasAction", &Agent::hasAction)
      .function("act", &Agent::act);

  em::class_<Observation>("Observation")
      .smart_ptr_constructor("Observation",
                             &Observation::create<>)
      .function("getDataType", &Observation_getDataType)
      .function("getShape", &Observation_getShape)
//      .function("getArray", &Observation_getArray)
      .function("getBytes", &Observation_getBytes);

  em::class_<ObservationSpace>("ObservationSpace")
      .smart_ptr_constructor("ObservationSpace",
                             &ObservationSpace::create<>)
      .function("getDataType", &ObservationSpace_getDataType)
      .function("getShape", &ObservationSpace_getShape);

  em::class_<Simulator>("BaseSimulator")
      .smart_ptr_constructor("Simulator",
                             &Simulator::create<const SimulatorConfiguration&>)
      // .function("get_active_scene_graph", &Simulator::getActiveSceneGraph)
      // .function("get_active_semantic_scene_graph", &Simulator::getActiveSemanticSceneGraph)
      // .property("semantic_scene", [] (Simulator& self) { return &self.getSemanticScene(); } )
      // .property("renderer", &Simulator::getRenderer)
         .function("seed", &Simulator::seed)
         .function("reconfigure", &Simulator::reconfigure)
         .function("reset", &Simulator::reset);

  em::class_<SimulatorWithAgents, em::base<Simulator>>("Simulator")
      .smart_ptr_constructor("Simulator",
                             &SimulatorWithAgents::create<const SimulatorConfiguration&>)
      .function("seed", &SimulatorWithAgents::seed)
      .function("reconfigure", &SimulatorWithAgents::reconfigure)
      .function("reset", &SimulatorWithAgents::reset)
      .function("getAgentObservations", &SimulatorWithAgents::getAgentObservations)
      .function("getAgentObservation", &SimulatorWithAgents::getAgentObservation)
      .function("getAgentObservationSpaces", &Simulator_getAgentObservationSpaces)
      .function("getAgentObservationSpace", &Simulator_getAgentObservationSpace)
      .function("getAgent", &Simulator_getAgent, em::allow_raw_pointers())
      .function("addAgent", &Simulator_addAgent, em::allow_raw_pointers())
      .function("addAgentToNode", &Simulator_addAgentToNode, em::allow_raw_pointers());

  em::register_vector<SensorSpec::ptr>("VectorSensorSpec");
  em::register_vector<size_t>("VectorSizeT");
  em::register_vector<std::string>("VectorString");

  em::register_map<std::string, float>("MapStringFloat");
  em::register_map<std::string, std::string>("MapStringString");
  em::register_map<std::string, SensorSpec::ptr>("MapStringSensorSpec");
  em::register_map<std::string, Sensor::ptr>("MapStringSensor");
  em::register_map<std::string, Observation>("MapStringObservation");
  em::register_map<std::string, ActionSpec::ptr>("ActionSpace");
}
