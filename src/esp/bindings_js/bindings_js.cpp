// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <emscripten/bind.h>

#include <Magnum/EigenIntegration/GeometryIntegration.h>
#include <Magnum/EigenIntegration/Integration.h>

namespace em = emscripten;

#include "esp/scene/SemanticScene.h"
#include "esp/sensor/CameraSensor.h"
#include "esp/sim/Simulator.h"

using namespace esp;
using namespace esp::agent;
using namespace esp::core;
using namespace esp::geo;
using namespace esp::gfx;
using namespace esp::nav;
using namespace esp::scene;
using namespace esp::sensor;
using namespace esp::sim;

// Consider
// https://becominghuman.ai/passing-and-returning-webassembly-array-parameters-a0f572c65d97
em::val Observation_getData(Observation& obs) {
  auto buffer = obs.buffer;
  if (buffer != nullptr) {
    return em::val(
        em::typed_memory_view(buffer->data.size(), buffer->data.data()));
  } else {
    return em::val::undefined();
  }
}

ObservationSpace Simulator_getAgentObservationSpace(Simulator& sim,
                                                    int agentId,
                                                    std::string sensorId) {
  ObservationSpace space;
  sim.getAgentObservationSpace(agentId, sensorId, space);
  return space;
}

std::map<std::string, ObservationSpace> Simulator_getAgentObservationSpaces(
    Simulator& sim,
    int agentId) {
  std::map<std::string, ObservationSpace> spaces;
  sim.getAgentObservationSpaces(agentId, spaces);
  return spaces;
}

template <class T, typename... Targs>
static inline auto create(Targs&&... args) {
  return std::make_shared<T>(std::forward<Targs>(args)...);
}

Magnum::Quaternion toQuaternion(const vec4f& rot) {
  return Magnum::Quaternion(quatf(rot)).normalized();
}

Magnum::Quaternion Quaternion_mul(const Magnum::Quaternion& q1,
                                  const Magnum::Quaternion& q2) {
  return q1 * q2;
}

Observation Sensor_getObservation(Sensor& sensor, Simulator& sim) {
  Observation ret;
  if (CameraSensor * camera{dynamic_cast<CameraSensor*>(&sensor)})
    camera->getObservation(sim, ret);
  return ret;
}

vec3f toVec3f(const Magnum::Vector3& pos) {
  return vec3f(pos.x(), pos.y(), pos.z());
}

vec4f toVec4f(const Magnum::Quaternion& rot) {
  return vec4f(rot.vector().x(), rot.vector().y(), rot.vector().z(),
               rot.scalar());
}

void Sensor_setLocalTransform(Sensor& sensor,
                              const vec3f& pos,
                              const vec4f& rot) {
  SceneNode& node{sensor.node()};

  node.resetTransformation();
  node.translate(Magnum::Vector3(pos));
  node.setRotation(Magnum::Quaternion(quatf(rot)).normalized());
}

EMSCRIPTEN_BINDINGS(habitat_sim_bindings_js) {
  em::function("toQuaternion", &toQuaternion);
  em::function("toVec3f", &toVec3f);
  em::function("toVec4f", &toVec4f);

  em::register_vector<SensorSpec::ptr>("VectorSensorSpec");
  em::register_vector<size_t>("VectorSizeT");
  em::register_vector<std::string>("VectorString");
  em::register_vector<std::shared_ptr<SemanticCategory>>(
      "VectorSemanticCategories");
  em::register_vector<std::shared_ptr<SemanticObject>>("VectorSemanticObjects");

  em::register_map<std::string, float>("MapStringFloat");
  em::register_map<std::string, std::string>("MapStringString");
  em::register_map<std::string, Sensor::ptr>("MapStringSensor");
  em::register_map<std::string, SensorSpec::ptr>("MapStringSensorSpec");
  em::register_map<std::string, Observation>("MapStringObservation");
  em::register_map<std::string, ActionSpec::ptr>("ActionSpace");

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

  em::value_object<std::pair<vec3f, vec3f>>("aabb")
      .field("min", &std::pair<vec3f, vec3f>::first)
      .field("max", &std::pair<vec3f, vec3f>::second);

  em::class_<Magnum::Rad>("Rad").constructor<float>();

  em::class_<Magnum::Vector3>("Vector3")
      .constructor<Magnum::Vector3>()
      .constructor<float, float, float>()
      .function("x", em::select_overload<float&()>(&Magnum::Vector3::x))
      .function("y", em::select_overload<float&()>(&Magnum::Vector3::y))
      .function("z", em::select_overload<float&()>(&Magnum::Vector3::z))
      .class_function("xAxis", &Magnum::Vector3::xAxis)
      .class_function("yAxis", &Magnum::Vector3::yAxis)
      .class_function("zAxis", &Magnum::Vector3::zAxis);

  em::class_<Magnum::Quaternion>("Quaternion")
      .constructor<Magnum::Vector3, float>()
      .constructor<Magnum::Vector3>()
      .function("normalized", &Magnum::Quaternion::normalized)
      .function("inverted", &Magnum::Quaternion::inverted)
      .function("transformVector", &Magnum::Quaternion::transformVector)
      .class_function("mul", &Quaternion_mul)
      .class_function("rotation", &Magnum::Quaternion::rotation);

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
      .property("coefficientOfRestitution",
                &AgentConfiguration::coefficientOfRestitution)
      .property("sensorSpecifications",
                &AgentConfiguration::sensorSpecifications);

  em::class_<ActionSpec>("ActionSpec")
      .smart_ptr_constructor(
          "ActionSpec",
          &ActionSpec::create<const std::string&, const ActuationMap&>)
      .property("name", &ActionSpec::name)
      .property("actuation", &ActionSpec::actuation);

  em::class_<PathFinder>("PathFinder")
      .smart_ptr<PathFinder::ptr>("PathFinder::ptr")
      .property("bounds", &PathFinder::bounds)
      .function("isNavigable", &PathFinder::isNavigable);

  em::class_<SensorSuite>("SensorSuite")
      .smart_ptr_constructor("SensorSuite", &SensorSuite::create<>)
      .function("get", &SensorSuite::get);

  em::enum_<SensorType>("SensorType")
      .value("NONE", SensorType::None)
      .value("COLOR", SensorType::Color)
      .value("DEPTH", SensorType::Depth)
      .value("NORMAL", SensorType::Normal)
      .value("SEMANTIC", SensorType::Semantic)
      .value("PATH", SensorType::Path)
      .value("GOAL", SensorType::Goal)
      .value("FORCE", SensorType::Force)
      .value("TENSOR", SensorType::Tensor)
      .value("TEXT", SensorType::Text);

  em::enum_<SensorSubType>("SensorSubType")
      .value("NONE", SensorSubType::None)
      .value("PINHOLE", SensorSubType::Pinhole)
      .value("ORTHOGRAPHIC", SensorSubType::Orthographic);
  em::class_<SensorSpec>("SensorSpec")
      .smart_ptr_constructor("SensorSpec", &SensorSpec::create<>)
      .property("uuid", &SensorSpec::uuid)
      .property("sensorType", &SensorSpec::sensorType)
      .property("sensorSubtype", &SensorSpec::sensorSubType)
      .property("position", &SensorSpec::position)
      .property("orientation", &SensorSpec::orientation);

  em::class_<VisualSensorSpec, em::base<SensorSpec>>("VisualSensorSpec")
      .smart_ptr_constructor("VisualSensorSpec", &VisualSensorSpec::create<>)
      .property("resolution", &VisualSensorSpec::resolution)
      .property("channels", &VisualSensorSpec::channels)
      .property("near", &VisualSensorSpec::near)
      .property("far", &VisualSensorSpec::far)
      .property("gpu2gpu_transfer", &VisualSensorSpec::gpu2gpuTransfer);

  em::class_<CameraSensorSpec, em::base<VisualSensorSpec>>("CameraSensorSpec")
      .smart_ptr_constructor("CameraSensorSpec", &CameraSensorSpec::create<>)
      .property("ortho_scale", &CameraSensorSpec::orthoScale);

  em::class_<Sensor>("Sensor")
      .smart_ptr<Sensor::ptr>("Sensor::ptr")
      .function("getObservation", &Sensor_getObservation)
      .function("setLocalTransform", &Sensor_setLocalTransform)
      .function("specification", &Sensor::specification);

  em::class_<SimulatorConfiguration>("SimulatorConfiguration")
      .smart_ptr_constructor("SimulatorConfiguration",
                             &SimulatorConfiguration::create<>)
      .property("scene_id", &SimulatorConfiguration::activeSceneName)
      .property("defaultAgentId", &SimulatorConfiguration::defaultAgentId)
      .property("defaultCameraUuid", &SimulatorConfiguration::defaultCameraUuid)
      .property("gpuDeviceId", &SimulatorConfiguration::gpuDeviceId)
      .property("compressTextures", &SimulatorConfiguration::compressTextures);

  em::class_<AgentState>("AgentState")
      .smart_ptr_constructor("AgentState", &AgentState::create<>)
      .property("position", &AgentState::position)
      .property("rotation", &AgentState::rotation)
      .property("velocity", &AgentState::velocity)
      .property("angularVelocity", &AgentState::angularVelocity)
      .property("force", &AgentState::force)
      .property("torque", &AgentState::torque);

  em::class_<Agent>("Agent")
      .smart_ptr<Agent::ptr>("Agent::ptr")
      .property("config",
                em::select_overload<const AgentConfiguration&() const>(
                    &Agent::getConfig))
      .property("sensorSuite", em::select_overload<const SensorSuite&() const>(
                                   &Agent::getSensorSuite))
      .function("getState", &Agent::getState)
      .function("setState", &Agent::setState)
      .function("hasAction", &Agent::hasAction)
      .function("act", &Agent::act);

  em::class_<Observation>("Observation")
      .smart_ptr_constructor("Observation", &Observation::create<>)
      .function("getData", &Observation_getData);

  em::class_<ObservationSpace>("ObservationSpace")
      .smart_ptr_constructor("ObservationSpace", &ObservationSpace::create<>)
      .property("dataType", &ObservationSpace::dataType)
      .property("shape", &ObservationSpace::shape);

  em::class_<SemanticCategory>("SemanticCategory")
      .smart_ptr<SemanticCategory::ptr>("SemanticCategory::ptr")
      .function("getIndex", &SemanticCategory::index)
      .function("getName", &SemanticCategory::name);

  em::class_<SemanticObject>("SemanticObject")
      .smart_ptr<SemanticObject::ptr>("SemanticObject::ptr")
      .property("category", &SemanticObject::category);

  em::class_<SemanticScene>("SemanticScene")
      .smart_ptr<SemanticScene::ptr>("SemanticScene::ptr")
      .property("categories", &SemanticScene::categories)
      .property("objects", &SemanticScene::objects);

  em::class_<Simulator>("Simulator")
      .smart_ptr_constructor("Simulator",
                             &Simulator::create<const SimulatorConfiguration&>)
      .function("getSemanticScene", &Simulator::getSemanticScene)
      .function("seed", &Simulator::seed)
      .function("reconfigure", &Simulator::reconfigure)
      .function("reset", &Simulator::reset)
      .function("getAgentObservations", &Simulator::getAgentObservations)
      .function("getAgentObservation", &Simulator::getAgentObservation)
      .function("displayObservation", &Simulator::displayObservation)
      .function("getAgentObservationSpaces",
                &Simulator_getAgentObservationSpaces)
      .function("getAgentObservationSpace", &Simulator_getAgentObservationSpace)
      .function("getAgent", &Simulator::getAgent)
      .function("getPathFinder", &Simulator::getPathFinder)
      .function("addAgent",
                em::select_overload<Agent::ptr(const AgentConfiguration&)>(
                    &Simulator::addAgent))
      .function("addAgentToNode",
                em::select_overload<Agent::ptr(const AgentConfiguration&,
                                               scene::SceneNode&)>(
                    &Simulator::addAgent));
}
