// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"

#include <Magnum/ImageView.h>
#include <Magnum/Magnum.h>
#include <Magnum/SceneGraph/SceneGraph.h>

#include <Magnum/Python.h>
#include <Magnum/SceneGraph/Python.h>

#include "esp/gfx/RenderCamera.h"
#include "esp/gfx/Renderer.h"
#include "esp/scene/SemanticScene.h"
#include "esp/sim/Simulator.h"

namespace py = pybind11;
using py::literals::operator""_a;

namespace esp {
namespace sim {

void initSimBindings(py::module& m) {
  // ==== SimulatorConfiguration ====
  py::class_<SimulatorConfiguration, SimulatorConfiguration::ptr>(
      m, "SimulatorConfiguration")
      .def(py::init(&SimulatorConfiguration::create<>))
      .def_readwrite("scene", &SimulatorConfiguration::scene)
      .def_readwrite("default_agent_id",
                     &SimulatorConfiguration::defaultAgentId)
      .def_readwrite("default_camera_uuid",
                     &SimulatorConfiguration::defaultCameraUuid)
      .def_readwrite("gpu_device_id", &SimulatorConfiguration::gpuDeviceId)
      .def_readwrite("compress_textures",
                     &SimulatorConfiguration::compressTextures)
      .def_readwrite("allow_sliding", &SimulatorConfiguration::allowSliding)
      .def_readwrite("create_renderer", &SimulatorConfiguration::createRenderer)
      .def_readwrite("frustum_culling", &SimulatorConfiguration::frustumCulling)
      .def_readwrite("enable_physics", &SimulatorConfiguration::enablePhysics)
      .def_readwrite("physics_config_file",
                     &SimulatorConfiguration::physicsConfigFile)
      .def("__eq__",
           [](const SimulatorConfiguration& self,
              const SimulatorConfiguration& other) -> bool {
             return self == other;
           })
      .def("__neq__",
           [](const SimulatorConfiguration& self,
              const SimulatorConfiguration& other) -> bool {
             return self != other;
           });

  // ==== Simulator ====
  py::class_<Simulator, Simulator::ptr>(m, "Simulator")
      .def(py::init(&Simulator::create<const SimulatorConfiguration&>))
      .def("get_active_scene_graph", &Simulator::getActiveSceneGraph,
           R"(PYTHON DOES NOT GET OWNERSHIP)",
           pybind11::return_value_policy::reference)
      .def("get_active_semantic_scene_graph",
           &Simulator::getActiveSemanticSceneGraph,
           R"(PYTHON DOES NOT GET OWNERSHIP)",
           pybind11::return_value_policy::reference)
      .def_property_readonly("semantic_scene", &Simulator::getSemanticScene)
      .def_property_readonly("renderer", &Simulator::getRenderer)
      .def("seed", &Simulator::seed, "new_seed"_a)
      .def("reconfigure", &Simulator::reconfigure, "configuration"_a)
      .def("reset", &Simulator::reset)
      .def_property_readonly("gpu_device", &Simulator::gpuDevice)
      .def_property("frustum_culling", &Simulator::isFrustumCullingEnabled,
                    &Simulator::setFrustumCullingEnabled,
                    R"(Enable or disable the frustum culling)")
      /* --- Physics functions --- */
      .def("add_object", py::overload_cast<int, int>(&Simulator::addObject),
           "object_lib_index"_a, "scene_id"_a = 0)
      .def("add_object",
           py::overload_cast<int, const std::string&, int>(
               &Simulator::addObject),
           "object_lib_index"_a, "light_setup_key"_a, "scene_id"_a = 0)
      .def("get_physics_object_library_size",
           &Simulator::getPhysicsObjectLibrarySize)
      .def("remove_object", &Simulator::removeObject, "object_id"_a,
           "sceneID"_a = 0)
      .def("get_object_motion_type", &Simulator::getObjectMotionType,
           "object_id"_a, "sceneID"_a = 0)
      .def("set_object_motion_type", &Simulator::setObjectMotionType,
           "motion_type"_a, "object_id"_a, "sceneID"_a = 0)
      .def("get_existing_object_ids", &Simulator::getExistingObjectIDs,
           "sceneID"_a = 0)
      .def("step_world", &Simulator::stepWorld, "dt"_a = 1.0 / 60.0)
      .def("get_world_time", &Simulator::getWorldTime)
      .def("set_transformation", &Simulator::setTransformation, "transform"_a,
           "object_id"_a, "sceneID"_a = 0)
      .def("get_transformation", &Simulator::getTransformation, "object_id"_a,
           "sceneID"_a = 0)
      .def("set_translation", &Simulator::setTranslation, "translation"_a,
           "object_id"_a, "sceneID"_a = 0)
      .def("get_translation", &Simulator::getTranslation, "object_id"_a,
           "sceneID"_a = 0)
      .def("set_rotation", &Simulator::setRotation, "rotation"_a, "object_id"_a,
           "sceneID"_a = 0)
      .def("get_rotation", &Simulator::getRotation, "object_id"_a,
           "sceneID"_a = 0)
      .def("apply_force", &Simulator::applyForce, "force"_a,
           "relative_position"_a, "object_id"_a, "sceneID"_a = 0)
      .def("apply_torque", &Simulator::applyTorque, "torque"_a, "object_id"_a,
           "sceneID"_a = 0)
      .def("contact_test", &Simulator::contactTest, "object_id"_a,
           "sceneID"_a = 0)
      .def("recompute_navmesh", &Simulator::recomputeNavMesh, "pathfinder"_a,
           "navmesh_settings"_a)
      .def("get_light_setup", &Simulator::getLightSetup,
           "key"_a = assets::ResourceManager::DEFAULT_LIGHTING_KEY)
      .def("set_light_setup", &Simulator::setLightSetup, "light_setup"_a,
           "key"_a = assets::ResourceManager::DEFAULT_LIGHTING_KEY)
      .def("set_object_light_setup", &Simulator::setObjectLightSetup,
           "object_id"_a, "light_setup_key"_a, "scene_id"_a = 0);
}

}  // namespace sim
}  // namespace esp
