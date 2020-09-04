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
      .def_readwrite("random_seed", &SimulatorConfiguration::randomSeed)
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
      .def_readwrite("scene_light_setup",
                     &SimulatorConfiguration::sceneLightSetup)
      .def_readwrite("load_semantic_mesh",
                     &SimulatorConfiguration::loadSemanticMesh)
      .def(py::self == py::self)
      .def(py::self != py::self);

  // ==== Simulator ====
  py::class_<Simulator, Simulator::ptr>(m, "Simulator")
      .def(py::init<const SimulatorConfiguration&>())
      .def("get_active_scene_graph", &Simulator::getActiveSceneGraph,
           R"(PYTHON DOES NOT GET OWNERSHIP)",
           py::return_value_policy::reference)
      .def("get_active_semantic_scene_graph",
           &Simulator::getActiveSemanticSceneGraph,
           R"(PYTHON DOES NOT GET OWNERSHIP)",
           py::return_value_policy::reference)
      .def_property_readonly("semantic_scene", &Simulator::getSemanticScene, R"(
        The semantic scene graph

        .. note-warning::

            Not avaliable for all datasets
            )")
      .def_property_readonly("renderer", &Simulator::getRenderer)
      .def("seed", &Simulator::seed, "new_seed"_a)
      .def("reconfigure", &Simulator::reconfigure, "configuration"_a)
      .def("reset", &Simulator::reset)
      .def("close", &Simulator::close)
      .def_property("pathfinder", &Simulator::getPathFinder,
                    &Simulator::setPathFinder)
      .def_property_readonly("gpu_device", &Simulator::gpuDevice)
      .def_property_readonly("random", &Simulator::random)
      .def_property("frustum_culling", &Simulator::isFrustumCullingEnabled,
                    &Simulator::setFrustumCullingEnabled,
                    R"(Enable or disable the frustum culling)")
      /* --- Physics functions --- */

      .def("get_template_handle_by_ID", &Simulator::getObjectTemplateHandleByID,
           "object_id"_a)
      .def("get_template_handles", &Simulator::getObjectTemplateHandles,
           "search_str"_a = "", "contains"_a = true)
      .def("add_object", &Simulator::addObject, "object_lib_index"_a,
           "attachment_node"_a = nullptr,
           "light_setup_key"_a = assets::ResourceManager::DEFAULT_LIGHTING_KEY,
           "scene_id"_a = 0)
      .def("add_object_by_handle", &Simulator::addObjectByHandle,
           "object_lib_handle"_a, "attachment_node"_a = nullptr,
           "light_setup_key"_a = assets::ResourceManager::DEFAULT_LIGHTING_KEY,
           "scene_id"_a = 0)
      .def("get_physics_object_library_size",
           &Simulator::getPhysicsObjectLibrarySize)
      .def("get_object_template", &Simulator::getObjectTemplate,
           "object_template_id"_a, py::return_value_policy::reference)
      .def("load_object_configs", &Simulator::loadObjectConfigs, "path"_a)
      .def("load_object_template", &Simulator::registerObjectTemplate,
           "object_template"_a, "object_template_handle"_a)
      .def("get_object_initialization_template",
           &Simulator::getObjectInitializationTemplate, "object_id"_a,
           "scene_id"_a = 0)
      .def("remove_object", &Simulator::removeObject, "object_id"_a,
           "delete_object_node"_a = true, "delete_visual_node"_a = true,
           "scene_id"_a = 0)
      .def("get_object_motion_type", &Simulator::getObjectMotionType,
           "object_id"_a, "scene_id"_a = 0)
      .def("set_object_motion_type", &Simulator::setObjectMotionType,
           "motion_type"_a, "object_id"_a, "scene_id"_a = 0)
      .def("get_existing_object_ids", &Simulator::getExistingObjectIDs,
           "scene_id"_a = 0)
      .def("step_world", &Simulator::stepWorld, "dt"_a = 1.0 / 60.0)
      .def("get_world_time", &Simulator::getWorldTime)
      .def("get_gravity", &Simulator::getGravity, "scene_id"_a = 0)
      .def("set_gravity", &Simulator::setGravity, "gravity"_a, "scene_id"_a = 0)
      .def("get_object_scene_node", &Simulator::getObjectSceneNode,
           "object_id"_a, "scene_id"_a = 0)
      .def("set_transformation", &Simulator::setTransformation, "transform"_a,
           "object_id"_a, "scene_id"_a = 0)
      .def("get_transformation", &Simulator::getTransformation, "object_id"_a,
           "scene_id"_a = 0)
      .def("set_translation", &Simulator::setTranslation, "translation"_a,
           "object_id"_a, "scene_id"_a = 0)
      .def("get_translation", &Simulator::getTranslation, "object_id"_a,
           "scene_id"_a = 0)
      .def("set_rotation", &Simulator::setRotation, "rotation"_a, "object_id"_a,
           "scene_id"_a = 0)
      .def("get_rotation", &Simulator::getRotation, "object_id"_a,
           "scene_id"_a = 0)
      .def("get_object_velocity_control", &Simulator::getObjectVelocityControl,
           "object_id"_a, "scene_id"_a = 0)
      .def("set_linear_velocity", &Simulator::setLinearVelocity, "linVel"_a,
           "object_id"_a, "scene_id"_a = 0)
      .def("get_linear_velocity", &Simulator::getLinearVelocity, "object_id"_a,
           "scene_id"_a = 0)
      .def("set_angular_velocity", &Simulator::setAngularVelocity, "angVel"_a,
           "object_id"_a, "scene_id"_a = 0)
      .def("get_angular_velocity", &Simulator::getAngularVelocity,
           "object_id"_a, "scene_id"_a = 0)
      .def("apply_force", &Simulator::applyForce, "force"_a,
           "relative_position"_a, "object_id"_a, "scene_id"_a = 0)
      .def("apply_torque", &Simulator::applyTorque, "torque"_a, "object_id"_a,
           "scene_id"_a = 0)
      .def("contact_test", &Simulator::contactTest, "object_id"_a,
           "scene_id"_a = 0)
      .def("set_object_bb_draw", &Simulator::setObjectBBDraw, "draw_bb"_a,
           "object_id"_a, "scene_id"_a = 0)
      .def("recompute_navmesh", &Simulator::recomputeNavMesh, "pathfinder"_a,
           "navmesh_settings"_a, "include_static_objects"_a = false)
      .def("get_light_setup", &Simulator::getLightSetup,
           "key"_a = assets::ResourceManager::DEFAULT_LIGHTING_KEY)
      .def("set_light_setup", &Simulator::setLightSetup, "light_setup"_a,
           "key"_a = assets::ResourceManager::DEFAULT_LIGHTING_KEY)
      .def("set_object_light_setup", &Simulator::setObjectLightSetup,
           "object_id"_a, "light_setup_key"_a, "scene_id"_a = 0)

      /* --- URDF and ArticualtedObject API functions --- */
      .def("add_articulated_object_from_urdf",
           &Simulator::addArticulatedObjectFromURDF, "object_id"_a,
           "fixed_base"_a = false)
      .def("remove_articulated_object", &Simulator::removeArticulatedObject,
           "object_id"_a)
      .def("get_existing_articulated_object_ids",
           &Simulator::getExistingArticulatedObjectIDs, "scene_id"_a = 0)
      .def("set_articulated_object_root_state",
           &Simulator::setArticulatedObjectRootState, "object_id"_a, "state"_a)
      .def("get_articulated_object_root_state",
           &Simulator::getArticulatedObjectRootState, "object_id"_a)
      .def("set_articulated_object_forces",
           &Simulator::setArticulatedObjectForces, "object_id"_a, "forces"_a)
      .def("set_articulated_object_velocities",
           &Simulator::setArticulatedObjectVelocities, "object_id"_a, "vels"_a)
      .def("set_articulated_object_positions",
           &Simulator::setArticulatedObjectPositions, "object_id"_a,
           "positions"_a)
      .def("get_articulated_object_positions",
           &Simulator::getArticulatedObjectPositions, "object_id"_a)
      .def("get_articulated_object_velocities",
           &Simulator::getArticulatedObjectVelocities, "object_id"_a)
      .def("get_articulated_object_forces",
           &Simulator::getArticulatedObjectForces, "object_id"_a)
      .def("reset_articulated_object", &Simulator::resetArticulatedObject,
           "object_id"_a)
      .def("set_articulated_object_sleep",
           &Simulator::setArticulatedObjectSleep, "object_id"_a,
           "sleep"_a = true)
      .def("get_articulated_object_sleep",
           &Simulator::getArticulatedObjectSleep, "object_id"_a)
      .def("set_articulated_object_motion_type",
           &Simulator::setArticulatedObjectMotionType, "object_id"_a,
           "motion_type"_a)
      .def("get_articulated_object_motion_type",
           &Simulator::getArticulatedObjectMotionType, "object_id"_a)
      .def("get_num_articulated_links", &Simulator::getNumArticulatedLinks,
           "object_id"_a)
      .def("get_articulated_link_rigid_state",
           &Simulator::getArticulatedLinkRigidState, "object_id"_a, "link_id"_a)

      /* --- Joint Motor API --- */
      .def("create_joint_motor", &Simulator::createJointMotor, "object_id"_a,
           "dof"_a, "settings"_a)
      .def("remove_joint_motor", &Simulator::removeJointMotor, "object_id"_a,
           "motor_id"_a)
      .def("get_joint_motor_settings", &Simulator::getJointMotorSettings,
           "object_id"_a, "motor_id"_a)
      .def("update_joint_motor", &Simulator::updateJointMotor, "object_id"_a,
           "motor_id"_a, "settings"_a)
      .def("get_existing_joint_motors", &Simulator::getExistingJointMotors,
           "object_id"_a)
      .def("create_motors_for_all_dofs", &Simulator::createMotorsForAllDofs,
           "object_id"_a, "settings"_a = esp::physics::JointMotorSettings());
}

}  // namespace sim
}  // namespace esp
