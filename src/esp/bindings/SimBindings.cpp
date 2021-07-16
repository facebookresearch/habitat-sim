// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"

#include <Magnum/ImageView.h>
#include <Magnum/Magnum.h>
#include <Magnum/SceneGraph/SceneGraph.h>

#include <Magnum/PythonBindings.h>
#include <Magnum/SceneGraph/PythonBindings.h>

#include "esp/gfx/RenderCamera.h"
#include "esp/gfx/Renderer.h"
#include "esp/gfx/replay/ReplayManager.h"
#include "esp/scene/SemanticScene.h"
#include "esp/sim/Simulator.h"
#include "esp/sim/SimulatorConfiguration.h"

namespace py = pybind11;
using py::literals::operator""_a;

namespace esp {
namespace sim {

void initSimBindings(py::module& m) {
  // ==== SimulatorConfiguration ====
  py::class_<SimulatorConfiguration, SimulatorConfiguration::ptr>(
      m, "SimulatorConfiguration")
      .def(py::init(&SimulatorConfiguration::create<>))
      .def_readwrite(
          "scene_dataset_config_file",
          &SimulatorConfiguration::sceneDatasetConfigFile,
          R"(The location of the scene dataset configuration file that describes the
          dataset to be used.)")
      .def_readwrite(
          "scene_id", &SimulatorConfiguration::activeSceneName,
          R"(Either the name of a stage asset or configuration file, or else the name of a scene
          instance configuration, used to initialize the simulator world.)")
      .def_readwrite("random_seed", &SimulatorConfiguration::randomSeed)
      .def_readwrite("default_agent_id",
                     &SimulatorConfiguration::defaultAgentId)
      .def_readwrite("default_camera_uuid",
                     &SimulatorConfiguration::defaultCameraUuid)
      .def_readwrite("gpu_device_id", &SimulatorConfiguration::gpuDeviceId)
      .def_readwrite("allow_sliding", &SimulatorConfiguration::allowSliding)
      .def_readwrite("create_renderer", &SimulatorConfiguration::createRenderer)
      .def_readwrite(
          "leave_context_with_background_renderer",
          &SimulatorConfiguration::leaveContextWithBackgroundRenderer,
          R"(See tutorials/async_rendering.py)")
      .def_readwrite("frustum_culling", &SimulatorConfiguration::frustumCulling)
      .def_readwrite("enable_physics", &SimulatorConfiguration::enablePhysics)
      .def_readwrite(
          "enable_gfx_replay_save",
          &SimulatorConfiguration::enableGfxReplaySave,
          R"(Enable replay recording. See sim.gfx_replay.save_keyframe.)")
      .def_readwrite("physics_config_file",
                     &SimulatorConfiguration::physicsConfigFile)
      .def_readwrite(
          "override_scene_light_defaults",
          &SimulatorConfiguration::overrideSceneLightDefaults,
          R"(Override scene lighting setup to use with value specified below.)")
      .def_readwrite("scene_light_setup",
                     &SimulatorConfiguration::sceneLightSetup)
      .def_readwrite("load_semantic_mesh",
                     &SimulatorConfiguration::loadSemanticMesh)
      .def_readwrite(
          "force_separate_semantic_scene_graph",
          &SimulatorConfiguration::forceSeparateSemanticSceneGraph,
          R"(Required to support playback of any gfx replay that includes a
          stage with a semantic mesh. Set to false otherwise.)")
      .def_readwrite("requires_textures",
                     &SimulatorConfiguration::requiresTextures)
      .def(py::self == py::self)
      .def(py::self != py::self);

  // ==== Simulator ====
  py::class_<Simulator, Simulator::ptr>(m, "Simulator")
      // modify constructor to pass MetadataMediator
      .def(py::init<const SimulatorConfiguration&,
                    esp::metadata::MetadataMediator::ptr>())
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

            Not available for all datasets
            )")
      .def_property_readonly("renderer", &Simulator::getRenderer)
      .def_property_readonly(
          "gfx_replay_manager", &Simulator::getGfxReplayManager,
          R"(Use gfx_replay_manager for replay recording and playback.)")
      .def("seed", &Simulator::seed, "new_seed"_a)
      .def("reconfigure", &Simulator::reconfigure, "configuration"_a)
      .def("reset", &Simulator::reset)
      .def(
          "close", &Simulator::close, "destroy"_a = true,
          R"(Free all loaded assets and GPU contexts. Use destroy=true except where noted in tutorials/async_rendering.py.)")
      .def_property("pathfinder", &Simulator::getPathFinder,
                    &Simulator::setPathFinder)
      .def_property(
          "navmesh_visualization", &Simulator::isNavMeshVisualizationActive,
          &Simulator::setNavMeshVisualization,
          R"(Enable or disable wireframe visualization of current pathfinder's NavMesh.)")
      .def_property_readonly("gpu_device", &Simulator::gpuDevice)
      .def_property_readonly("random", &Simulator::random)
      .def_property("frustum_culling", &Simulator::isFrustumCullingEnabled,
                    &Simulator::setFrustumCullingEnabled,
                    R"(Enable or disable the frustum culling)")
      .def_property(
          "active_dataset", &Simulator::getActiveSceneDatasetName,
          &Simulator::setActiveSceneDatasetName,
          R"(The currently active dataset being used.  Will attempt to load
            configuration files specified if does not already exist.)")
      /* --- Template Manager accessors --- */
      // We wish a copy of the metadata mediator smart pointer so that we
      // increment its ref counter
      .def_property(
          "metadata_mediator", &Simulator::getMetadataMediator,
          &Simulator::setMetadataMediator, py::return_value_policy::copy,
          R"(This construct manages all configuration template managers
          and the Scene Dataset Configurations)")
      .def("get_asset_template_manager", &Simulator::getAssetAttributesManager,
           pybind11::return_value_policy::reference,
           R"(Get the current dataset's AssetAttributesManager instance
            for configuring primitive asset templates.)")
      .def("get_lighting_template_manager",
           &Simulator::getLightLayoutAttributesManager,
           pybind11::return_value_policy::reference,
           R"(Get the current dataset's LightLayoutAttributesManager instance
            for configuring light templates and layouts.)")
      .def("get_object_template_manager",
           &Simulator::getObjectAttributesManager,
           pybind11::return_value_policy::reference,
           R"(Get the current dataset's ObjectAttributesManager instance
            for configuring object templates.)")
      .def("get_physics_template_manager",
           &Simulator::getPhysicsAttributesManager,
           pybind11::return_value_policy::reference,
           R"(Get the current PhysicsAttributesManager instance
            for configuring PhysicsManager templates.)")
      .def("get_stage_template_manager", &Simulator::getStageAttributesManager,
           pybind11::return_value_policy::reference,
           R"(Get the current dataset's StageAttributesManager instance
            for configuring simulation stage templates.)")
      .def("get_rigid_object_manager", &Simulator::getRigidObjectManager,
           pybind11::return_value_policy::reference,
           R"(Get the manager responsible for organizing and accessing all the
           currently constructed rigid objects.)")
      .def("get_articulated_object_manager",
           &Simulator::getArticulatedObjectManager,
           pybind11::return_value_policy::reference,
           R"(Get the manager responsible for organizing and accessing all the
          currently constructed articulated objects.)")
      .def(
          "get_physics_simulation_library",
          &Simulator::getPhysicsSimulationLibrary,
          R"(Query the physics library implementation currently configured by this Simulator instance.)")
      /* --- Object instancing and access --- */
      .def("add_object", &Simulator::addObject, "object_lib_id"_a,
           "attachment_node"_a = nullptr,
           "light_setup_key"_a = DEFAULT_LIGHTING_KEY, "scene_id"_a = 0,
           R"(
          DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
          to directly access objects instead.
          Instance an object into the scene via a template referenced by
          library id. Optionally attach the object to an existing SceneNode
          and assign its initial LightSetup key.)")
      .def("add_object_by_handle", &Simulator::addObjectByHandle,
           "object_lib_handle"_a, "attachment_node"_a = nullptr,
           "light_setup_key"_a = DEFAULT_LIGHTING_KEY, "scene_id"_a = 0,
           R"(
          DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
          to directly access objects instead.
          Instance an object into the scene via a template referenced by
          its handle. Optionally attach the object to an existing SceneNode
          and assign its initial LightSetup key.)")
      .def("remove_object", &Simulator::removeObject, "object_id"_a,
           "delete_object_node"_a = true, "delete_visual_node"_a = true,
           "scene_id"_a = 0, R"(
        DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
        to directly access objects instead.
        Remove an object instance from the scene. Optionally leave its root
        SceneNode and visual SceneNode on the SceneGraph.

        .. note-warning::

            Removing an object which was attached to the SceneNode
            of an Agent expected to continue producing observations
            will likely result in errors if delete_object_node is true.
            )")
      .def("get_object_initialization_template",
           &Simulator::getObjectInitializationTemplate, "object_id"_a,
           "scene_id"_a = 0,
           R"(
          DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
          to directly access objects instead.
          Get a copy of the ObjectAttributes template used to instance an
          object.)")
      .def(
          "get_stage_initialization_template",
          &Simulator::getStageInitializationTemplate, "scene_id"_a = 0,
          R"(Get a copy of the StageAttributes template used to instance a scene's stage or None if it does not exist.)")
      .def("get_object_motion_type", &Simulator::getObjectMotionType,
           "object_id"_a, "scene_id"_a = 0,
           R"(
          DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
          to directly access objects instead.
          Get the MotionType of an object.)")
      .def("set_object_motion_type", &Simulator::setObjectMotionType,
           "motion_type"_a, "object_id"_a, "scene_id"_a = 0,
           R"(
          DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
          to directly access objects instead.
          Set the MotionType of an object.)")
      .def("get_existing_object_ids", &Simulator::getExistingObjectIDs,
           "scene_id"_a = 0,
           R"(
          DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
          to directly access objects instead.
          Get the list of ids for all objects currently instanced in the
          scene.)")

      /* --- Kinematics and dynamics --- */
      .def(
          "step_world", &Simulator::stepWorld, "dt"_a = 1.0 / 60.0,
          R"(Step the physics simulation by a desired timestep (dt). Note that resulting world time after step may not be exactly t+dt. Use get_world_time to query current simulation time.)")
      .def("get_world_time", &Simulator::getWorldTime,
           R"(Query the current simulation world time.)")
      .def("get_gravity", &Simulator::getGravity, "scene_id"_a = 0,
           R"(Query the gravity vector for a scene.)")
      .def("set_gravity", &Simulator::setGravity, "gravity"_a, "scene_id"_a = 0,
           R"(Set the gravity vector for a scene.)")
      .def("get_object_scene_node", &Simulator::getObjectSceneNode,
           "object_id"_a, "scene_id"_a = 0,
           R"(
          DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
          to directly access objects instead.
          Get a reference to the root SceneNode of an object's SceneGraph
          subtree.)")
      .def("get_object_visual_scene_nodes",
           &Simulator::getObjectVisualSceneNodes, "object_id"_a,
           "scene_id"_a = 0,
           R"(
          DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
          to directly access objects instead.
          Get a list of references to the SceneNodes with an object's
          render assets attached. Use this to manipulate the visual state of
          an object. Changes to these nodes will not affect physics
          simulation.)")
      .def("set_transformation", &Simulator::setTransformation, "transform"_a,
           "object_id"_a, "scene_id"_a = 0,
           R"(
          DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
          to directly access objects instead.
          Set the transformation matrix of an object's root SceneNode and
          update its simulation state.)")
      .def("get_transformation", &Simulator::getTransformation, "object_id"_a,
           "scene_id"_a = 0,
           R"(
          DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
          to directly access objects instead.
          Get the transformation matrix of an object's root SceneNode.)")
      .def("set_rigid_state", &Simulator::setRigidState, "rigid_state"_a,
           "object_id"_a, "scene_id"_a = 0,
           R"(
          DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
          to directly access objects instead.
          Set the transformation of an object from a RigidState and update
          its simulation state.)")
      .def("get_rigid_state", &Simulator::getRigidState, "object_id"_a,
           "scene_id"_a = 0,
           R"(
          DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
          to directly access objects instead.
          Get an object's transformation as a RigidState (i.e. vector,
          quaternion).)")
      .def("set_translation", &Simulator::setTranslation, "translation"_a,
           "object_id"_a, "scene_id"_a = 0,
           R"(
          DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
          to directly access objects instead.
          Set an object's translation and update its simulation state.)")
      .def("get_translation", &Simulator::getTranslation, "object_id"_a,
           "scene_id"_a = 0, R"(
          DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
          to directly access objects instead.
          Get an object's translation.)")
      .def("set_rotation", &Simulator::setRotation, "rotation"_a, "object_id"_a,
           "scene_id"_a = 0,
           R"(
          DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
          to directly access objects instead.
          Set an object's orientation and update its simulation state.)")
      .def("get_rotation", &Simulator::getRotation, "object_id"_a,
           "scene_id"_a = 0, R"(
          DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
          to directly access objects instead.
          Get an object's orientation.)")
      .def("get_object_velocity_control", &Simulator::getObjectVelocityControl,
           "object_id"_a, "scene_id"_a = 0, R"(
          DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
          to directly access objects instead.
          Get a reference to an object's VelocityControl struct. Use
          this to set constant control velocities for MotionType::KINEMATIC
          and MotionType::DYNAMIC objects.)")
      .def("set_linear_velocity", &Simulator::setLinearVelocity, "linVel"_a,
           "object_id"_a, "scene_id"_a = 0,
           R"(
          DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
          to directly access objects instead.
          Set the linear component of an object's velocity. Only applies
          to MotionType::DYNAMIC objects.)")
      .def("get_linear_velocity", &Simulator::getLinearVelocity, "object_id"_a,
           "scene_id"_a = 0, R"(
          DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
          to directly access objects instead.
          Get the linear component of an
          object's velocity. Only non-zero for MotionType::DYNAMIC
          objects.)")
      .def("set_angular_velocity", &Simulator::setAngularVelocity, "angVel"_a,
           "object_id"_a, "scene_id"_a = 0,
           R"(
          DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
          to directly access objects instead.
          Set the angular component of an object's velocity. Only applies
          to MotionType::DYNAMIC objects.)")
      .def("get_angular_velocity", &Simulator::getAngularVelocity,
           "object_id"_a, "scene_id"_a = 0, R"(
          DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
          to directly access objects instead.
          Get the angular component of an
          object's velocity. Only non-zero for MotionType::DYNAMIC
          objects.)")
      .def("apply_force", &Simulator::applyForce, "force"_a,
           "relative_position"_a, "object_id"_a, "scene_id"_a = 0,
           R"(
          DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
          to directly access objects instead.
          Apply an external force to an object at a specific point
          relative to the object's center of mass in global coordinates. Only
          applies to MotionType::DYNAMIC objects.)")
      .def("apply_torque", &Simulator::applyTorque, "torque"_a, "object_id"_a,
           "scene_id"_a = 0,
           R"(
          DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
          to directly access objects instead.
          Apply torque to an object. Only applies to MotionType::DYNAMIC
          objects.)")
      .def("get_object_is_collidable", &Simulator::getObjectIsCollidable,
           "object_id"_a, R"(
          DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
          to directly access objects instead.
          Get whether or not an object is collidable.)")
      .def("set_object_is_collidable", &Simulator::setObjectIsCollidable,
           "collidable"_a, "object_id"_a,
           R"(
          DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
          to directly access objects instead.
          Set whether or not an object is collidable.)")
      .def("get_stage_is_collidable", &Simulator::getStageIsCollidable,
           R"(Get whether or not the static stage is collidable.)")
      .def("set_stage_is_collidable", &Simulator::setStageIsCollidable,
           "collidable"_a,
           R"(Set whether or not the static stage is collidable.)")
      .def(
          "contact_test", &Simulator::contactTest, "object_id"_a,
          "scene_id"_a = 0,
          R"(DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Run collision detection and return a binary indicator of penetration between the specified object and any other collision object. Physics must be enabled.)")
      .def(
          "get_physics_num_active_contact_points",
          &Simulator::getPhysicsNumActiveContactPoints,
          R"(The number of contact points that were active during the last step. An object resting on another object will involve several active contact points. Once both objects are asleep, the contact points are inactive. This count is a proxy for complexity/cost of collision-handling in the current scene.)")
      .def(
          "get_physics_contact_points", &Simulator::getPhysicsContactPoints,
          R"(Return a list of ContactPointData objects describing the contacts from the most recent physics substep.)")
      .def(
          "perform_discrete_collision_detection",
          &Simulator::performDiscreteCollisionDetection,
          R"(Perform discrete collision detection for the scene. Physics must be enabled. Warning: may break simulation determinism.)")
      .def(
          "cast_ray", &Simulator::castRay, "ray"_a, "max_distance"_a = 100.0,
          "scene_id"_a = 0,
          R"(Cast a ray into the collidable scene and return hit results. Physics must be enabled. max_distance in units of ray length.)")
      .def("set_object_bb_draw", &Simulator::setObjectBBDraw, "draw_bb"_a,
           "object_id"_a, "scene_id"_a = 0,
           R"(Enable or disable bounding box visualization for an object.)")
      .def("set_object_semantic_id", &Simulator::setObjectSemanticId,
           "semantic_id"_a, "object_id"_a, "scene_id"_a = 0,
           R"(
          DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
          to directly access objects instead.
          Convenience function to set the semanticId for all visual
          SceneNodes belonging to an object.)")
      .def(
          "recompute_navmesh", &Simulator::recomputeNavMesh, "pathfinder"_a,
          "navmesh_settings"_a, "include_static_objects"_a = false,
          R"(Recompute the NavMesh for a given PathFinder instance using configured NavMeshSettings. Optionally include all MotionType::STATIC objects in the navigability constraints.)")
#ifdef ESP_BUILD_WITH_VHACD
      .def(
          "apply_convex_hull_decomposition",
          &Simulator::convexHullDecomposition, "filename"_a,
          "vhacd_params"_a = assets::ResourceManager::VHACDParameters(),
          "render_chd_result"_a = false, "save_chd_to_obj"_a = false,
          R"(Decomposite an object into its constituent convex hulls with specified VHACD parameters.)")
#endif
      .def("add_trajectory_object", &Simulator::addTrajectoryObject,
           "traj_vis_name"_a, "points"_a, "num_segments"_a = 3,
           "radius"_a = .001, "color"_a = Mn::Color4{0.9, 0.1, 0.1, 1.0},
           "smooth"_a = false, "num_interpolations"_a = 10,
           R"(Build a tube visualization around the passed trajectory of points.
              points : (list of 3-tuples of floats) key point locations to use to create trajectory tube.
              num_segments : (Integer) the number of segments around the tube to be used to make the visualization.
              radius : (Float) the radius of the resultant tube.
              color : (4-tuple of float) the color of the trajectory tube.
              smooth : (Bool) whether or not to smooth trajectory using a Catmull-Rom spline interpolating spline.
              num_interpolations : (Integer) the number of interpolation points to find between successive key points.)")
      .def("get_light_setup", &Simulator::getLightSetup,
           "key"_a = DEFAULT_LIGHTING_KEY,
           R"(Get a copy of the LightSetup registered with a specific key.)")
      .def("get_current_light_setup", &Simulator::getCurrentLightSetup,
           R"(Get a copy of the LightSetup used to create the current scene.)")
      .def(
          "set_light_setup", &Simulator::setLightSetup, "light_setup"_a,
          "key"_a = DEFAULT_LIGHTING_KEY,
          R"(Register a LightSetup with a specific key. If a LightSetup is already registered with this key, it will be overridden. All Drawables referencing the key will use the newly registered LightSetup.)")
      .def("set_object_light_setup", &Simulator::setObjectLightSetup,
           "object_id"_a, "light_setup_key"_a, "scene_id"_a = 0,
           R"(
          DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Use rigid object manager
          to directly access objects instead.
          Modify the LightSetup used to the render all components of an
          object by setting the LightSetup key referenced by all Drawables
          attached to the object's visual SceneNodes.)")
      /* --- P2P/Fixed Constraints API --- */
      .def(
          "create_rigid_constraint", &Simulator::createRigidConstraint,
          "settings"_a,
          R"(Create a rigid constraint between two objects or an object and the world from a RigidConstraintsSettings.)")
      .def("update_rigid_constraint", &Simulator::updateRigidConstraint,
           "constraint_id"_a, "settings"_a,
           R"(Update the settings of a rigid constraint.)")
      .def("get_rigid_constraint_settings",
           &Simulator::getRigidConstraintSettings, "constraint_id"_a,
           R"(Get a copy of the settings for an existing rigid constraint.)")
      .def("remove_rigid_constraint", &Simulator::removeRigidConstraint,
           "constraint_id"_a, R"(Remove a rigid constraint by id.)")
      .def("get_debug_line_render", &Simulator::getDebugLineRender,
           pybind11::return_value_policy::reference,
           R"(Get visualization helper for rendering lines.)");
}

}  // namespace sim
}  // namespace esp
