// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/Bindings.h"

#include <Magnum/ImageView.h>
#include <Magnum/Magnum.h>
#include <Magnum/SceneGraph/SceneGraph.h>

#include <Magnum/PythonBindings.h>
#include <Magnum/SceneGraph/PythonBindings.h>

#include "esp/gfx/Renderer.h"
#include "esp/gfx/replay/ReplayManager.h"
#include "esp/metadata/MetadataMediator.h"
#include "esp/physics/objectManagers/ArticulatedObjectManager.h"
#include "esp/physics/objectManagers/RigidObjectManager.h"
#include "esp/scene/SemanticScene.h"
#include "esp/sim/AbstractReplayRenderer.h"
#include "esp/sim/BatchReplayRenderer.h"
#include "esp/sim/ClassicReplayRenderer.h"
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
      .def_readwrite(
          "random_seed", &SimulatorConfiguration::randomSeed,
          R"(The Simulator and Pathfinder random seed. Set during scene initialization.)")
      .def_readwrite(
          "default_agent_id", &SimulatorConfiguration::defaultAgentId,
          R"(The default agent id used during initialization and functionally whenever alternative agent ids are not provided.)")
      .def_readwrite("gpu_device_id", &SimulatorConfiguration::gpuDeviceId,
                     R"(The system GPU device to use for rendering.)")
      .def_readwrite(
          "allow_sliding", &SimulatorConfiguration::allowSliding,
          R"(Whether or not the agent can slide on NavMesh collisions.)")
      .def_readwrite(
          "create_renderer", &SimulatorConfiguration::createRenderer,
          R"(Optimisation for non-visual simulation. If false, no renderer will be created and no materials or textures loaded.)")
      .def_readwrite(
          "leave_context_with_background_renderer",
          &SimulatorConfiguration::leaveContextWithBackgroundRenderer,
          R"(See tutorials/async_rendering.py)")
      .def_readwrite("frustum_culling", &SimulatorConfiguration::frustumCulling,
                     R"(Enable or disable the frustum culling optimisation.)")
      .def_readwrite(
          "enable_physics", &SimulatorConfiguration::enablePhysics,
          R"(Specifies whether or not dynamics is supported by the simulation if a suitable library (i.e. Bullet) has been installed. Install with --bullet to enable.)")
      .def_readwrite(
          "use_semantic_textures",
          &SimulatorConfiguration::useSemanticTexturesIfFound,
          R"(If the loaded scene/dataset supports semantically annotated textures, use these for
      semantic rendering. Defaults to True)")
      .def_readwrite(
          "enable_gfx_replay_save",
          &SimulatorConfiguration::enableGfxReplaySave,
          R"(Enable replay recording. See sim.gfx_replay.save_keyframe.)")
      .def_readwrite("physics_config_file",
                     &SimulatorConfiguration::physicsConfigFile,
                     R"(Path to the physics parameter config file.)")
      .def_readwrite(
          "override_scene_light_defaults",
          &SimulatorConfiguration::overrideSceneLightDefaults,
          R"(Override scene lighting setup to use with value specified by `scene_light_setup`.)")
      .def_readwrite("scene_light_setup",
                     &SimulatorConfiguration::sceneLightSetupKey,
                     R"(Light setup key for the scene.)")
      .def_readwrite("load_semantic_mesh",
                     &SimulatorConfiguration::loadSemanticMesh,
                     R"(Whether or not to load the semantic mesh.)")
      .def_readwrite(
          "force_separate_semantic_scene_graph",
          &SimulatorConfiguration::forceSeparateSemanticSceneGraph,
          R"(Required to support playback of any gfx replay that includes a
          stage with a semantic mesh. Set to false otherwise.)")
      .def_readwrite(
          "requires_textures", &SimulatorConfiguration::requiresTextures,
          R"(Whether or not to load textures for the meshes. This MUST be true for RGB rendering.)")
      .def_readwrite(
          "navmesh_settings", &SimulatorConfiguration::navMeshSettings,
          R"(Optionally provide a pre-configured NavMeshSettings. If provided, the NavMesh will be recomputed with the provided settings if: A. no NavMesh was loaded, or B. the loaded NavMesh's settings differ from the configured settings. If not provided, no NavMesh recompute will be done automatically.)")
      .def_readwrite(
          "enable_hbao", &SimulatorConfiguration::enableHBAO,
          R"(Whether or not to enable horizon-based ambient occlusion, which provides soft shadows in corners and crevices.)")
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
      .def(
          "physics_debug_draw", &Simulator::physicsDebugDraw, "projMat"_a,
          R"(Render any debugging visualizations provided by the underlying physics simulator implementation given the composed projection and transformation matrix for the render camera.)")
      .def_property("pathfinder", &Simulator::getPathFinder,
                    &Simulator::setPathFinder)
      .def_property(
          "navmesh_visualization", &Simulator::isNavMeshVisualizationActive,
          &Simulator::setNavMeshVisualization,
          R"(Enable or disable wireframe visualization of current pathfinder's NavMesh.)")
      .def_property_readonly("gpu_device", &Simulator::gpuDevice)
      .def_property_readonly("random", &Simulator::random)
      .def_property_readonly(
          "curr_scene_name", &Simulator::getCurSceneInstanceName,
          R"(The simplified, but unique, name of the currently loaded scene.)")
      .def_property_readonly(
          "semantic_color_map", &Simulator::getSemanticSceneColormap,
          R"(The list of semantic colors being used for semantic rendering. The index
            in the list corresponds to the semantic ID.)")
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
      .def(
          "get_stage_initialization_template",
          &Simulator::getStageInitializationTemplate,
          R"(Get a copy of the StageAttributes template used to instance a scene's stage or None if it does not exist.)")

      .def("build_semantic_CC_objects", &Simulator::buildSemanticCCObjects,
           R"(Get a dictionary of the current semantic scene's connected
          components keyed by color or id, where each value is a list of Semantic Objects
          corresponding to an individual connected component.")
      .def(
          "build_vertex_color_map_report",
          &Simulator::buildVertexColorMapReport,
          R"(Get a list of strings describing first each color found on vertices in the
          semantic mesh that is not present in the loaded semantic scene descriptor file,
          and then a list of each semantic object whose specified color is not found on
          any vertex in the mesh.)")

      /* --- Kinematics and dynamics --- */
      .def(
          "step_world", &Simulator::stepWorld, "dt"_a = 1.0 / 60.0,
          R"(Step the physics simulation by a desired timestep (dt). Note that resulting world time after step may not be exactly t+dt. Use get_world_time to query current simulation time.)")
      .def("get_world_time", &Simulator::getWorldTime,
           R"(Query the current simulation world time.)")
      .def("get_physics_time_step", &Simulator::getPhysicsTimeStep,
           R"(Get the last used physics timestep)")
      .def("get_gravity", &Simulator::getGravity,
           R"(Query the gravity vector for the scene.)")
      .def("set_gravity", &Simulator::setGravity, "gravity"_a,
           R"(Set the gravity vector for the scene.)")

      .def("get_stage_is_collidable", &Simulator::getStageIsCollidable,
           R"(Get whether or not the static stage is collidable.)")
      .def("set_stage_is_collidable", &Simulator::setStageIsCollidable,
           "collidable"_a,
           R"(Set whether or not the static stage is collidable.)")
      .def(
          "contact_test", &Simulator::contactTest, "object_id"_a,
          R"(DEPRECATED AND WILL BE REMOVED IN HABITAT-SIM 2.0. Run collision detection and return a binary indicator of penetration between the specified object and any other collision object. Physics must be enabled.)")
      .def(
          "get_physics_num_active_contact_points",
          &Simulator::getPhysicsNumActiveContactPoints,
          R"(The number of contact points that were active during the last step. An object resting on another object will involve several active contact points. Once both objects are asleep, the contact points are inactive. This count is a proxy for complexity/cost of collision-handling in the current scene.)")
      .def(
          "get_physics_num_active_overlapping_pairs",
          &Simulator::getPhysicsNumActiveOverlappingPairs,
          R"(The number of active overlapping pairs during the last step. When object bounding boxes overlap and either object is active, additional "narrowphase" collision-detection must be run. This count is a proxy for complexity/cost of collision-handling in the current scene.)")
      .def(
          "get_physics_step_collision_summary",
          &Simulator::getPhysicsStepCollisionSummary,
          R"(Get a summary of collision-processing from the last physics step.)")
      .def("get_physics_contact_points", &Simulator::getPhysicsContactPoints,
           R"(Return a list of ContactPointData "
          "objects describing the contacts from the most recent physics substep.)")
      .def(
          "perform_discrete_collision_detection",
          &Simulator::performDiscreteCollisionDetection,
          R"(Perform discrete collision detection for the scene. Physics must be enabled. Warning: may break simulation determinism.)")
      .def(
          "cast_ray", &Simulator::castRay, "ray"_a, "max_distance"_a = 100.0,
          R"(Cast a ray into the collidable scene and return hit results. Physics must be enabled. max_distance in units of ray length.)")
      .def("set_object_bb_draw", &Simulator::setObjectBBDraw, "draw_bb"_a,
           "object_id"_a,
           R"(Enable or disable bounding box visualization for an object.)")
      .def(
          "recompute_navmesh", &Simulator::recomputeNavMesh, "pathfinder"_a,
          "navmesh_settings"_a,
          R"(Recompute the NavMesh for a given PathFinder instance using configured NavMeshSettings.)")

      .def(
          "add_trajectory_object",
          [](Simulator& self, const std::string& name,
             const std::vector<Mn::Vector3>& pts, int numSegments, float radius,
             const Magnum::Color4& color, bool smooth, int numInterps) {
            return self.addTrajectoryObject(
                name, pts, {Mn::Color3(color.rgb())}, numSegments, radius,
                smooth, numInterps);
          },
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
      .def("add_gradient_trajectory_object",
           static_cast<int (Simulator::*)(
               const std::string&, const std::vector<Mn::Vector3>&,
               const std::vector<Mn::Color3>&, int, float, bool, int)>(
               &Simulator::addTrajectoryObject),
           "traj_vis_name"_a, "points"_a, "colors"_a, "num_segments"_a = 3,
           "radius"_a = .001, "smooth"_a = false, "num_interpolations"_a = 10,
           R"(Build a tube visualization around the passed trajectory of
          points, using the passed colors to build a gradient along the length of the tube.
              points : (list of 3-tuples of floats) key point locations to
              use to create trajectory tube. num_segments : (Integer) the
              number of segments around the tube to be used to make the
              visualization. radius : (Float) the radius of the resultant
              tube. colors : (List of 3-tuple of byte) the colors to build
              the gradient along the length of the trajectory tube. smooth :
              (Bool) whether or not to smooth trajectory using a Catmull-Rom
              spline interpolating spline. num_interpolations : (Integer) the
              number of interpolation points to find between successive key
              points.)")
      .def(
          "save_current_scene_config",
          static_cast<bool (Simulator::*)(const std::string&) const>(
              &Simulator::saveCurrentSceneInstance),
          R"(Save the current simulation world's state as a Scene Instance Config JSON
          using the passed name. This can be used to reload the stage, objects, articulated
          objects and other values as they currently are.)",
          "file_name"_a)
      .def(
          "save_current_scene_config",
          static_cast<bool (Simulator::*)(bool) const>(
              &Simulator::saveCurrentSceneInstance),
          R"(Save the current simulation world's state as a Scene Instance Config JSON
          using the name of the loaded scene, either overwritten, if overwrite is True, or
          with an incrementer in the file name of the form (copy xxxx) where xxxx is a number.
          This can be used to reload the stage, objects, articulated
          objects and other values as they currently are.)",
          "overwrite"_a = false)
      .def("get_light_setup", &Simulator::getLightSetup,
           "key"_a = DEFAULT_LIGHTING_KEY,
           R"(Get a copy of the LightSetup registered with a specific key.)")
      .def("get_current_light_setup", &Simulator::getCurrentLightSetup,
           R"(Get a copy of the LightSetup used to create the current scene.)")
      .def(
          "set_light_setup", &Simulator::setLightSetup, "light_setup"_a,
          "key"_a = DEFAULT_LIGHTING_KEY,
          R"(Register a LightSetup with a specific key. If a LightSetup is already registered with
          this key, it will be overridden. All Drawables referencing the key will use the newly
          registered LightSetup.)")
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
      .def(
          "get_runtime_perf_stat_names", &Simulator::getRuntimePerfStatNames,
          R"(Runtime perf stats are various scalars helpful for troubleshooting runtime perf. This can be called once at startup. See also get_runtime_perf_stat_values.)")
      .def(
          "get_runtime_perf_stat_values", &Simulator::getRuntimePerfStatValues,
          R"(Runtime perf stats are various scalars helpful for troubleshooting runtime perf. These values generally change after every sim step. See also get_runtime_perf_stat_names.)")
      .def("get_debug_line_render", &Simulator::getDebugLineRender,
           pybind11::return_value_policy::reference,
           R"(Get visualization helper for rendering lines.)");

  // ==== ReplayRendererConfiguration ====
  py::class_<ReplayRendererConfiguration, ReplayRendererConfiguration::ptr>(
      m, "ReplayRendererConfiguration")
      .def(py::init(&ReplayRendererConfiguration::create<>))
      .def_readwrite("num_environments",
                     &ReplayRendererConfiguration::numEnvironments,
                     R"(Number of concurrent environments to render.)")
      .def_readwrite(
          "standalone", &ReplayRendererConfiguration::standalone,
          R"(Determines if the renderer is standalone (windowless) or not (embedded in another window).)")
      .def_readwrite(
          "sensor_specifications",
          &ReplayRendererConfiguration::sensorSpecifications,
          R"(List of sensor specifications for one simulator. For batch rendering, all simulators must have the same specification.)")
      .def_readwrite("gpu_device_id", &ReplayRendererConfiguration::gpuDeviceId,
                     R"(The system GPU device to use for rendering)")
      .def_readwrite("enable_frustum_culling",
                     &ReplayRendererConfiguration::enableFrustumCulling,
                     R"(Controls whether frustum culling is enabled.)")
      .def_readwrite(
          "enable_hbao", &ReplayRendererConfiguration::enableHBAO,
          R"(Controls whether horizon-based ambient occlusion is enabled.)")
      .def_readwrite(
          "force_separate_semantic_scene_graph",
          &ReplayRendererConfiguration::forceSeparateSemanticSceneGraph,
          R"(Required to support playback of any gfx replay that includes a
          stage with a semantic mesh. Set to false otherwise.)")
      .def_readwrite(
          "leave_context_with_background_renderer",
          &ReplayRendererConfiguration::leaveContextWithBackgroundRenderer,
          R"(See See tutorials/async_rendering.py.)");

  // ==== ReplayRenderer ====
  py::class_<AbstractReplayRenderer, AbstractReplayRenderer::ptr>(
      m, "ReplayRenderer")
      .def_static(
          "create_classic_replay_renderer",
          [](const ReplayRendererConfiguration& cfg)
              -> AbstractReplayRenderer::ptr {
            return std::make_shared<ClassicReplayRenderer>(cfg);
          },
          R"(Create a replay renderer using the classic render pipeline.)")
      .def_static(
          "create_batch_replay_renderer",
          [](const ReplayRendererConfiguration& cfg)
              -> AbstractReplayRenderer::ptr {
            return std::make_shared<BatchReplayRenderer>(cfg);
          },
          R"(Create a replay renderer using the batch render pipeline.)")
      .def("close", &AbstractReplayRenderer::close,
           "Releases the graphics context and resources used by the replay "
           "renderer.")
      .def(
          "preload_file",
          [](AbstractReplayRenderer& self, const std::string& filePath) {
            self.preloadFile(filePath);
          },
          R"(Load a composite file that the renderer will use in-place of simulation assets to improve memory usage and performance.)")
      .def_property_readonly("environment_count",
                             &AbstractReplayRenderer::environmentCount,
                             "Get the batch size.")
      .def("sensor_size", &AbstractReplayRenderer::sensorSize,
           "Get the resolution of a sensor.")
      .def("clear_environment", &AbstractReplayRenderer::clearEnvironment,
           "Clear all instances and resets memory of an environment.")
      .def("render",
           static_cast<void (AbstractReplayRenderer::*)(
               Magnum::GL::AbstractFramebuffer&)>(
               &AbstractReplayRenderer::render),
           R"(Render all sensors onto the specified framebuffer.)")
      .def(
          "render",
          [](AbstractReplayRenderer& self,
             std::vector<Mn::MutableImageView2D> colorImageViews,
             std::vector<Mn::MutableImageView2D> depthImageViews) {
            self.render(colorImageViews, depthImageViews);
          },
          R"(Render sensors into the specified image vectors (one per environment).
          Blocks the thread during the GPU-to-CPU memory transfer operation.
          Empty lists can be supplied to skip the copying render targets.
          The images are required to be pre-allocated.)",
          py::arg("color_images") = std::vector<Mn::MutableImageView2D>{},
          py::arg("depth_images") = std::vector<Mn::MutableImageView2D>{})
      .def(
          "set_sensor_transforms_from_keyframe",
          &AbstractReplayRenderer::setSensorTransformsFromKeyframe,
          R"(Set the sensor transforms from a keyframe. Sensors are stored as user data and identified using a prefix in their name.)")
      .def("set_sensor_transform", &AbstractReplayRenderer::setSensorTransform,
           R"(Set the transform of a specific sensor.)")
      .def("set_environment_keyframe",
           &AbstractReplayRenderer::setEnvironmentKeyframe,
           R"(Set the keyframe for a specific environment.)")
      .def_static(
          "environment_grid_size", &AbstractReplayRenderer::environmentGridSize,
          R"(Get the dimensions (tile counts) of the environment grid.)")
      .def(
          "cuda_color_buffer_device_pointer",
          [](AbstractReplayRenderer& self) {
            return py::capsule(self.getCudaColorBufferDevicePointer());
          },
          R"(Retrieve the color buffer as a CUDA device pointer.)")
      .def(
          "cuda_depth_buffer_device_pointer",
          [](AbstractReplayRenderer& self) {
            return py::capsule(self.getCudaColorBufferDevicePointer());
          },
          R"(Retrieve the depth buffer as a CUDA device pointer.)")
      .def("debug_line_render", &AbstractReplayRenderer::getDebugLineRender,
           R"(Get visualization helper for rendering lines.)")
      .def("unproject", &AbstractReplayRenderer::unproject,
           R"(Unproject a screen-space point to a world-space ray.)");
}

}  // namespace sim
}  // namespace esp
