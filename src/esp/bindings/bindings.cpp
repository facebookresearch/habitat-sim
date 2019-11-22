// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/OpaqueTypes.h"

namespace py = pybind11;
using namespace py::literals;

#include "esp/core/Configuration.h"
#include "esp/geo/OBB.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/gfx/Renderer.h"
#include "esp/gfx/Simulator.h"
#include "esp/nav/PathFinder.h"
#include "esp/physics/PhysicsManager.h"
#include "esp/scene/Mp3dSemanticScene.h"
#include "esp/scene/ObjectControls.h"
#include "esp/scene/SceneGraph.h"
#include "esp/scene/SceneNode.h"
#include "esp/scene/SemanticScene.h"
#include "esp/scene/SuncgSemanticScene.h"
#include "esp/sensor/PinholeCamera.h"
#include "esp/sensor/Sensor.h"

#ifdef ESP_BUILD_WITH_CUDA
#include "esp/sensor/RedwoodNoiseModel.h"
#endif

#include <Magnum/ImageView.h>
#include <Magnum/Python.h>
#include <Magnum/SceneGraph/Python.h>

using namespace esp;
using namespace esp::core;
using namespace esp::geo;
using namespace esp::gfx;
using namespace esp::nav;
using namespace esp::scene;
using namespace esp::sensor;
using namespace esp::physics;

void initShortestPathBindings(py::module& m);
void initGeoBindings(py::module& m);

namespace {
template <class T>
SceneNode* nodeGetter(T& self) {
  // TODO(mosra) PR#353
  // NOLINTNEXTLINE(clang-diagnostic-undefined-bool-conversion)
  if (!&self.node())
    throw py::value_error{"feature not valid"};
  return &self.node();
};
}  // namespace

PYBIND11_MODULE(habitat_sim_bindings, m) {
  m.attr("cuda_enabled") =
#ifdef ESP_BUILD_WITH_CUDA
      true;
#else
      false;
#endif

  initGeoBindings(m);

  py::bind_map<std::map<std::string, std::string>>(m, "MapStringString");

  m.import("magnum.scenegraph");

  py::class_<Configuration, Configuration::ptr>(m, "Configuration")
      .def(py::init(&Configuration::create<>))
      .def("getBool", &Configuration::getBool)
      .def("getString", &Configuration::getString)
      .def("getInt", &Configuration::getInt)
      .def("getFloat", &Configuration::getFloat)
      .def("get", &Configuration::getString)
      .def("set", &Configuration::set<std::string>)
      .def("set", &Configuration::set<int>)
      .def("set", &Configuration::set<float>)
      .def("set", &Configuration::set<bool>);

  // !!Warning!!
  // CANNOT apply smart pointers to "SceneNode" or ANY its descendant classes,
  // namely, any class whose instance can be a node in the scene graph. Reason:
  // Memory will be automatically handled in simulator (C++ backend). Using
  // smart pointers on scene graph node from Python code, will claim the
  // ownership and eventually free its resources, which leads to "duplicated
  // deallocation", and thus memory corruption.

  // ==== enum SceneNodeType ====
  py::enum_<SceneNodeType>(m, "SceneNodeType")
      .value("EMPTY", SceneNodeType::EMPTY)
      .value("SENSOR", SceneNodeType::SENSOR)
      .value("AGENT", SceneNodeType::AGENT)
      .value("CAMERA", SceneNodeType::CAMERA);

  // ==== SceneNode ====
  py::class_<scene::SceneNode, Magnum::SceneGraph::PyObject<scene::SceneNode>,
             MagnumObject,
             Magnum::SceneGraph::PyObjectHolder<scene::SceneNode>>(
      m, "SceneNode", R"(
      SceneNode: a node in the scene graph.

      Cannot apply a smart pointer to a SceneNode object.
      You can "create it and forget it".
      Simulator backend will handle the memory.)")
      .def(py::init_alias<std::reference_wrapper<scene::SceneNode>>(),
           R"(Constructor: creates a scene node, and sets its parent.)")
      .def_property("type", &SceneNode::getType, &SceneNode::setType)
      .def(
          "create_child", [](SceneNode& self) { return &self.createChild(); },
          R"(Creates a child node, and sets its parent to the current node.)")
      .def_property_readonly("absolute_translation",
                             &SceneNode::absoluteTranslation);

  // ==== RenderCamera ====
  py::class_<RenderCamera, Magnum::SceneGraph::PyFeature<RenderCamera>,
             Magnum::SceneGraph::AbstractFeature3D,
             Magnum::SceneGraph::PyFeatureHolder<RenderCamera>>(
      m, "Camera",
      R"(RenderCamera: The object of this class is a camera attached
      to the scene node for rendering.)")
      .def(py::init_alias<std::reference_wrapper<scene::SceneNode>,
                          const vec3f&, const vec3f&, const vec3f&>())
      .def("setProjectionMatrix", &RenderCamera::setProjectionMatrix, R"(
        Set this `Camera`'s projection matrix.
      )",
           "width"_a, "height"_a, "znear"_a, "zfar"_a, "hfov"_a)
      .def("getProjectionMatrix", &RenderCamera::getProjectionMatrix, R"(
        Get this `Camera`'s projection matrix.
      )")
      .def("getCameraMatrix", &RenderCamera::getCameraMatrix, R"(
        Get this `Camera`'s camera matrix.
      )")
      .def_property_readonly("node", nodeGetter<RenderCamera>,
                             "Node this object is attached to")
      .def_property_readonly("object", nodeGetter<RenderCamera>,
                             "Alias to node");

  // Renderer::draw() and SceneGraph::setDefaultRenderCamera needs the Sensor
  // definition
  py::class_<Sensor, Magnum::SceneGraph::PyFeature<Sensor>,
             Magnum::SceneGraph::AbstractFeature3D,
             Magnum::SceneGraph::PyFeatureHolder<Sensor>>
      sensor(m, "Sensor");

  // ==== SceneGraph ====
  py::class_<scene::SceneGraph>(m, "SceneGraph")
      .def(py::init())
      .def("get_root_node",
           py::overload_cast<>(&scene::SceneGraph::getRootNode, py::const_),
           R"(
            Get the root node of the scene graph.

            User can specify transformation of the root node w.r.t. the world
            frame. (const function) PYTHON DOES NOT GET OWNERSHIP)",
           pybind11::return_value_policy::reference)
      .def("get_root_node",
           py::overload_cast<>(&scene::SceneGraph::getRootNode),
           R"(
            Get the root node of the scene graph.

            User can specify transformation of the root node w.r.t. the world
            frame. PYTHON DOES NOT GET OWNERSHIP)",
           pybind11::return_value_policy::reference)
      .def("set_default_render_camera_parameters",
           &scene::SceneGraph::setDefaultRenderCamera,
           R"(
            Set transformation and the projection matrix to the default render camera.

            The camera will have the same absolute transformation as the target
            scene node after the operation.)",
           "targetSceneNode"_a)
      .def("get_default_render_camera",
           &scene::SceneGraph::getDefaultRenderCamera,
           R"(
            Get the default camera stored in scene graph for rendering.

            PYTHON DOES NOT GET OWNERSHIP)",
           pybind11::return_value_policy::reference);

  // ==== SceneManager ====
  py::class_<scene::SceneManager>(m, "SceneManager")
      .def("init_scene_graph", &scene::SceneManager::initSceneGraph,
           R"(
          Initialize a new scene graph, and return its ID.)")
      .def("get_scene_graph",
           py::overload_cast<int>(&scene::SceneManager::getSceneGraph),
           R"(
             Get the scene graph by scene graph ID.

             PYTHON DOES NOT GET OWNERSHIP)",
           "sceneGraphID"_a, pybind11::return_value_policy::reference)
      .def("get_scene_graph",
           py::overload_cast<int>(&scene::SceneManager::getSceneGraph,
                                  py::const_),
           R"(
             Get the scene graph by scene graph ID.

             PYTHON DOES NOT GET OWNERSHIP)",
           "sceneGraphID"_a, pybind11::return_value_policy::reference);

  // ==== box3f ====
  py::class_<box3f>(m, "BBox")
      .def_property_readonly("sizes", &box3f::sizes)
      .def_property_readonly("center", &box3f::center);

  // ==== OBB ====
  py::class_<OBB>(m, "OBB")
      .def_property_readonly("center", &OBB::center)
      .def_property_readonly("sizes", &OBB::sizes)
      .def_property_readonly("half_extents", &OBB::halfExtents)
      .def_property_readonly(
          "rotation", [](const OBB& self) { return self.rotation().coeffs(); });

  // ==== SemanticCategory ====
  py::class_<SemanticCategory, SemanticCategory::ptr>(m, "SemanticCategory")
      .def("index", &SemanticCategory::index, "mapping"_a = "")
      .def("name", &SemanticCategory::name, "mapping"_a = "");

  // === Mp3dObjectCategory ===
  py::class_<Mp3dObjectCategory, SemanticCategory, Mp3dObjectCategory::ptr>(
      m, "Mp3dObjectCategory")
      .def("index", &Mp3dObjectCategory::index, "mapping"_a = "")
      .def("name", &Mp3dObjectCategory::name, "mapping"_a = "");

  // === Mp3dRegionCategory ===
  py::class_<Mp3dRegionCategory, SemanticCategory, Mp3dRegionCategory::ptr>(
      m, "Mp3dRegionCategory")
      .def("index", &Mp3dRegionCategory::index, "mapping"_a = "")
      .def("name", &Mp3dRegionCategory::name, "mapping"_a = "");

  // === SuncgObjectCategory ===
  py::class_<SuncgObjectCategory, SemanticCategory, SuncgObjectCategory::ptr>(
      m, "SuncgObjectCategory")
      .def("index", &SuncgObjectCategory::index, "mapping"_a = "")
      .def("name", &SuncgObjectCategory::name, "mapping"_a = "");

  // === SuncgRegionCategory ===
  py::class_<SuncgRegionCategory, SemanticCategory, SuncgRegionCategory::ptr>(
      m, "SuncgRegionCategory")
      .def("index", &SuncgRegionCategory::index, "mapping"_a = "")
      .def("name", &SuncgRegionCategory::name, "mapping"_a = "");

  // These two are (cyclically) referenced by multiple classes below, define
  // the classes first so pybind has the type definition available when binding
  // functions
  py::class_<SemanticObject, SemanticObject::ptr> semanticObject(
      m, "SemanticObject");
  py::class_<SemanticRegion, SemanticRegion::ptr> semanticRegion(
      m, "SemanticRegion");

  // ==== SemanticLevel ====
  py::class_<SemanticLevel, SemanticLevel::ptr>(m, "SemanticLevel")
      .def_property_readonly("id", &SemanticLevel::id)
      .def_property_readonly("aabb", &SemanticLevel::aabb)
      .def_property_readonly("regions", &SemanticLevel::regions)
      .def_property_readonly("objects", &SemanticLevel::objects);

  // ==== SemanticRegion ====
  semanticRegion
      .def_property_readonly(
          "id", &SemanticRegion::id,
          "The ID of the region, of the form ``<level_id>_<region_id>``")
      .def_property_readonly("level", &SemanticRegion::level)
      .def_property_readonly("aabb", &SemanticRegion::aabb)
      .def_property_readonly("category", &SemanticRegion::category,
                             "The semantic category of the region")
      .def_property_readonly("objects", &SemanticRegion::objects,
                             "All objects in the region");

  // ==== SuncgSemanticRegion ====
  py::class_<SuncgSemanticRegion, SemanticRegion, SuncgSemanticRegion::ptr>(
      m, "SuncgSemanticRegion")
      .def_property_readonly("id", &SuncgSemanticRegion::id)
      .def_property_readonly("level", &SuncgSemanticRegion::level)
      .def_property_readonly("aabb", &SuncgSemanticRegion::aabb)
      .def_property_readonly("category", &SuncgSemanticRegion::category)
      .def_property_readonly("objects", &SuncgSemanticRegion::objects);

  // ==== SemanticObject ====
  semanticObject
      .def_property_readonly("id", &SemanticObject::id,
                             "The ID of the object, of the form "
                             "``<level_id>_<region_id>_<object_id>``")
      .def_property_readonly("region", &SemanticObject::region)
      .def_property_readonly("aabb", &SemanticObject::aabb)
      .def_property_readonly("obb", &SemanticObject::obb)
      .def_property_readonly("category", &SemanticObject::category);

  // ==== SuncgSemanticObject ====
  py::class_<SuncgSemanticObject, SemanticObject, SuncgSemanticObject::ptr>(
      m, "SuncgSemanticObject")
      .def_property_readonly("id", &SuncgSemanticObject::id)
      .def_property_readonly("region", &SuncgSemanticObject::region)
      .def_property_readonly("aabb", &SuncgSemanticObject::aabb)
      .def_property_readonly("obb", &SuncgSemanticObject::obb)
      .def_property_readonly("category", &SuncgSemanticObject::category,
                             "The semantic category of the object.");

  // ==== SemanticScene ====
  py::class_<SemanticScene, SemanticScene::ptr>(m, "SemanticScene")
      .def(py::init(&SemanticScene::create<>))
      .def_static(
          "load_mp3d_house",
          [](const std::string& filename, SemanticScene& scene,
             const vec4f& rotation) {
            // numpy doesn't have a quaternion equivalent, use vec4
            // instead
            return SemanticScene::loadMp3dHouse(
                filename, scene, Eigen::Map<const quatf>(rotation.data()));
          },
          R"(
        Loads a SemanticScene from a Matterport3D House format file into passed
        `SemanticScene`.
      )",
          "file"_a, "scene"_a, "rotation"_a)
      .def_property_readonly("aabb", &SemanticScene::aabb)
      .def_property_readonly("categories", &SemanticScene::categories,
                             "All semantic categories in the house")
      .def_property_readonly("levels", &SemanticScene::levels,
                             "All levels in the house")
      .def_property_readonly("regions", &SemanticScene::regions,
                             "All regions in the house")
      .def_property_readonly("objects", &SemanticScene::objects,
                             "All object in the house")
      .def_property_readonly("semantic_index_map",
                             &SemanticScene::getSemanticIndexMap)
      .def("semantic_index_to_object_index",
           &SemanticScene::semanticIndexToObjectIndex);

  // ==== ObjectControls ====
  py::class_<ObjectControls, ObjectControls::ptr>(m, "ObjectControls")
      .def(py::init(&ObjectControls::create<>))
      .def("action", &ObjectControls::action, R"(
        Take action using this :py:class:`ObjectControls`.
      )",
           "object"_a, "name"_a, "amount"_a, "apply_filter"_a = true);

  // ==== Renderer ====
  py::class_<Renderer, Renderer::ptr>(m, "Renderer")
      .def(py::init(&Renderer::create<>))
      .def("draw",
           py::overload_cast<sensor::Sensor&, scene::SceneGraph&>(
               &Renderer::draw),
           R"(Draw given scene using the visual sensor)", "visualSensor"_a,
           "scene"_a)
      .def("draw",
           py::overload_cast<gfx::RenderCamera&, scene::SceneGraph&>(
               &Renderer::draw),
           R"(Draw given scene using the camera)", "camera"_a, "scene"_a)
      .def("bind_render_target", &Renderer::bindRenderTarget);

  // TODO fill out other SensorTypes
  // ==== enum SensorType ====
  py::enum_<SensorType>(m, "SensorType")
      .value("NONE", SensorType::NONE)
      .value("COLOR", SensorType::COLOR)
      .value("DEPTH", SensorType::DEPTH)
      .value("SEMANTIC", SensorType::SEMANTIC);

  py::class_<SensorSpec, SensorSpec::ptr>(m, "SensorSpec", py::dynamic_attr())
      .def(py::init(&SensorSpec::create<>))
      .def_readwrite("uuid", &SensorSpec::uuid)
      .def_readwrite("sensor_type", &SensorSpec::sensorType)
      .def_readwrite("sensor_subtype", &SensorSpec::sensorSubtype)
      .def_readwrite("parameters", &SensorSpec::parameters)
      .def_readwrite("position", &SensorSpec::position)
      .def_readwrite("orientation", &SensorSpec::orientation)
      .def_readwrite("resolution", &SensorSpec::resolution)
      .def_readwrite("channels", &SensorSpec::channels)
      .def_readwrite("encoding", &SensorSpec::encoding)
      .def_readwrite("gpu2gpu_transfer", &SensorSpec::gpu2gpuTransfer)
      .def_readwrite("observation_space", &SensorSpec::observationSpace)
      .def_readwrite("noise_model", &SensorSpec::noiseModel)
      .def_property(
          "noise_model_kwargs",
          [](SensorSpec& self) -> py::dict {
            py::handle handle = py::cast(self);
            if (!py::hasattr(handle, "__noise_model_kwargs")) {
              py::setattr(handle, "__noise_model_kwargs", py::dict());
            }
            return py::getattr(handle, "__noise_model_kwargs");
          },
          [](SensorSpec& self, py::dict v) {
            py::setattr(py::cast(self), "__noise_model_kwargs", v);
          })
      .def("__eq__",
           [](const SensorSpec& self, const SensorSpec& other) -> bool {
             return self == other;
           })
      .def("__neq__",
           [](const SensorSpec& self, const SensorSpec& other) -> bool {
             return self != other;
           });

  // ==== Observation ====
  py::class_<Observation, Observation::ptr>(m, "Observation");

  py::class_<RenderTarget>(m, "RenderTarget")
      .def("__enter__",
           [](RenderTarget& self) {
             self.renderEnter();
             return &self;
           })
      .def("__exit__",
           [](RenderTarget& self, py::object exc_type, py::object exc_value,
              py::object traceback) { self.renderExit(); })
      .def("read_frame_rgba", &RenderTarget::readFrameRgba,
           "Reads RGBA frame into passed img in uint8 byte format.")
      .def("read_frame_depth", &RenderTarget::readFrameDepth)
      .def("read_frame_object_id", &RenderTarget::readFrameObjectId)
      .def("blit_rgba_to_default", &RenderTarget::blitRgbaToDefault)
#ifdef ESP_BUILD_WITH_CUDA
      .def("read_frame_rgba_gpu",
           [](RenderTarget& self, size_t devPtr) {
             /*
              * Python has no concept of a pointer, so PyTorch thus exposes the
              pointer to CUDA memory as a simple size_t
              * Thus we need to take in the pointer as a size_t and then
              reinterpret_cast it to the correct type.
              *
              * What PyTorch does internally is similar to
              * ::code
                   uint8_t* tmp = new uint8_t[5];
                   size_t ptr = reinterpret_cast<size_t>(tmp);
              *
              * so reinterpret_cast<uint8_t*> simply undoes the
              reinterpret_cast<size_t>
              */

             self.readFrameRgbaGPU(reinterpret_cast<uint8_t*>(devPtr));
           })
      .def("read_frame_depth_gpu",
           [](RenderTarget& self, size_t devPtr) {
             self.readFrameDepthGPU(reinterpret_cast<float*>(devPtr));
           })
      .def("read_frame_object_id_gpu",
           [](RenderTarget& self, size_t devPtr) {
             self.readFrameObjectIdGPU(reinterpret_cast<int32_t*>(devPtr));
           })
#endif
      .def("render_enter", &RenderTarget::renderEnter)
      .def("render_exit", &RenderTarget::renderExit);

  // Needed by Sensor
  py::class_<Simulator, Simulator::ptr> simulator(m, "Simulator");

  // ==== Sensor ====
  sensor
      .def(py::init_alias<std::reference_wrapper<scene::SceneNode>,
                          const SensorSpec::ptr&>())
      .def("specification", &Sensor::specification)
      .def("set_transformation_from_spec", &Sensor::setTransformationFromSpec)
      .def("is_visual_sensor", &Sensor::isVisualSensor)
      .def("get_observation", &Sensor::getObservation)
      .def_property_readonly("node", nodeGetter<Sensor>,
                             "Node this object is attached to")
      .def_property_readonly("object", nodeGetter<Sensor>, "Alias to node")
      .def_property_readonly("framebuffer_size", &Sensor::framebufferSize)
      .def_property_readonly("render_target", &Sensor::renderTarget);

#ifdef ESP_BUILD_WITH_CUDA
  py::class_<RedwoodNoiseModelGPUImpl, RedwoodNoiseModelGPUImpl::uptr>(
      m, "RedwoodNoiseModelGPUImpl")
      .def(py::init(&RedwoodNoiseModelGPUImpl::create_unique<
                    const Eigen::Ref<const Eigen::RowMatrixXf>&, int, float>))
      .def("simulate_from_cpu", &RedwoodNoiseModelGPUImpl::simulateFromCPU)
      .def("simulate_from_gpu", [](RedwoodNoiseModelGPUImpl& self,
                                   std::size_t devDepth, const int rows,
                                   const int cols, std::size_t devNoisyDepth) {
        self.simulateFromGPU(reinterpret_cast<const float*>(devDepth), rows,
                             cols, reinterpret_cast<float*>(devNoisyDepth));
      });
#endif

  // ==== PinholeCamera (subclass of Sensor) ====
  py::class_<sensor::PinholeCamera,
             Magnum::SceneGraph::PyFeature<sensor::PinholeCamera>,
             sensor::Sensor,
             Magnum::SceneGraph::PyFeatureHolder<PinholeCamera>>(
      m, "PinholeCamera")
      // initialized, attached to pinholeCameraNode, status: "valid"
      .def(py::init_alias<std::reference_wrapper<scene::SceneNode>,
                          const sensor::SensorSpec::ptr&>())
      .def("set_transformation_matrix",
           &sensor::PinholeCamera::setTransformationMatrix,
           R"(Compute and set the transformation matrix to the render camera.)")
      .def("set_projection_matrix", &sensor::PinholeCamera::setProjectionMatrix,
           R"(Set the width, height, near, far, and hfov,
          stored in pinhole camera to the render camera.)")
      .def("set_viewport", &sensor::PinholeCamera::setViewport,
           R"(Set the viewport to the render camera)");

  // ==== SensorSuite ====
  py::class_<SensorSuite, SensorSuite::ptr>(m, "SensorSuite")
      .def(py::init(&SensorSuite::create<>))
      .def("add", &SensorSuite::add)
      .def("get", &SensorSuite::get, R"(get the sensor by id)");

  // ==== SceneConfiguration ====
  py::class_<SceneConfiguration, SceneConfiguration::ptr>(m,
                                                          "SceneConfiguration")
      .def(py::init(&SceneConfiguration::create<>))
      .def_readwrite("dataset", &SceneConfiguration::dataset)
      .def_readwrite("id", &SceneConfiguration::id)
      .def_readwrite("filepaths", &SceneConfiguration::filepaths)
      .def_readwrite("scene_up_dir", &SceneConfiguration::sceneUpDir)
      .def_readwrite("scene_front_dir", &SceneConfiguration::sceneFrontDir)
      .def_readwrite("scene_scale_unit", &SceneConfiguration::sceneScaleUnit)
      .def(
          "__eq__",
          [](const SceneConfiguration& self,
             const SceneConfiguration& other) -> bool { return self == other; })
      .def("__neq__",
           [](const SceneConfiguration& self, const SceneConfiguration& other)
               -> bool { return self != other; });

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
      .def_readwrite("create_renderer", &SimulatorConfiguration::createRenderer)
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

  initShortestPathBindings(m);

  // ==== enum object MotionType ====
  py::enum_<MotionType>(m, "MotionType")
      .value("ERROR_MOTIONTYPE", MotionType::ERROR_MOTIONTYPE)
      .value("STATIC", MotionType::STATIC)
      .value("KINEMATIC", MotionType::KINEMATIC)
      .value("DYNAMIC", MotionType::DYNAMIC);

  // ==== Simulator ====
  simulator.def(py::init(&Simulator::create<const SimulatorConfiguration&>))
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
      /* --- Physics functions --- */
      .def("add_object", &Simulator::addObject, "object_lib_index"_a,
           "scene_id"_a = 0)
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
           "sceneID"_a = 0);
}
