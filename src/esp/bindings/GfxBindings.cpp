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
#include "esp/gfx/Simulator.h"
#include "esp/scene/SemanticScene.h"

namespace py = pybind11;
using py::literals::operator""_a;

namespace {
template <class T>
esp::scene::SceneNode* nodeGetter(T& self) {
  // TODO(mosra) PR#353
  // NOLINTNEXTLINE(clang-diagnostic-undefined-bool-conversion)
  if (!&self.node())
    throw py::value_error{"feature not valid"};
  return &self.node();
};
}  // namespace

namespace esp {
namespace gfx {

void initGfxBindings(py::module& m) {
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

  // ==== Renderer ====
  py::class_<Renderer, Renderer::ptr>(m, "Renderer")
      .def(py::init(&Renderer::create<>))
      .def("draw",
           py::overload_cast<sensor::Sensor&, scene::SceneGraph&>(
               &Renderer::draw),
           R"(Draw given scene using the visual sensor)", "visualSensor"_a,
           "scene"_a)
      .def(
          "draw",
          py::overload_cast<RenderCamera&, scene::SceneGraph&>(&Renderer::draw),
          R"(Draw given scene using the camera)", "camera"_a, "scene"_a)
      .def("bind_render_target", &Renderer::bindRenderTarget);

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

}  // namespace gfx
}  // namespace esp
