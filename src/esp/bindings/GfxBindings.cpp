// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"

#include <Magnum/ImageView.h>
#include <Magnum/Magnum.h>
#include <Magnum/SceneGraph/SceneGraph.h>

#include <Magnum/Python.h>
#include <Magnum/SceneGraph/Python.h>

#include "esp/assets/ResourceManager.h"
#include "esp/gfx/LightSetup.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/gfx/Renderer.h"
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
             Magnum::SceneGraph::Camera3D,
             Magnum::SceneGraph::PyFeatureHolder<RenderCamera>>(
      m, "Camera",
      R"(RenderCamera: The object of this class is a camera attached
      to the scene node for rendering.)")
      .def(py::init_alias<std::reference_wrapper<scene::SceneNode>,
                          const vec3f&, const vec3f&, const vec3f&>())
      .def("set_projection_matrix", &RenderCamera::setProjectionMatrix, R"(
        Set this `Camera`'s projection matrix.
      )",
           "width"_a, "height"_a, "znear"_a, "zfar"_a, "hfov"_a)
      .def_property_readonly("node", nodeGetter<RenderCamera>,
                             "Node this object is attached to")
      .def_property_readonly("object", nodeGetter<RenderCamera>,
                             "Alias to node");

  // ==== Renderer ====
  py::class_<Renderer, Renderer::ptr>(m, "Renderer")
      .def(py::init(&Renderer::create<>))
      .def("draw",
           py::overload_cast<sensor::VisualSensor&, scene::SceneGraph&, bool>(
               &Renderer::draw),
           R"(Draw given scene using the visual sensor)", "visualSensor"_a,
           "scene"_a, "frustumCulling"_a = true)
      .def("draw",
           py::overload_cast<RenderCamera&, scene::SceneGraph&, bool>(
               &Renderer::draw),
           R"(Draw given scene using the camera)", "camera"_a, "scene"_a,
           "frustumCulling"_a = true)
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

  py::enum_<LightPositionModel>(m, "LightPositionModel")
      .value("CAMERA", LightPositionModel::CAMERA)
      .value("GLOBAL", LightPositionModel::GLOBAL)
      .value("OBJECT", LightPositionModel::OBJECT);

  py::class_<LightInfo>(m, "LightInfo")
      .def(py::init())
      .def(py::init<Magnum::Vector3, Magnum::Color4, LightPositionModel>(),
           "position"_a, "color"_a = Magnum::Color4{1},
           "model"_a = LightPositionModel::GLOBAL)
      .def_readwrite("position", &LightInfo::position)
      .def_readwrite("color", &LightInfo::color)
      .def_readwrite("model", &LightInfo::model)
      .def(py::self == py::self)
      .def(py::self != py::self);

  m.attr("DEFAULT_LIGHTING_KEY") =
      assets::ResourceManager::DEFAULT_LIGHTING_KEY;
  m.attr("NO_LIGHT_KEY") = assets::ResourceManager::NO_LIGHT_KEY;
}

}  // namespace gfx
}  // namespace esp
