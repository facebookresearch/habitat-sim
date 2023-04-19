// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/Bindings.h"

#include <Magnum/ImageView.h>
#include <Magnum/Magnum.h>
#include <Magnum/SceneGraph/SceneGraph.h>

#include <Magnum/PythonBindings.h>
#include <Magnum/SceneGraph/PythonBindings.h>

#include "esp/assets/ResourceManager.h"
#include "esp/bindings/EnumOperators.h"
#include "esp/gfx/DebugLineRender.h"
#include "esp/gfx/LightSetup.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/gfx/RenderTarget.h"
#include "esp/gfx/Renderer.h"
#include "esp/scene/SemanticScene.h"
#include "esp/sensor/CameraSensor.h"
#include "esp/sim/Simulator.h"

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
             Magnum::SceneGraph::PyFeatureHolder<RenderCamera>>
      render_camera(
          m, "Camera",
          R"(RenderCamera: The object of this class is a camera attached
      to the scene node for rendering.)");

  py::enum_<RenderCamera::Flag> flags{render_camera, "Flags", "Flags"};

  flags.value("FRUSTUM_CULLING", RenderCamera::Flag::FrustumCulling)
      .value("OBJECTS_ONLY", RenderCamera::Flag::ObjectsOnly)
      .value("NONE", RenderCamera::Flag{});
  pybindEnumOperators(flags);

  render_camera
      .def(py::init_alias<std::reference_wrapper<scene::SceneNode>,
                          const vec3f&, const vec3f&, const vec3f&>())
      .def(
          "set_projection_matrix",
          [](RenderCamera& self, int w, int h, float n, float f, Mn::Degd fov) {
            self.setProjectionMatrix(w, h, n, f, Mn::Deg(fov));
          },
          R"(Set this `Camera`'s projection matrix.)", "width"_a, "height"_a,
          "znear"_a, "zfar"_a, "hfov"_a)
      .def("set_orthographic_projection_matrix",
           &RenderCamera::setOrthoProjectionMatrix,
           R"(Set this `Orthographic Camera`'s projection matrix.)", "width"_a,
           "height"_a, "znear"_a, "zfar"_a, "scale"_a)
      .def(
          "unproject", &RenderCamera::unproject,
          R"(Unproject a 2D viewport point to a 3D ray with its origin at the camera position. Ray direction is optionally normalized. Non-normalized rays originate at the camera location and terminate at a view plane one unit down the Z axis.)",
          "viewport_point"_a, "normalized"_a = true)
      .def_property_readonly("node", nodeGetter<RenderCamera>,
                             "Node this object is attached to")
      .def_property_readonly("object", nodeGetter<RenderCamera>,
                             "Alias to node");

  // ==== Renderer ====
  py::class_<Renderer, Renderer::ptr> renderer(m, "Renderer");

  py::enum_<Renderer::Flag> rendererFlags{renderer, "Flags", "Flags"};

  rendererFlags.value("VISUALIZE_TEXTURE", Renderer::Flag::VisualizeTexture)
      .value("NONE", Renderer::Flag{});
  pybindEnumOperators(rendererFlags);

  renderer.def(py::init(&Renderer::create<>))
      .def(
          "draw",
          [](Renderer& self, RenderCamera& camera,
             scene::SceneGraph& sceneGraph, RenderCamera::Flag flags) {
            self.draw(camera, sceneGraph, RenderCamera::Flags{flags});
          },
          R"(Draw given scene using the camera)", "camera"_a, "scene"_a,
          "flags"_a = RenderCamera::Flag{RenderCamera::Flag::FrustumCulling})
      .def(
          "draw",
          [](Renderer& self, sensor::VisualSensor& visualSensor,
             sim::Simulator& sim) { self.draw(visualSensor, sim); },
          R"(Draw the active scene in current simulator using the visual sensor)",
          "visualSensor"_a, "sim"_a)
#ifdef ESP_BUILD_WITH_BACKGROUND_RENDERER
      .def(
          "enqueue_async_draw_job",
          [](Renderer& self, sensor::VisualSensor& visualSensor,
             scene::SceneGraph& sceneGraph, const Mn::MutableImageView2D& view,
             RenderCamera::Flag flags) {
            self.enqueueAsyncDrawJob(visualSensor, sceneGraph, view,
                                     RenderCamera::Flags{flags});
          },
          R"(Draw given scene using the visual sensor. See tutorials/async_rendering.py)",
          "visualSensor"_a, "scene"_a, "view"_a,
          "flags"_a = RenderCamera::Flag{RenderCamera::Flag::FrustumCulling})
      .def("wait_draw_jobs", &Renderer::waitDrawJobs,
           R"(See tutorials/async_rendering.py)")
      .def("start_draw_jobs", &Renderer::startDrawJobs,
           R"(See tutorials/async_rendering.py)")
#endif
      .def(
          "acquire_gl_context", &Renderer::acquireGlContext,
          R"(See tutorials/async_rendering.py. This is a noop if the main-thread already has the context.)")
      .def(
          "bind_render_target",
          [](Renderer& self, sensor::VisualSensor& visualSensor,
             Renderer::Flag flags) {
            self.bindRenderTarget(visualSensor, Renderer::Flags{flags});
          },
          R"(Binds a RenderTarget to the sensor)", "visualSensor"_a,
          "flags"_a = Renderer::Flag{});

  py::class_<RenderTarget>(m, "RenderTarget")
      .def("__enter__",
           [](RenderTarget& self) {
             self.renderEnter();
             return &self;
           })
      .def("__exit__",
           [](RenderTarget& self, const py::object&, const py::object&,
              const py::object&) { self.renderExit(); })
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

  py::enum_<LightPositionModel>(
      m, "LightPositionModel",
      R"(Defines the coordinate frame of a light source.)")
      .value("Camera", LightPositionModel::Camera)
      .value("Global", LightPositionModel::Global)
      .value("Object", LightPositionModel::Object);

  py::enum_<LightType>(
      m, "LightType", R"(Defines the type of light described by the LightInfo)")
      .value("Point", LightType::Point)
      .value("Directional", LightType::Directional);

  py::class_<LightInfo>(
      m, "LightInfo",
      R"(Defines the vector, color and LightPositionModel of a single light source.
      For vector, use a Vector3 position and w == 1 to specify a point light with distance attenuation.
      Or, use a Vector3 direction and w == 0 to specify a directional light with no distance attenuation.)")
      .def(py::init())
      .def(py::init<Magnum::Vector4, Magnum::Color3, LightPositionModel>(),
           "vector"_a, "color"_a = Magnum::Color3{1},
           "model"_a = LightPositionModel::Global)
      .def_readwrite("vector", &LightInfo::vector)
      .def_readwrite("color", &LightInfo::color)
      .def_readwrite("model", &LightInfo::model)
      .def(py::self == py::self)
      .def(py::self != py::self);

  py::class_<DebugLineRender, std::shared_ptr<DebugLineRender>>(
      m, "DebugLineRender")
      .def(
          "set_line_width", &DebugLineRender::setLineWidth,
          R"(Set global line width for all lines rendered by DebugLineRender.)")
      .def(
          "push_transform", &DebugLineRender::pushTransform,
          R"(Push (multiply) a transform onto the transform stack, affecting all line-drawing until popped. Must be paired with popTransform().)")
      .def("pop_transform", &DebugLineRender::popTransform,
           R"(See push_transform.)")
      .def("draw_box", &DebugLineRender::drawBox,
           R"(Draw a box in world-space or local-space (see pushTransform).)")
      .def(
          "draw_circle", &DebugLineRender::drawCircle, "translation"_a,
          "radius"_a, "color"_a, "num_segments"_a = 24,
          "normal"_a = Magnum::Vector3{0.0, 1.0, 0.0},
          R"(Draw a circle in world-space or local-space (see pushTransform). The circle is an approximation; see numSegments.)")
      .def(
          "draw_transformed_line",
          py::overload_cast<const Magnum::Vector3&, const Magnum::Vector3&,
                            const Magnum::Color4&, const Magnum::Color4&>(
              &DebugLineRender::drawTransformedLine),
          "from"_a, "to"_a, "from_color"_a, "to_color"_a,
          R"(Draw a line segment in world-space or local-space (see pushTransform) with interpolated color.)")
      .def(
          "draw_transformed_line",
          py::overload_cast<const Magnum::Vector3&, const Magnum::Vector3&,
                            const Magnum::Color4&>(
              &DebugLineRender::drawTransformedLine),
          "from"_a, "to"_a, "color"_a,
          R"(Draw a line segment in world-space or local-space (see pushTransform).)")
      .def(
          "draw_path_with_endpoint_circles",
          py::overload_cast<const std::vector<Magnum::Vector3>&, float,
                            const Magnum::Color4&, int, const Magnum::Vector3&>(
              &DebugLineRender::drawPathWithEndpointCircles),
          "points"_a, "radius"_a, "color"_a, "num_segments"_a = 24,
          "normal"_a = Magnum::Vector3{0.0, 1.0, 0.0},
          R"(Draw a sequence of line segments with circles at the two endpoints. In world-space or local-space (see pushTransform).)");

  m.attr("DEFAULT_LIGHTING_KEY") = DEFAULT_LIGHTING_KEY;
  m.attr("NO_LIGHT_KEY") = NO_LIGHT_KEY;
}

}  // namespace gfx
}  // namespace esp
