// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"

#include <Magnum/Magnum.h>
#include <Magnum/SceneGraph/SceneGraph.h>

#include <Magnum/PythonBindings.h>
#include <Magnum/SceneGraph/PythonBindings.h>

#include <utility>

#include "esp/sensor/CameraSensor.h"
#ifdef ESP_BUILD_WITH_CUDA
#include "esp/sensor/RedwoodNoiseModel.h"
#endif
#include "esp/sensor/Sensor.h"
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
namespace sensor {

void initSensorBindings(py::module& m) {
  // ==== Observation ====
  // NOLINTNEXTLINE(bugprone-unused-raii)
  py::class_<Observation, Observation::ptr>(m, "Observation");

  // TODO fill out other SensorTypes
  // ==== enum SensorType ====
  py::enum_<SensorType>(m, "SensorType")
      .value("NONE", SensorType::None)
      .value("COLOR", SensorType::Color)
      .value("DEPTH", SensorType::Depth)
      .value("SEMANTIC", SensorType::Semantic);

  py::enum_<SensorSubType>(m, "SensorSubType")
      .value("PINHOLE", SensorSubType::Pinhole)
      .value("ORTHOGRAPHIC", SensorSubType::Orthographic);

  // ==== SensorSpec ====
  py::class_<SensorSpec, SensorSpec::ptr>(m, "SensorSpec", py::dynamic_attr())
      .def(py::init(&SensorSpec::create<>))
      .def_readwrite("uuid", &SensorSpec::uuid)
      .def_readwrite("sensor_type", &SensorSpec::sensorType)
      .def_readwrite("sensor_subtype", &SensorSpec::sensorSubType)
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
            py::setattr(py::cast(self), "__noise_model_kwargs", std::move(v));
          })
      .def("__eq__",
           [](const SensorSpec& self, const SensorSpec& other) -> bool {
             return self == other;
           })
      .def("__neq__",
           [](const SensorSpec& self, const SensorSpec& other) -> bool {
             return self != other;
           });

  // ==== Sensor ====
  py::class_<Sensor, Magnum::SceneGraph::PyFeature<Sensor>,
             Magnum::SceneGraph::AbstractFeature3D,
             Magnum::SceneGraph::PyFeatureHolder<Sensor>>(m, "Sensor")
      .def("specification", &Sensor::specification)
      .def("set_transformation_from_spec", &Sensor::setTransformationFromSpec)
      .def("is_visual_sensor", &Sensor::isVisualSensor)
      .def("get_observation", &Sensor::getObservation)
      .def_property_readonly("node", nodeGetter<Sensor>,
                             "Node this object is attached to")
      .def_property_readonly("object", nodeGetter<Sensor>, "Alias to node");

  // ==== VisualSensor ====
  py::class_<VisualSensor, Magnum::SceneGraph::PyFeature<VisualSensor>, Sensor,
             Magnum::SceneGraph::PyFeatureHolder<VisualSensor>>(m,
                                                                "VisualSensor")
      .def_property_readonly("framebuffer_size", &VisualSensor::framebufferSize)
      .def_property_readonly("render_target", &VisualSensor::renderTarget);

  // === CameraSensor ====
  py::class_<CameraSensor, Magnum::SceneGraph::PyFeature<CameraSensor>,
             VisualSensor, Magnum::SceneGraph::PyFeatureHolder<CameraSensor>>(
      m, "CameraSensor")
      .def(py::init_alias<std::reference_wrapper<scene::SceneNode>,
                          const SensorSpec::ptr&>())
      .def("set_projection_params", &CameraSensor::setProjectionParameters,
           R"(Specify the projection parameters this CameraSensor should use.
           Should be consumed by first querying this CameraSensor's SensorSpec
           and then modifying as necessary.)",
           "sensor_spec"_a)
      .def("zoom", &CameraSensor::modZoom,
           R"(Modify Orthographic Zoom or Perspective FOV multiplicatively by
          passed amount. User >1 to increase, 0<factor<1 to decrease.)",
           "factor"_a)
      .def("reset_zoom", &CameraSensor::resetZoom,
           R"(Reset Orthographic Zoom or Perspective FOV to values
          specified in current sensor spec for this CameraSensor.)")
      .def_property(
          "fov",
          static_cast<Mn::Deg (CameraSensor::*)() const>(&CameraSensor::getFOV),
          static_cast<void (CameraSensor::*)(Mn::Deg)>(&CameraSensor::setFOV),
          R"(Set the field of view to use for this CameraSensor.  Only applicable to
          Pinhole Camera Types)")
      .def_property(
          "camera_type", &CameraSensor::getCameraType,
          &CameraSensor::setCameraType,
          R"(The type of projection (ORTHOGRAPHIC or PINHOLE) this CameraSensor uses.)")
      .def_property("width", &CameraSensor::getWidth, &CameraSensor::setWidth,
                    R"(The width of the viewport for this CameraSensor.)")
      .def_property("height", &CameraSensor::getHeight,
                    &CameraSensor::setHeight,
                    R"(The height of the viewport for this CameraSensor.)")
      .def_property(
          "near_plane_dist", &CameraSensor::getNear, &CameraSensor::setNear,
          R"(The distance to the near clipping plane for this CameraSensor uses.)")
      .def_property(
          "far_plane_dist", &CameraSensor::getFar, &CameraSensor::setFar,
          R"(The distance to the far clipping plane for this CameraSensor uses.)");

  // ==== SensorSuite ====
  py::class_<SensorSuite, SensorSuite::ptr>(m, "SensorSuite")
      .def(py::init(&SensorSuite::create<>))
      .def("add", &SensorSuite::add)
      .def("get", &SensorSuite::get, R"(get the sensor by id)");

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
}

}  // namespace sensor
}  // namespace esp
