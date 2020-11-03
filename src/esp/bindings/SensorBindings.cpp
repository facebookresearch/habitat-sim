// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"

#include <Magnum/Magnum.h>
#include <Magnum/SceneGraph/SceneGraph.h>

#include <Magnum/PythonBindings.h>
#include <Magnum/SceneGraph/PythonBindings.h>

#include <utility>

#include "esp/sensor/PinholeCamera.h"
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
  py::class_<Observation, Observation::ptr>(m, "Observation");

  // TODO fill out other SensorTypes
  // ==== enum SensorType ====
  py::enum_<SensorType>(m, "SensorType")
      .value("NONE", SensorType::NONE)
      .value("COLOR", SensorType::COLOR)
      .value("DEPTH", SensorType::DEPTH)
      .value("SEMANTIC", SensorType::SEMANTIC);

  // ==== SensorSpec ====
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

  // ==== PinholeCamera (subclass of Sensor) ====
  py::class_<PinholeCamera, Magnum::SceneGraph::PyFeature<PinholeCamera>,
             VisualSensor, Magnum::SceneGraph::PyFeatureHolder<PinholeCamera>>(
      m, "PinholeCamera")
      // initialized, attached to pinholeCameraNode, status: "valid"
      .def(py::init_alias<std::reference_wrapper<scene::SceneNode>,
                          const SensorSpec::ptr&>());

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
