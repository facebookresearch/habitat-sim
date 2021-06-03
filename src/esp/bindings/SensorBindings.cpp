// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"

#include <Corrade/Containers/OptionalPythonBindings.h>
#include <Magnum/Magnum.h>
#include <Magnum/SceneGraph/SceneGraph.h>

#include <Magnum/PythonBindings.h>
#include <Magnum/SceneGraph/PythonBindings.h>

#include <utility>

#include "esp/sensor/CameraSensor.h"
#include "esp/sensor/CubeMapSensorBase.h"
#include "esp/sensor/EquirectangularSensor.h"
#include "esp/sensor/FisheyeSensor.h"
#include "esp/sensor/VisualSensor.h"
#ifdef ESP_BUILD_WITH_CUDA
#include "esp/sensor/RedwoodNoiseModel.h"
#endif
#include "esp/sensor/Sensor.h"
#include "esp/sensor/SensorFactory.h"
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
      .value("NONE", SensorSubType::None)
      .value("PINHOLE", SensorSubType::Pinhole)
      .value("ORTHOGRAPHIC", SensorSubType::Orthographic)
      .value("FISHEYE", SensorSubType::Fisheye)
      .value("EQUIRECTANGULAR", SensorSubType::Equirectangular);

  py::enum_<FisheyeSensorModelType>(m, "FisheyeSensorModelType")
      .value("DOUBLE_SPHERE", FisheyeSensorModelType::DoubleSphere);

  // ==== SensorSpec ====
  py::class_<SensorSpec, SensorSpec::ptr>(m, "SensorSpec", py::dynamic_attr())
      .def(py::init(&SensorSpec::create<>))
      .def_readwrite("uuid", &SensorSpec::uuid)
      .def_readwrite("sensor_type", &SensorSpec::sensorType)
      .def_readwrite("sensor_subtype", &SensorSpec::sensorSubType)
      .def_readwrite("position", &SensorSpec::position)
      .def_readwrite("orientation", &SensorSpec::orientation)
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
      .def("is_visual_sensor_spec", &SensorSpec::isVisualSensorSpec)
      .def("__eq__", &SensorSpec::operator==)
      .def("__neq__", &SensorSpec::operator!=);

  // ==== VisualSensorSpec ====
  py::class_<VisualSensorSpec, VisualSensorSpec::ptr, SensorSpec>(
      m, "VisualSensorSpec", py::dynamic_attr())
      .def(py::init(&VisualSensorSpec::create<>))
      .def_readwrite("near", &VisualSensorSpec::near)
      .def_readwrite("far", &VisualSensorSpec::far)
      .def_readwrite("resolution", &VisualSensorSpec::resolution)
      .def_readwrite("gpu2gpu_transfer", &VisualSensorSpec::gpu2gpuTransfer)
      .def_readwrite("channels", &VisualSensorSpec::channels)
      .def_readwrite("clear_color", &CameraSensorSpec::clearColor);

  // ====CameraSensorSpec ====
  py::class_<CameraSensorSpec, CameraSensorSpec::ptr, VisualSensorSpec>(
      m, "CameraSensorSpec", py::dynamic_attr())
      .def(py::init(&CameraSensorSpec::create<>))
      .def_property(
          "hfov", [](CameraSensorSpec& self) { return Mn::Degd(self.hfov); },
          [](CameraSensorSpec& self, const py::object& angle) {
            auto PyDeg = py::module_::import("magnum").attr("Deg");
            self.hfov = Mn::Deg(PyDeg(angle).cast<Mn::Degd>());
          })
      .def_readwrite("ortho_scale", &CameraSensorSpec::orthoScale);

  // === CubemapSensorBaseSpec ===
  // NOLINTNEXTLINE (bugprone-unused-raii)
  py::class_<CubeMapSensorBaseSpec, CubeMapSensorBaseSpec::ptr,
             VisualSensorSpec>(m, "CubeMapSensorBaseSpec");

  // === EquirectangularSensorSpec ===
  py::class_<EquirectangularSensorSpec, EquirectangularSensorSpec::ptr,
             CubeMapSensorBaseSpec>(m, "EquirectangularSensorSpec")
      .def(py::init(&EquirectangularSensorSpec::create<>));

  // ====FisheyeSensorSpec ====
  py::class_<FisheyeSensorSpec, FisheyeSensorSpec::ptr, CubeMapSensorBaseSpec>(
      m, "FisheyeSensorSpec", py::dynamic_attr())
      .def(py::init(&FisheyeSensorSpec::create<>))
      .def_readwrite("focal_length", &FisheyeSensorSpec::focalLength)
      .def_readwrite("principal_point_offset",
                     &FisheyeSensorSpec::principalPointOffset)
      .def_readwrite(
          "cubemap_size", &FisheyeSensorSpec::cubemapSize,
          R"(If not set, will be the min(height, width) of resolution)")
      .def_readwrite("sensor_model_type", &FisheyeSensorSpec::fisheyeModelType);

  // ====FisheyeSensorDoubleSphereSpec ====
  /* alpha and xi are specific to "double sphere" camera model.
    see details (value ranges) in:
    Vladyslav Usenko, Nikolaus Demmel and Daniel Cremers: The Double Sphere
    Camera Model, The International Conference on 3D Vision (3DV), 2018
  */
  py::class_<FisheyeSensorDoubleSphereSpec, FisheyeSensorDoubleSphereSpec::ptr,
             FisheyeSensorSpec>(m, "FisheyeSensorDoubleSphereSpec",
                                py::dynamic_attr())
      .def(py::init(&FisheyeSensorDoubleSphereSpec::create<>))
      .def_readwrite("alpha", &FisheyeSensorDoubleSphereSpec::alpha)
      .def_readwrite("xi", &FisheyeSensorDoubleSphereSpec::xi);

  // ==== SensorFactory ====
  py::class_<SensorFactory>(m, "SensorFactory")
      .def("create_sensors", &SensorFactory::createSensors)
      .def("delete_sensor", &SensorFactory::deleteSensor)
      .def("delete_subtree_sensor", &SensorFactory::deleteSubtreeSensor);

  // ==== SensorSuite ====
  py::class_<SensorSuite, Magnum::SceneGraph::PyFeature<SensorSuite>,
             Magnum::SceneGraph::AbstractFeature3D,
             Magnum::SceneGraph::PyFeatureHolder<SensorSuite>>(m, "SensorSuite")
      .def("add", &SensorSuite::add)
      .def("remove", py::overload_cast<const Sensor&>(&SensorSuite::remove))
      .def("remove",
           py::overload_cast<const std::string&>(&SensorSuite::remove))
      .def("clear", &SensorSuite::clear)
      .def("get", &SensorSuite::get)
      .def("get_sensors",
           py::overload_cast<>(&SensorSuite::getSensors, py::const_))
      .def_property_readonly("node", nodeGetter<Sensor>,
                             "Node this object is attached to")
      .def_property_readonly("object", nodeGetter<Sensor>, "Alias to node");

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
      .def_property_readonly(
          "render_camera", &VisualSensor::getRenderCamera,
          R"(Get the RenderCamera in the sensor (if there is one) for rendering PYTHON DOES NOT GET OWNERSHIP)",
          pybind11::return_value_policy::reference)
      .def_property_readonly(
          "near", &VisualSensor::getNear,
          R"(The distance to the near clipping plane this VisualSensor uses.)")
      .def_property_readonly(
          "far", &VisualSensor::getFar,
          R"(The distance to the far clipping plane this VisualSensor uses.)")
      .def_property_readonly(
          "hfov", [](VisualSensor& self) { return Mn::Degd(self.getFOV()); },
          R"(The Field of View this VisualSensor uses.)")
      .def_property_readonly("framebuffer_size", &VisualSensor::framebufferSize)
      .def_property_readonly("render_target", &VisualSensor::renderTarget);

  // === CameraSensor ====
  py::class_<CameraSensor, Magnum::SceneGraph::PyFeature<CameraSensor>,
             VisualSensor, Magnum::SceneGraph::PyFeatureHolder<CameraSensor>>(
      m, "CameraSensor")
      .def(py::init_alias<std::reference_wrapper<scene::SceneNode>,
                          const CameraSensorSpec::ptr&>())
      .def("set_projection_params", &CameraSensor::setProjectionParameters,
           R"(Specify the projection parameters this CameraSensor should use.
           Should be consumed by first querying this CameraSensor's SensorSpec
           and then modifying as necessary.)",
           "sensor_spec"_a)
      .def("zoom", &CameraSensor::modifyZoom,
           R"(Modify Orthographic Zoom or Perspective FOV multiplicatively by
          passed amount. User >1 to increase, 0<factor<1 to decrease.)",
           "factor"_a)
      .def("reset_zoom", &CameraSensor::resetZoom,
           R"(Reset Orthographic Zoom or Perspective FOV.)")
      .def(
          "set_width", &CameraSensor::setWidth,
          R"(Set the width of the resolution in SensorSpec for this CameraSensor.)")
      .def(
          "set_height", &CameraSensor::setHeight,
          R"(Set the height of the resolution in the SensorSpec for this CameraSensor.)")
      .def_property(
          "fov", [](CameraSensor& self) { return Mn::Degd(self.getFOV()); },
          [](CameraSensor& self, const py::object& angle) {
            auto PyDeg = py::module_::import("magnum").attr("Deg");
            self.setFOV(Mn::Deg(PyDeg(angle).cast<Mn::Degd>()));
          },
          R"(Set the field of view to use for this CameraSensor.  Only applicable to
          Pinhole Camera Types)")
      .def_property(
          "camera_type", &CameraSensor::getCameraType,
          &CameraSensor::setCameraType,
          R"(The type of projection (ORTHOGRAPHIC or PINHOLE) this CameraSensor uses.)")
      .def_property(
          "near_plane_dist", &CameraSensor::getNear, &CameraSensor::setNear,
          R"(The distance to the near clipping plane for this CameraSensor uses.)")
      .def_property(
          "far_plane_dist", &CameraSensor::getFar, &CameraSensor::setFar,
          R"(The distance to the far clipping plane for this CameraSensor uses.)");

  // === CubeMapSensorBase ===
  // NOLINTNEXTLINE (bugprone-unused-raii)
  py::class_<CubeMapSensorBase,
             Magnum::SceneGraph::PyFeature<CubeMapSensorBase>, VisualSensor,
             Magnum::SceneGraph::PyFeatureHolder<CubeMapSensorBase>>(
      m, "CubeMapSensorBase");

  // === EquirectangularSensor ===
  py::class_<EquirectangularSensor,
             Magnum::SceneGraph::PyFeature<EquirectangularSensor>,
             CubeMapSensorBase,
             Magnum::SceneGraph::PyFeatureHolder<EquirectangularSensor>>(
      m, "EquirectangularSensor")
      .def(py::init_alias<std::reference_wrapper<scene::SceneNode>,
                          const EquirectangularSensorSpec::ptr&>());

  // === FisheyeSensor ====
  py::class_<FisheyeSensor, Magnum::SceneGraph::PyFeature<FisheyeSensor>,
             CubeMapSensorBase,
             Magnum::SceneGraph::PyFeatureHolder<FisheyeSensor>>(
      m, "FisheyeSensor")
      .def(py::init_alias<std::reference_wrapper<scene::SceneNode>,
                          const FisheyeSensorSpec::ptr&>());

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
