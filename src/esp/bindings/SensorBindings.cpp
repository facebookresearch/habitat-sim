// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/Bindings.h"

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

#include "esp/sensor/AudioSensor.h"

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
      .value("SEMANTIC", SensorType::Semantic)
      .value("AUDIO", SensorType::Audio);

  py::enum_<SensorSubType>(m, "SensorSubType")
      .value("NONE", SensorSubType::None)
      .value("PINHOLE", SensorSubType::Pinhole)
      .value("ORTHOGRAPHIC", SensorSubType::Orthographic)
      .value("FISHEYE", SensorSubType::Fisheye)
      .value("EQUIRECTANGULAR", SensorSubType::Equirectangular)
      .value("IMPULSERESPONSE", SensorSubType::ImpulseResponse);
  ;

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

#ifdef ESP_BUILD_WITH_AUDIO
  // ==== RLRAudioPropagation Configuration ====
  py::class_<RLRA_ContextConfiguration>(
      m, "RLRAudioPropagationConfiguration")
      .def(py::init<>())
      .def_readwrite(
          "sampleRate", &RLRA_ContextConfiguration::sampleRate,
          R"(int | 44100 | Sample rate for the simulated audio)")
      .def_readwrite(
          "frequencyBands", &RLRA_ContextConfiguration::frequencyBands,
          R"(int | 4 | Number of frequency bands in the audio simulation)")
      .def_readwrite(
          "directSHOrder", &RLRA_ContextConfiguration::directSHOrder,
          R"(int | 3 | The spherical harmonic order used for calculating direct sound spatialization for area sources (those with non-zero radii). It is not recommended to go above order 5.)")
      .def_readwrite(
          "indirectSHOrder", &RLRA_ContextConfiguration::indirectSHOrder,
          R"(int | 1 | The spherical harmonic order used for calculating the spatialization of indirect sound (reflections, reverb). It is not recommended to go above order 5. Increasing this value requires more rays to be traced for the results to converge properly, and uses substantially more memory (scales quadratically).)")
      .def_readwrite(
          "threadCount", &RLRA_ContextConfiguration::threadCount,
          R"(int | 1 | Number of CPU threads the simulation will use)")
      .def_readwrite(
          "maxIRLength", &RLRA_ContextConfiguration::maxIRLength,
          R"(float | 4.f | Maximum impulse response length for the audio simulation)")
      .def_readwrite(
          "unitScale", &RLRA_ContextConfiguration::unitScale,
          R"(float | 1.f | Unit scale for the scene. Mesh and positions are multiplied by this factor to convert to meters.)")
      .def_readwrite(
          "globalVolume", &RLRA_ContextConfiguration::globalVolume,
          R"(float | 1.f | A scale factor applied to all audio output)")
      .def_readwrite(
          "directRayCount", &RLRA_ContextConfiguration::directRayCount,
          R"(int | 500 | Maximum number of rays used to compute direct sound for area sources)")
      .def_readwrite(
          "indirectRayCount", &RLRA_ContextConfiguration::indirectRayCount,
          R"(int | 5000 | Number of indirect rays that the ray tracer will emit from the listener)")
      .def_readwrite(
          "indirectRayDepth", &RLRA_ContextConfiguration::indirectRayDepth,
          R"(int | 200 | Maximum number of bounces of each indirect ray emitted by the ray tracer)")
      .def_readwrite(
          "sourceRayCount", &RLRA_ContextConfiguration::sourceRayCount,
          R"(int | 200 | Number of indirect rays that the ray tracer will emit from the source)")
      .def_readwrite(
          "sourceRayDepth", &RLRA_ContextConfiguration::sourceRayDepth,
          R"(int | 10 | Maximum number of bounces of each source ray emitted by the ray tracer)")
      .def_readwrite(
          "maxDiffractionOrder", &RLRA_ContextConfiguration::maxDiffractionOrder,
          R"(int | 10 | The maximum number of edge diffraction events that can occur between a source and listener. This value cannot exceed 10 (compile-time limit))")
      .def_readwrite(
          "direct", &RLRA_ContextConfiguration::direct,
          R"(bool | true | Enable contribution from the direct rays)")
      .def_readwrite(
          "indirect", &RLRA_ContextConfiguration::indirect,
          R"(bool | true | Enable contribution from the indirect rays)")
      .def_readwrite(
          "diffraction", &RLRA_ContextConfiguration::diffraction,
          R"(bool | true | Enable diffraction for the simulation)")
      .def_readwrite(
          "transmission", &RLRA_ContextConfiguration::transmission,
          R"(bool | false | Enable transmission of rays)")
      .def_readwrite(
          "meshSimplification", &RLRA_ContextConfiguration::meshSimplification,
          R"(bool | false | Uses a series of mesh simplification operations to reduce the mesh complexity for ray tracing. Vertex welding is applied, followed by simplification using the edge collapse algorithm.)")
      .def_readwrite(
          "temporalCoherence", &RLRA_ContextConfiguration::temporalCoherence,
          R"(bool | false | Turn on/off temporal smoothing of the impulse response. This uses the impulse response from the previous simulation time step as a starting point for the next time step. This reduces the number of rays required by about a factor of 10, resulting in faster simulations, but should not be used if the motion of sources/listeners is not continuous.)");

  py::enum_<RLRA_ChannelLayoutType>(
      m, "RLRAudioPropagationChannelLayoutType")
      .value("Unknown", RLRA_ChannelLayoutType_Unknown,
             R"(Unknown channel layout type)")
      .value(
          "Mono", RLRA_ChannelLayoutType_Mono,
          R"(Monaural channel layout that does not have any spatial information. This layout has 1 channel)")
      .value(
          "Binaural", RLRA_ChannelLayoutType_Binaural,
          R"(Channel layout with 2 channels that spatializes audio using an HRTF)")
      .value(
          "Ambisonics", RLRA_ChannelLayoutType_Ambisonics,
          R"(Channel layout that encodes fully spherical spatial audio as a set of spherical harmonic basis function coefficients)");

  // ==== RLRAudioPropagation::ChannelLayout ====
  py::class_<RLRA_ChannelLayout>(
      m, "RLRAudioPropagationChannelLayout")
      .def(py::init<>())
      .def_readwrite(
          "type", &RLRA_ChannelLayout::type,
          R"(enum | RLRAudioPropagationChannelLayoutType.Binaural | Channel layout type for the simulated audio)")
      .def_readwrite(
          "channelCount", &RLRA_ChannelLayout::channelCount,
          R"(int | 2 | Number of output channels in simulated audio)");

#else
  py::class_<AudioEmptyStubConfigurationClass>(
      m, "RLRAudioPropagationConfiguration")
      .def(py::init<>());
  py::class_<AudioEmptyStubChannelLayoutClass>(
      m, "RLRAudioPropagationChannelLayout")
      .def(py::init<>());
  m.attr("RLRAudioPropagationChannelLayoutType") = py::none();
#endif  // ESP_BUILD_WITH_AUDIO

#ifdef ESP_BUILD_WITH_AUDIO
  // ==== AudioSensorSpec ====
  py::class_<AudioSensorSpec, AudioSensorSpec::ptr, SensorSpec>(
      m, "AudioSensorSpec", py::dynamic_attr())
      .def(py::init(&AudioSensorSpec::create<>))
      .def_readwrite(
          "acousticsConfig", &AudioSensorSpec::acousticsConfig_,
          R"(RLRAudioPropagationConfiguration | Defined in the relevant section | Acoustic configuration struct that defines simulation parameters)")
      .def_readwrite(
          "channelLayout", &AudioSensorSpec::channelLayout_,
          R"(RLRAudioPropagationChannelLayout | Defined in the relevant section | Channel layout for simulated output audio)")
      .def_readwrite(
          "enableMaterials", &AudioSensorSpec::enableMaterials_,
          R"(bool | true | Enable audio materials)");
#else
  py::class_<AudioSensorSpec, AudioSensorSpec::ptr, SensorSpec>(
      m, "AudioSensorSpec", py::dynamic_attr())
      .def(py::init(&AudioSensorSpec::create<>));
#endif  // ESP_BUILD_WITH_AUDIO

#ifdef ESP_BUILD_WITH_AUDIO
  // ==== AudioSensor ====
  py::class_<AudioSensor, Magnum::SceneGraph::PyFeature<AudioSensor>, Sensor,
             Magnum::SceneGraph::PyFeatureHolder<AudioSensor>>(m, "AudioSensor")
      .def(py::init_alias<std::reference_wrapper<scene::SceneNode>,
                          const AudioSensorSpec::ptr&>())
      .def("setAudioSourceTransform", &AudioSensor::setAudioSourceTransform)
      .def("setAudioListenerTransform", &AudioSensor::setAudioListenerTransform)
      .def("runSimulation", &AudioSensor::runSimulation)
      .def("setAudioMaterialsJSON", &AudioSensor::setAudioMaterialsJSON)
      .def("setListenerHRTF", &AudioSensor::setListenerHRTF)
      .def("getIR", &AudioSensor::getIR)
      .def("writeIRWave", &AudioSensor::writeIRWave)
      .def("getRayEfficiency", &AudioSensor::getRayEfficiency)
      .def("writeSceneMeshOBJ", &AudioSensor::writeSceneMeshOBJ)
      .def("reset", &AudioSensor::reset);
#else
  py::class_<AudioSensor, Magnum::SceneGraph::PyFeature<AudioSensor>, Sensor,
             Magnum::SceneGraph::PyFeatureHolder<AudioSensor>>(m, "AudioSensor")
      .def(py::init_alias<std::reference_wrapper<scene::SceneNode>,
                          const AudioSensorSpec::ptr&>());
#endif  // ESP_BUILD_WITH_AUDIO
}

}  // namespace sensor
}  // namespace esp
