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
  // ==== RLRAudioPropagation::Config ====
  py::class_<RLRAudioPropagation::Configuration>(
      m, "RLRAudioPropagationConfiguration")
      .def(py::init<>())
      .def_readwrite("sampleRate",
                     &RLRAudioPropagation::Configuration::sampleRate,
                     R"(int | 44100 | Sample rate for the simulated audio)")
      .def_readwrite(
          "frequencyBands", &RLRAudioPropagation::Configuration::frequencyBands,
          R"(int | 4 | Number of frequency bands in the audio simulation)")
      .def_readwrite(
          "directSHOrder", &RLRAudioPropagation::Configuration::directSHOrder,
          R"(int | 3 | The spherical harmonic order used for calculating direct sound spatialization for non-point sources (those with non-zero radii). It is not recommended to go above order 9.)")
      .def_readwrite(
          "indirectSHOrder",
          &RLRAudioPropagation::Configuration::indirectSHOrder,
          R"(int | 1 | The spherical harmonic order used for calculating the spatialization of indirect sound (reflections, reverb). It is not recommended to go above order 5. Increasing this value requires more rays to be traced for the results to converge properly, and uses substantially more memory (scales quadratically).)")
      .def_readwrite(
          "threadCount", &RLRAudioPropagation::Configuration::threadCount,
          R"(int | 1 | Number of CPU thread the simulation will use)")
      .def_readwrite("updateDt", &RLRAudioPropagation::Configuration::updateDt,
                     R"(float | 0.02f | Simulation time step)")
      .def_readwrite(
          "irTime", &RLRAudioPropagation::Configuration::irTime,
          R"(float | 4.f | Maximum render time budget for the audio simulation)")
      .def_readwrite(
          "unitScale", &RLRAudioPropagation::Configuration::unitScale,
          R"(float | 1.f | Unit scale for the scene. Mesh and positions are multiplied by this factor)")
      .def_readwrite("globalVolume",
                     &RLRAudioPropagation::Configuration::globalVolume,
                     R"(float | 4.f | Total initial pressure value)")
      .def_readwrite(
          "indirectRayCount",
          &RLRAudioPropagation::Configuration::indirectRayCount,
          R"(int | 5000 | Number of indirect rays that the ray tracer will use)")
      .def_readwrite(
          "indirectRayDepth",
          &RLRAudioPropagation::Configuration::indirectRayDepth,
          R"(int | 200 | Maximum depth of each indirect ray cast by the ray tracer)")
      .def_readwrite(
          "sourceRayCount", &RLRAudioPropagation::Configuration::sourceRayCount,
          R"(int | 200 | Number of direct rays that the ray tracer will use)")
      .def_readwrite(
          "sourceRayDepth", &RLRAudioPropagation::Configuration::sourceRayDepth,
          R"(int | 10 | Maximum depth of direct rays cast by the ray tracer)")
      .def_readwrite(
          "maxDiffractionOrder",
          &RLRAudioPropagation::Configuration::maxDiffractionOrder,
          R"(int | 10 | The maximum number of edge diffraction events that can occur between a source and listener. This value cannot exceed 10 (compile-time limit))")
      .def_readwrite(
          "direct", &RLRAudioPropagation::Configuration::direct,
          R"(bool | true | Enable contribution from the direct rays)")
      .def_readwrite(
          "indirect", &RLRAudioPropagation::Configuration::indirect,
          R"(bool | true | Enable contribution from the indirect rays)")
      .def_readwrite("diffraction",
                     &RLRAudioPropagation::Configuration::diffraction,
                     R"(bool | true | Enable diffraction for the simulation)")
      .def_readwrite("transmission",
                     &RLRAudioPropagation::Configuration::transmission,
                     R"(bool | false | Enable transmission of rays)")
      .def_readwrite(
          "meshSimplification",
          &RLRAudioPropagation::Configuration::meshSimplification,
          R"(bool | false | Uses a series of mesh simplification operations to reduce the mesh complexity for ray tracing. Vertex welding is applied, followed by simplification using the edge collapse algorithm.)")
      .def_readwrite(
          "temporalCoherence",
          &RLRAudioPropagation::Configuration::temporalCoherence,
          R"(bool | false | Turn on/off temporal smoothing of the impulse response. This uses the impulse response from the previous simulation time step as a starting point for the next time step. This reduces the number of rays required by about a factor of 10, resulting in faster simulations, but should not be used if the motion of sources/listeners is not continuous.)")
      .def_readwrite(
          "dumpWaveFiles", &RLRAudioPropagation::Configuration::dumpWaveFiles,
          R"(bool | false | Write the wave files for different bands. Will be writted to the AudioSensorSpec's outputDirectory)")
      .def_readwrite("enableMaterials",
                     &RLRAudioPropagation::Configuration::enableMaterials,
                     R"(bool | true | Enable audio materials)")
      .def_readwrite(
          "writeIrToFile", &RLRAudioPropagation::Configuration::writeIrToFile,
          R"(bool | false | Write the final impulse response to a file)");

  py::enum_<RLRAudioPropagation::ChannelLayoutType>(
      m, "RLRAudioPropagationChannelLayoutType")
      .value("Unknown", RLRAudioPropagation::ChannelLayoutType::Unknown,
             R"(Unknown channel layout type)")
      .value(
          "Mono", RLRAudioPropagation::ChannelLayoutType::Mono,
          R"(Monaural channel layout that does not have any spatial information. This layout usually has 1 channel)")
      .value(
          "Stereo", RLRAudioPropagation::ChannelLayoutType::Stereo,
          R"(Channel layout with 2 channels (e.g. speakers) that does not use any HRTF)")
      .value(
          "Binaural", RLRAudioPropagation::ChannelLayoutType::Binaural,
          R"(Channel layout with 2 channels that spatializes audio using an HRTF)")
      .value(
          "Quad", RLRAudioPropagation::ChannelLayoutType::Quad,
          R"(Channel layout with 4 channels (speakers) arranged at +-30 and +-95 degrees in the horizontal plane)")
      .value(
          "Surround_5_1", RLRAudioPropagation::ChannelLayoutType::Surround_5_1,
          R"(Channel layout with 6 channels (speakers) arranged at 0, +-30, and +-110 degrees in the horizontal plane, with unpositioned low frequency channel)")
      .value(
          "Surround_7_1", RLRAudioPropagation::ChannelLayoutType::Surround_7_1,
          R"(Channel layout with 8 channels (speakers) arranged at 0, +-30, +-90, and +-135 degrees in the horizontal plane, with unpositioned low frequency channel)")
      .value(
          "Ambisonics", RLRAudioPropagation::ChannelLayoutType::Ambisonics,
          R"(Channel layout that encodes fully spherical spatial audio as a set of spherical harmonic basis function coefficients)");

  // ==== RLRAudioPropagation::ChannelLayout ====
  py::class_<RLRAudioPropagation::ChannelLayout>(
      m, "RLRAudioPropagationChannelLayout")
      .def(py::init<>())
      .def_readwrite(
          "channelType", &RLRAudioPropagation::ChannelLayout::channelType,
          R"(enum | RLRAudioPropagationChannelLayoutType.Binaural | Channel type for the simulated audio)")
      .def_readwrite(
          "channelCount", &RLRAudioPropagation::ChannelLayout::channelCount,
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
          "outputDirectory", &AudioSensorSpec::outputDirectory_,
          R"(string | empty | Output directory prefix for the simulation. Folders with outputDirectory + i should be created if you want to dump the wave files. (i = 0 indexed simulation iteration)")
      .def_readwrite(
          "acousticsConfig", &AudioSensorSpec::acousticsConfig_,
          R"(RLRAudioPropagationConfiguration | Defined in the relevant section | Acoustic configuration struct that defines simulation parameters)")
      .def_readwrite(
          "channelLayout", &AudioSensorSpec::channelLayout_,
          R"(RLRAudioPropagationChannelLayout | Defined in the relevant section | Channel layout for simulated output audio)");
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
      .def("getIR", &AudioSensor::getIR)
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
