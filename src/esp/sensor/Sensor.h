// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SENSOR_SENSOR_H_
#define ESP_SENSOR_SENSOR_H_

#include "esp/core/esp.h"

#include "esp/core/Buffer.h"
#include "esp/scene/SceneNode.h"

namespace esp {
namespace sim {
class Simulator;
}

namespace sensor {

// Enumeration of types of sensors
enum class SensorType {
  None = 0,
  Color = 1,
  Depth = 2,
  Normal = 3,
  Semantic = 4,
  Path = 5,
  Goal = 6,
  Force = 7,
  Tensor = 8,
  Text = 9,
};

enum class ObservationSpaceType {
  None = 0,
  Tensor = 1,
  Text = 2,
};

enum class SensorSubType {
  Pinhole = 0,
  Orthographic = 1,
};

// Specifies the configuration parameters of a sensor
struct SensorSpec {
  std::string uuid = "rgba_camera";
  SensorType sensorType = SensorType::Color;
  SensorSubType sensorSubType = SensorSubType::Pinhole;
  std::map<std::string, std::string> parameters = {{"near", "0.01"},
                                                   {"far", "1000"},
                                                   {"hfov", "90"},
                                                   {"ortho_scale", ".1"}};
  vec3f position = {0, 1.5, 0};
  vec3f orientation = {0, 0, 0};
  vec2i resolution = {84, 84};
  int channels = 4;
  std::string encoding = "rgba_uint8";
  // description of Sensor observation space as gym.spaces.Dict()
  std::string observationSpace = "";
  std::string noiseModel = "None";
  bool gpu2gpuTransfer = false;
  ESP_SMART_POINTERS(SensorSpec)
};

bool operator==(const SensorSpec& a, const SensorSpec& b);
bool operator!=(const SensorSpec& a, const SensorSpec& b);

// Represents a particular sensor Observation
struct Observation {
  // TODO: populate this struct with raw data
  core::Buffer::ptr buffer{nullptr};
  ESP_SMART_POINTERS(Observation)
};

struct ObservationSpace {
  ObservationSpaceType spaceType = ObservationSpaceType::Tensor;
  core::DataType dataType = core::DataType::DT_UINT8;
  std::vector<size_t> shape;
  ESP_SMART_POINTERS(ObservationSpace)
};

// Represents a sensor that provides data from the environment to an agent
class Sensor : public Magnum::SceneGraph::AbstractFeature3D {
 public:
  explicit Sensor(scene::SceneNode& node, SensorSpec::ptr spec);
  virtual ~Sensor() { LOG(INFO) << "Deconstructing Sensor"; }

  // Get the scene node being attached to.
  scene::SceneNode& node() { return object(); }
  const scene::SceneNode& node() const { return object(); }

  // Overloads to avoid confusion
  scene::SceneNode& object() {
    return static_cast<scene::SceneNode&>(
        Magnum::SceneGraph::AbstractFeature3D::object());
  }
  const scene::SceneNode& object() const {
    return static_cast<const scene::SceneNode&>(
        Magnum::SceneGraph::AbstractFeature3D::object());
  }

  SensorSpec::ptr specification() const { return spec_; }

  // can be called ONLY when it is attached to a scene node
  virtual void setTransformationFromSpec();

  virtual bool isVisualSensor() { return false; }

  virtual bool getObservation(sim::Simulator& sim, Observation& obs) = 0;
  virtual bool getObservationSpace(ObservationSpace& space) = 0;

  /**
   * @brief Display next observation from Simulator on default frame buffer
   * @param[in] sim Instance of Simulator class for which the observation needs
   *                to be displayed
   * @return Whether the display process was successful or not
   */
  virtual bool displayObservation(sim::Simulator& sim) = 0;

 protected:
  SensorSpec::ptr spec_ = nullptr;
  core::Buffer::ptr buffer_ = nullptr;

  ESP_SMART_POINTERS(Sensor)
};

// Represents a set of sensors, with each sensor being identified through a
// unique id

class SensorSuite {
 public:
  void add(const Sensor::ptr& sensor);
  void clear();
  ~SensorSuite() { LOG(INFO) << "Deconstructing SensorSuite"; }

  Sensor::ptr get(const std::string& uuid) const;
  std::map<std::string, Sensor::ptr>& getSensors() { return sensors_; }

 protected:
  std::map<std::string, Sensor::ptr> sensors_;

  ESP_SMART_POINTERS(SensorSuite)
};

}  // namespace sensor
}  // namespace esp

#endif  // ESP_SENSOR_SENSOR_H_
