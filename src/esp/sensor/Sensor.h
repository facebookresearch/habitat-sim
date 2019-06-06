// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "esp/core/esp.h"

#include "esp/gfx/RenderCamera.h"
#include "esp/scene/AttachedObject.h"
#include "esp/scene/SceneNode.h"

namespace esp {
namespace sensor {

// Enumeration of types of sensors
enum class SensorType {
  NONE = 0,
  COLOR = 1,
  DEPTH = 2,
  NORMAL = 3,
  SEMANTIC = 4,
  PATH = 5,
  GOAL = 6,
  FORCE = 7,
  TENSOR = 8,
  TEXT = 9,
};

// Specifies the configuration parameters of a sensor
struct SensorSpec {
  std::string uuid = "rgba_camera";
  SensorType sensorType = SensorType::COLOR;
  std::string sensorSubtype = "pinhole";
  std::map<std::string, std::string> parameters = {{"near", "0.01"},
                                                   {"far", "1000"},
                                                   {"hfov", "90"}};
  vec3f position = {0, 1.5, 0};
  vec3f orientation = {0, 0, 0};
  vec2i resolution = {84, 84};
  int channels = 4;
  std::string encoding = "rgba_uint8";
  // description of Sensor observation space as gym.spaces.Dict()
  std::string observationSpace = "";
  ESP_SMART_POINTERS(SensorSpec)
};
bool operator==(const SensorSpec& a, const SensorSpec& b);
bool operator!=(const SensorSpec& a, const SensorSpec& b);

// Represents a particular sensor Observation
struct Observation {
  // TODO: populate this struct with raw data
  ESP_SMART_POINTERS(Observation)
};

// Represents a sensor that provides data from the environment to an agent
class Sensor : public scene::AttachedObject {
 public:
  explicit Sensor(scene::SceneNode& node, SensorSpec::ptr spec);
  virtual ~Sensor() {
    // LOG(INFO) << "Deconstructing Sensor";
  }

  SensorSpec::ptr specification() const { return spec_; }

  // can be called ONLY when it is attached to a scene node
  virtual void setTransformationFromSpec();

  virtual bool isVisualSensor() { return false; }

  // visual sensor should implement and override this function
  virtual void setProjectionMatrix(gfx::RenderCamera& targetCamera){};

  virtual Observation getObservation();

 protected:
  SensorSpec::ptr spec_ = nullptr;

  ESP_SMART_POINTERS(Sensor)
};

// Represents a set of sensors, with each sensor being identified through a
// unique id

class SensorSuite {
 public:
  void add(Sensor::ptr sensor);
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
