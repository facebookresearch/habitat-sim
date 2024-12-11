// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SENSOR_SENSOR_H_
#define ESP_SENSOR_SENSOR_H_

#include <Magnum/Math/Vector3.h>
#include <Magnum/SceneGraph/Object.h>
#include "esp/core/Buffer.h"
#include "esp/core/Esp.h"

#include "esp/sensor/configure.h"

namespace esp {
namespace scene {
class SceneNode;
}

namespace sim {
class Simulator;
}

namespace sensor {
// Enumeration of types of sensors
enum class SensorType : int32_t {
  Unspecified = 0,
  Custom,
  Color,
  Depth,
  Normal,
  Semantic,
  Audio,
  EndSensorType,
};

enum class ObservationSpaceType {
  None = 0,
  Tensor = 1,
  Text = 2,
  EndObservationSpaceType,
};

enum class SensorSubType : int32_t {
  Unspecified = 0,
  Custom,
  Pinhole,
  Orthographic,
  Fisheye,
  Equirectangular,
  ImpulseResponse,
  EndSensorSubType,
};

// Specifies the configuration parameters of a sensor
// User should make sure all uuids are unique
struct SensorSpec {
  std::string uuid = "";
  SensorType sensorType = SensorType::Unspecified;
  SensorSubType sensorSubType = SensorSubType::Unspecified;
  Magnum::Vector3 position = {0, 1.5, 0};
  Magnum::Vector3 orientation = {0, 0, 0};
  std::string noiseModel = "None";
  SensorSpec() = default;
  virtual ~SensorSpec() = default;
  virtual bool isVisualSensorSpec() const { return false; }
  virtual void sanityCheck() const;
  bool operator==(const SensorSpec& a) const;
  bool operator!=(const SensorSpec& a) const;
  ESP_SMART_POINTERS(SensorSpec)
};

using SensorSetup = std::vector<sensor::SensorSpec::ptr>;
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

// Represents a sensor that provides data from the environment
class Sensor : public Magnum::SceneGraph::AbstractFeature3D {
 public:
  explicit Sensor(scene::SceneNode& node, SensorSpec::ptr spec);
  ~Sensor() override;

  // Get the scene node being attached to.
  scene::SceneNode& node() { return object(); }
  const scene::SceneNode& node() const { return object(); }

  // Overloads to avoid confusion
  scene::SceneNode& object();
  const scene::SceneNode& object() const;

  /**
   * @brief Return a pointer to this Sensor's SensorSpec
   */
  SensorSpec::ptr specification() const { return spec_; }

  /**
   * @brief Return whether or not this Sensor is a VisualSensor
   */
  virtual bool isVisualSensor() const { return false; }

  /**
   * @brief Return whether or not this Sensor can use the HBAO effect
   */
  virtual bool canUseHBAO() const { return false; }

  /**
   * @brief Sets node's position and orientation from Sensor's SensorSpec
   * can be called ONLY when it is attached to a scene node
   */
  void setTransformationFromSpec();

  /**
   * @brief Draws an observation to the frame buffer using simulator's renderer,
   * then reads the observation to the sensor's memory buffer
   * @return true if success, otherwise false (e.g., failed to draw or read
   * observation)
   * @param[in] sim Instance of Simulator class for which the observation needs
   *                to be drawn, obs Instance of Observation class in which the
   * observation will be stored
   */
  virtual bool getObservation(sim::Simulator& sim, Observation& obs) = 0;

  /**
   * @brief Updates ObservationSpace space with spaceType, shape, and dataType
   * of this sensor. The information in space is later used to resize the
   * sensor's memory buffer if sensor is resized.
   * @return true if success, otherwise false
   * @param[in] space Instance of ObservationSpace class which will be updated
   * with information from this sensor
   */
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
class SensorSuite : public Magnum::SceneGraph::AbstractFeature3D {
 public:
  explicit SensorSuite(scene::SceneNode& node);

  // Get the scene node being attached to.
  scene::SceneNode& node() { return object(); }
  const scene::SceneNode& node() const { return object(); }

  // Overloads to avoid confusion
  scene::SceneNode& object();

  const scene::SceneNode& object() const;

  /**
   * @brief Add Sensor sensor to existing sensors_ with key sensor's uuid
   * @param[in] sensor to be added to sensors_
   * Note: it does not update any element whose key already exists.
   */
  void add(Sensor& sensor);

  /**
   * @brief Remove Sensor sensor from existing sensors_
   * @param[in] sensor to be removed from sensors_
   */
  void remove(const Sensor& sensor);

  /**
   * @brief Remove Sensor with key uuid from existing sensors_
   * @param[in] uuid of Sensor to be removed from sensors_
   */
  void remove(const std::string& uuid);

  /**
   * @brief Clear all entries of sensors_
   */
  void clear();

  /**
   * @brief Return reference to Sensor with key uuid in existing sensors_
   * @param[in] uuid of Sensor to be found in sensors_
   */
  sensor::Sensor& get(const std::string& uuid) const;

  /**
   * @brief Return sensors_, map of uuid keys and Sensor values
   */
  std::map<std::string, std::reference_wrapper<sensor::Sensor>>& getSensors() {
    return sensors_;
  }

  const std::map<std::string, std::reference_wrapper<sensor::Sensor>>&
  getSensors() const {
    return sensors_;
  }

 protected:
  std::map<std::string, std::reference_wrapper<sensor::Sensor>> sensors_;

  ESP_SMART_POINTERS(SensorSuite)
};

}  // namespace sensor
}  // namespace esp

#endif  // ESP_SENSOR_SENSOR_H_
