// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "esp/io/json.h"
#include "esp/scene/SceneNode.h"

#include <string>

namespace esp {
namespace sensor {

// Sensors need to extend this for their specification
struct BaseSensorSpec {
  // Specifies a unique idenfitier for the sensor for e.g. "rgb"
  std::string UID = "base";
  // Specifies the type of sensor
  // TODO: Consider if this should be converted to an enum
  std::string sensorType = "none";
};

// Base class for a sensor register to the simulator
// Sensor provides data from the environment to the agent
class BaseSensor : public Magnum::SceneGraph::AbstractFeature3D {
 public:
  /**
   * @brief Constructor, sets common private variable
   * @param[in] node Instance of SceneNode to which sensor will be attached
   * @param[in] spec Instance of BaseSensorSpec or derived structs describing
   *                 specification of the sensor
   */
  explicit BaseSensor(scene::SceneNode& node, const BaseSensorSpec spec);

  /**
   * @brief Provides the type of the sensor
   * @return string type of the sensor
   *
   * NOTE: Override in child sensor class
   */
  virtual const std::string getType() const = 0;

  /**
   * @brief Universally Unique Identifier for the current instance of the
   *        sensor. Appends a unique string to UID defined by the
   *        sensor specification.
   * @return string UUID for the current instance
   */
  const std::string getUUID() const;

  /**
   * @brief Get current observation for the sensor
   * @return unique pointer to char observation
   *
   * NOTE: Override in child sensor class
   */
  virtual std::unique_ptr<char> getObservation() = 0;

  /**
   * @brief Assign current observation of the sensor to the
   *        passed argument
   * @param[in, out] obs Pointer to the variable to which current observation
   *                will be assigned
   *
   * NOTE: Override in child sensor class
   */
  virtual bool getObservation(void* obs);

  /**
   * @brief Get shape of the observation returned by the sensor
   * @return vector<int> Array defining shape of the observation
   *                     returned by the sensor for e.g. [2, 3]
   *
   * NOTE: Override in child sensor class
   */
  virtual const std::vector<int> getObservationShape() const = 0;

  /**
   * @brief Get data type of the observation
   * @return string Type of the observation returned by the sensor
   *                For e.g. "uint32", "double".
   *
   * NOTE: Override in child class
   */
  virtual const std::string getObservationDataType() const = 0;
};

// Factory for creating sensors
class SensorFactory {
 public:
  /**
   * @brief Creates a returns a sensor based on JSON specification
   * @param[in] node a SceneNode to which sensor will be attached
   * @param[in] spec JSONDocument defining specifications of the sensor
   *                 This will be converted to SensorSpec object
   *
   * First, creates SensorSpec object based on the passed JSONDocument
   * object. Then, creates BaseSensor/derived class object using SensorSpec
   * object and SceneNode object.
   */
  static BaseSensor* create(scene::SceneNode& node,
                            const esp::io::JsonDocument spec);
};
}  // namespace sensor
}  // namespace esp
