// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_ABSTRACTSENSORATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_ABSTRACTSENSORATTRIBUTES_H_

#include "AbstractAttributes.h"

namespace esp {
namespace metadata {
namespace attributes {

/**
 * @brief Attributes object holding the descriptions of a Sensor object
 */
class AbstractSensorAttributes : public AbstractAttributes {
 public:
  AbstractSensorAttributes(const std::string& classKey,
                           const std::string& handle);

  /**
   * @brief Populate this attributes from an appropriate @ref sensor::SensorSpec.
   * @todo Remove when SensorSpecs are removed
   *
   */
  virtual void populateWithSensorSpec(const sensor::SensorSpec::ptr& spec);

  /** @brief Set the position of the sensor. */
  void setPosition(const Magnum::Vector3& position) {
    set("position", position);
  }

  /** @brief Get the position of the sensor. */
  Magnum::Vector3 getPosition() const {
    return get<Magnum::Vector3>("position");
  }

  /** @brief Set the position of the sensor. */
  void setOrientation(const Magnum::Vector3& orientation) {
    set("orientation", orientation);
  }

  /** @brief Get the position of the sensor. */
  Magnum::Vector3 getOrientation() const {
    return get<Magnum::Vector3>("orientation");
  }

  /** @brief Sets the noise model to use for this sensor. */
  void setNoiseModel(const std::string& noise_model) {
    set("noise_model", noise_model);
  }

  /** @brief Gets the noise model to use for this sensor. */
  std::string getNoiseModel() const { return get<std::string>("noise_model"); }

  /**
   * @brief Set the sensor type for this sensor.
   */
  void setSensorType(const std::string& sensorType) {
    // force to lowercase to check if present
    const std::string sensorTypeLC = Cr::Utility::String::lowercase(sensorType);
    auto mapIter = SensorTypeNamesMap.find(sensorTypeLC);
    if (mapIter == SensorTypeNamesMap.end()) {
      ESP_ERROR(Mn::Debug::Flag::NoSpace)
          << "'" << sensorType
          << "' is an illegal value for "
             "AbstractSensorAttributes::setSensorType, so current value "
          << get<std::string>("sensor_type") << " not changed.";
      return;
    }
    setTranslated("sensor_type", sensorType);
  }  // setSensorType

  /**
   * @brief Set the sensor type for this sensor as specified by given @ref SensorType
   */
  void setSensorTypeEnum(sensor::SensorType sensorTypeEnum) {
    // force to lowercase to check if present
    const std::string sensorType = getSensorTypeName(sensorTypeEnum);
    auto mapIter = SensorTypeNamesMap.find(sensorType);

    ESP_CHECK(mapIter != SensorTypeNamesMap.end(),
              "Illegal SensorType enum value given"
                  << static_cast<int>(sensorTypeEnum) << ":" << sensorType
                  << "attempted to be initialized in AbstractSensorAttributes:"
                  << getHandle() << ". Aborting.");

    setTranslated("sensor_type", sensorType);
  }  // setSensorTypeEnum

  /**
   * @brief Get the sensor type for this sensor.
   */
  sensor::SensorType getSensorType() const {
    const std::string val =
        Cr::Utility::String::lowercase(get<std::string>("sensor_type"));
    auto mapIter = SensorTypeNamesMap.find(val);
    if (mapIter != SensorTypeNamesMap.end()) {
      return mapIter->second;
    }
    // This should never get to here. It would mean that this field was set
    // to an invalid value somehow.
    return sensor::SensorType::Unspecified;
  }

  /**
   * @brief Set the sensor type for this sensor.
   */
  void setSensorSubType(const std::string& sensorSubType) {
    // force to lowercase to check if present
    const std::string sensorTypeLC =
        Cr::Utility::String::lowercase(sensorSubType);
    auto mapIter = SensorSubTypeNamesMap.find(sensorTypeLC);
    if (mapIter == SensorSubTypeNamesMap.end()) {
      ESP_ERROR(Mn::Debug::Flag::NoSpace)
          << "'" << sensorSubType
          << "' is an illegal value for "
             "AbstractSensorAttributes::setSensorSubType, so current value "
          << get<std::string>("sensor_subtype") << " not changed.";
      return;
    }
    setTranslated("sensor_subtype", sensorSubType);
  }  // setSensorSubType

  /**
   * @brief Set the sensor subtype for this sensor as specified by given @ref SensorSubType
   */
  void setSensorSubTypeEnum(sensor::SensorSubType sensorSubTypeEnum) {
    // force to lowercase to check if present
    const std::string sensorSubType = getSensorSubTypeName(sensorSubTypeEnum);
    auto mapIter = SensorSubTypeNamesMap.find(sensorSubType);

    ESP_CHECK(mapIter != SensorSubTypeNamesMap.end(),
              "Illegal SensorSubType enum value given"
                  << static_cast<int>(sensorSubTypeEnum) << ":" << sensorSubType
                  << "attempted to be initialized in AbstractSensorAttributes:"
                  << getHandle() << ". Aborting.");

    setTranslated("sensor_subtype", sensorSubType);
  }  // setSensorSubTypeEnum

  /**
   * @brief Get the sensor subtype for this sensor.
   */
  sensor::SensorSubType getSensorSubType() const {
    const std::string val =
        Cr::Utility::String::lowercase(get<std::string>("sensor_subtype"));
    auto mapIter = SensorSubTypeNamesMap.find(val);
    if (mapIter != SensorSubTypeNamesMap.end()) {
      return mapIter->second;
    }
    // This should never get to here. It would mean that this field was set
    // to an invalid value somehow.
    return sensor::SensorSubType::Unspecified;
  }  // getSensorSubType

  /**
   * @brief Populate a json object with all the first-level values held in this
   * configuration. Default is overridden to handle special cases for
   * AbstractSensorAttributes and deriving (i.e. AudioSensorAttributes or
   * VisualSensorAttributes) classes.
   */
  void writeValuesToJson(io::JsonGenericValue& jsonObj,
                         io::JsonAllocator& allocator) const override;

 protected:
  /**
   * @brief Write child-class-specific values to json object
   *
   */
  virtual void writeValuesToJsonInternal(
      CORRADE_UNUSED io::JsonGenericValue& jsonObj,
      CORRADE_UNUSED io::JsonAllocator& allocator) const {}

  /**
   * @brief Retrieve a comma-separated string holding the header values for the
   * info returned for this managed object, type-specific.
   */

  std::string getObjectInfoHeaderInternal() const override;
  /**
   * @brief get AbstractSensorAttributes-specific info header
   */
  virtual std::string getAbstractSensorInfoHeaderInternal() const {
    return "";
  };

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  std::string getObjectInfoInternal() const override;
  /**
   * @brief get AbstractSensorAttributes specific info for csv string
   */
  virtual std::string getAbstractSensorInfoInternal() const { return ""; };

 public:
  ESP_SMART_POINTERS(AbstractSensorAttributes)
};  // class AbstractSensorAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_ABSTRACTSENSORATTRIBUTES_H_
