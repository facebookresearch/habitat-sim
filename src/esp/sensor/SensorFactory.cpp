// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/sensor/SensorFactory.h"

#include "esp/scene/SceneNode.h"
#include "esp/sensor/CameraSensor.h"
#include "esp/sensor/EquirectangularSensor.h"
#include "esp/sensor/FisheyeSensor.h"
#include "esp/sensor/Sensor.h"

#include "esp/sensor/AudioSensor.h"

namespace esp {
namespace sensor {
std::map<std::string, std::reference_wrapper<sensor::Sensor>>&
SensorFactory::createSensors(scene::SceneNode& node,
                             const sensor::SensorSetup& sensorSetup) {
  for (const SensorSpec::ptr& spec : sensorSetup) {
    scene::SceneNode& sensorNode =
        node.createChild({scene::SceneNodeTag::Leaf});
    // VisualSensor Setup
    if (spec->isVisualSensorSpec()) {
      switch (spec->sensorSubType) {
        case sensor::SensorSubType::Fisheye:
          sensorNode.addFeature<sensor::FisheyeSensor>(
              std::dynamic_pointer_cast<FisheyeSensorSpec>(spec));
          break;
        case sensor::SensorSubType::Orthographic:
          /* fall through */
        case sensor::SensorSubType::Pinhole:
          sensorNode.addFeature<sensor::CameraSensor>(
              std::dynamic_pointer_cast<sensor::CameraSensorSpec>(spec));
          break;
        case sensor::SensorSubType::Equirectangular:
          sensorNode.addFeature<sensor::EquirectangularSensor>(
              std::dynamic_pointer_cast<EquirectangularSensorSpec>(spec));
          break;

          // TODO: implement Panorama sensor
        default:
          ESP_ERROR() << "Unreachable code : Cannot add the specified visual "
                         "sensorType: "
                      << static_cast<std::uint32_t>(spec->sensorType);
          CORRADE_INTERNAL_ASSERT_UNREACHABLE();
          break;
      }
    } else if (!spec->isVisualSensorSpec()) {
      switch (spec->sensorType) {
        case sensor::SensorType::Audio:
          sensorNode.addFeature<sensor::AudioSensor>(
              std::dynamic_pointer_cast<AudioSensorSpec>(spec));
          break;
        default:
          ESP_ERROR() << "Unreachable code : Cannot add the specified "
                         "non-visual sensorType:"
                      << static_cast<std::uint32_t>(spec->sensorType);
          CORRADE_INTERNAL_ASSERT_UNREACHABLE();
          break;
      }
    }
    // TODO: implement any other type of sensors (if applicable)
  }
  return node.getNodeSensors();
}

void SensorFactory::deleteSensor(const sensor::Sensor& sensor) {
  delete (&sensor.node());
}

void SensorFactory::deleteSubtreeSensor(scene::SceneNode& node,
                                        const std::string& uuid) {
  // If sensor does not exist, SensorFactory will log a message but proceed
  // without errors thrown
  if (node.getSubtreeSensors().count(uuid) == 0) {
    ESP_DEBUG() << "Sensor with uuid" << uuid << "does not exist at node"
                << node.getId();
    return;
  }
  deleteSensor(node.getSubtreeSensorSuite().get(uuid));
}

}  // namespace sensor
}  // namespace esp
