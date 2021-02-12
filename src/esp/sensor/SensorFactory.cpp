#include "esp/sensor/SensorFactory.h"

#include "esp/scene/SceneNode.h"
#include "esp/sensor/CameraSensor.h"
#include "esp/sensor/Sensor.h"

namespace esp {
namespace sensor {
sensor::SensorSuite SensorFactory::createSensors(
    scene::SceneNode& node,
    const sensor::SensorSetup& sensorSetup) {
  sensor::SensorSuite sensorSuite = sensor::SensorSuite();
  for (const sensor::SensorSpec::ptr& spec : sensorSetup) {
    scene::SceneNode& sensorNode = node.createChild();
    if (spec->sensorType == SensorType::Color) {
      if (spec->sensorSubType == SensorSubType::Pinhole ||
          spec->sensorSubType == SensorSubType::Orthographic) {
        sensorSuite.add(sensor::CameraSensor::create(sensorNode, spec));
      }
      // TODO: Implement fisheye sensor, Equirectangle sensor, Panorama sensor
      // else if (spec->sensorSubType == SensorSubType::Fisheye) {
      //   sensorSuite.add(sensor::FisheyeSensor::create(sensorNode, spec));
      // }
    }

  }
  return sensorSuite;
}
}  // namespace sensor
}  // namespace esp
