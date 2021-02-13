#include "esp/sensor/SensorFactory.h"

#include "esp/scene/SceneNode.h"
#include "esp/sensor/CameraSensor.h"
#include "esp/sensor/FisheyeSensor.h"

namespace esp {
namespace sensor {
sensor::SensorSuite SensorFactory::createSensors(
    scene::SceneNode& node,
    const sensor::SensorSetup& sensorSetup) {
  sensor::SensorSuite sensorSuite = sensor::SensorSuite();
  for (const sensor::SensorSpec::ptr& spec : sensorSetup) {
    scene::SceneNode& sensorNode = node.createChild();

    if (spec->sensorSubType == sensor::SensorSubType::Fisheye) {
      sensorSuite.add(sensor::FisheyeSensor::create(sensorNode, spec));
    } else if (spec->sensorSubType == sensor::SensorSubType::Orthographic ||
               spec->sensorSubType == sensor::SensorSubType::Pinhole) {
      sensorSuite.add(sensor::CameraSensor::create(sensorNode, spec));
    }
  }

  return sensorSuite;
}
}  // namespace sensor
}  // namespace esp
