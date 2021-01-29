#include "esp/sensor/SensorFactory.h"

#include "esp/scene/SceneNode.h"
#include "esp/sensor/CameraSensor.h"
#include "esp/sensor/Sensor.h"

namespace esp {
sensor::SensorSuite SensorFactory::createSensors(
    scene::SceneNode& node,
    const sensor::SensorSetup& sensorSetup) {
  sensor::SensorSuite sensorSuite = sensor::SensorSuite();
  for (sensor::SensorSpec::ptr spec : sensorSetup) {
    scene::SceneNode& sensorNode = node.createChild();
    sensorSuite.add(sensor::CameraSensor::create(sensorNode, spec));
  }
  return sensorSuite;
}
}  // namespace esp
