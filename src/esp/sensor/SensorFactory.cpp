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
    // VisualSensor Setup
    if (std::dynamic_pointer_cast<VisualSensorSpec>(spec)) {
      if (std::dynamic_pointer_cast<CameraSensorSpec>(spec)) {
        sensorSuite.add(sensor::CameraSensor::create(
            sensorNode, std::dynamic_pointer_cast<CameraSensorSpec>(spec)));
      }
      // TODO: Implement fisheye sensor, Equirectangle sensor, Panorama sensor
      // else if(std::dynamic_pointer_cast<FishEyeSensorSpec>(spec)) {
      //   sensorSuite.add(sensor::FisheyeSensor::create(sensorNode, spec));
      //
    }
    // TODO: Implement NonVisualSensorSpecs
    // else if (std::dynamic_pointer_cast<NonVisualSensorSpec>(spec)) {
    //   //NonVisualSensor Setup
    // }
  }
  return sensorSuite;
}
}  // namespace sensor
}  // namespace esp
