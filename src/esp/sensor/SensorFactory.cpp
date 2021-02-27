#include "esp/sensor/SensorFactory.h"

#include "esp/scene/SceneNode.h"
#include "esp/sensor/CameraSensor.h"

namespace esp {
namespace sensor {
sensor::SensorSuite SensorFactory::createSensors(
    scene::SceneNode& node,
    const sensor::SensorSetup& sensorSetup) {
  sensor::SensorSuite sensorSuite = sensor::SensorSuite();
  for (const SensorSpec::ptr& spec : sensorSetup) {
    scene::SceneNode& sensorNode = node.createChild();
    // VisualSensor Setup
    if (spec->isVisualSensorSpec()) {
      if (spec->sensorSubType == SensorSubType::Orthographic ||
          spec->sensorSubType == SensorSubType::Pinhole) {
        sensorSuite.add(CameraSensor::create(
            sensorNode, std::dynamic_pointer_cast<CameraSensorSpec>(spec)));
      }
      // TODO: Implement fisheye sensor, Equirectangle sensor, Panorama sensor
      // else if(spec->sensorSubType == SensorSubType::Fisheye) {
      //   sensorSuite.add(sensor::FisheyeSensor::create(sensorNode, spec));
      //
    }
    // TODO: Implement NonVisualSensorSpecs
    // else if (!spec->isVisualSensorSpec()) {}
    //   //NonVisualSensor Setup
    // }
  }
  return sensorSuite;
}
}  // namespace sensor
}  // namespace esp
