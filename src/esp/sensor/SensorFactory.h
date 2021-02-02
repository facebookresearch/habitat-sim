#include "esp/scene/SceneNode.h"
#include "esp/sensor/CameraSensor.h"
#include "esp/sensor/Sensor.h"

namespace esp {
class SensorFactory {
 public:
  static sensor::SensorSuite createSensors(
      scene::SceneNode& node,
      const sensor::SensorSetup& sensorSetup);
};
}  // namespace esp
