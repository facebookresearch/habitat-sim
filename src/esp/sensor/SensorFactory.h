#include "esp/scene/SceneNode.h"
#include "esp/sensor/CameraSensor.h"
#include "esp/sensor/Sensor.h"

namespace esp {
class SensorFactory {
 public:
  virtual ~SensorFactory();
  static sensor::SensorSuite createSensors(
      scene::SceneNode& node,
      const sensor::SensorSetup& sensorSetup);

 private:
  ESP_SMART_POINTERS(SensorFactory)
};
}  // namespace esp
