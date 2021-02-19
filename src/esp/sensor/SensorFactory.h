#include "esp/scene/SceneNode.h"
#include "esp/sensor/Sensor.h"

namespace esp {
namespace sensor {
class SensorFactory {
 public:
  /**
   * @brief Static method to initialize and return a SensorSuite of Sensors
   * @param[in] node SceneNode to create child SceneNodes and attach sensors to,
   * sensorSetup SensorSetup a vector of SensorSpec::ptr defining specs for each
   * sensor
   * @return SensorSuite of specified Sensors attached to children SceneNodes of
   * node
   */
  static sensor::SensorSuite createSensors(
      scene::SceneNode& node,
      const sensor::SensorSetup& sensorSetup);
};
}  // namespace sensor
}  // namespace esp
