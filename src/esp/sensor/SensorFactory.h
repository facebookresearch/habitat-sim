#include "esp/sensor/Sensor.h"
#include "esp/sensor/CameraSensor.h"
#include "esp/scene/SceneNode.h"

namespace esp {
    class SensorFactory {
    public:
        virtual ~SensorFactory();
        static sensor::SensorSuite createSensors(scene::SceneNode& node, const sensor::SensorSetup& sensorSetup) {
            sensor::SensorSuite sensorSuite = sensor::SensorSuite();
            for(sensor::SensorSpec::ptr spec : sensorSetup) {
                scene::SceneNode& sensorNode = node.createChild();
                sensorSuite.add(sensor::CameraSensor::create(sensorNode, spec));
            }
            return sensorSuite;
        }
    private:
        ESP_SMART_POINTERS(SensorFactory)
};
} //namespace esp
