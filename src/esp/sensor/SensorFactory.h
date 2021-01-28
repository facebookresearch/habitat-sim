#include "esp/sensor/Sensor.h"
#include "esp/scene/SceneNode.h"

namespace esp {
    class SensorFactory {
    public:
        virtual ~SensorFactory();
        static sensor::SensorSuite createSensors(scene::SceneNode& node, sensor::SensorSetup& sensorSetup);
    private:
        ESP_SMART_POINTERS(SensorFactory)
};
} //namespace esp
