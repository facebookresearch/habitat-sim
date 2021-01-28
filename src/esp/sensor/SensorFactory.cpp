#include "esp/sensor/SensorFactory.h"

#include "esp/sensor/Sensor.h"
#include "esp/scene/SceneNode.h"

namespace esp {

sensor::SensorSuite SensorFactory::createSensors(scene::SceneNode& node, sensor::SensorSetup& sensorSetup){
    sensor::SensorSuite sensorSuite = sensor::SensorSuite();
    for(sensor::SensorSetup::iterator it = sensorSetup.begin(); it != sensorSetup.end(); ++it) {
        scene::SceneNode& sensorNode = node.createChild();
        sensorSuite.add(sensor::Sensor::create(sensorNode, **it));
    }
    return sensorSuite;
};

} //namespace esp
