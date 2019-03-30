import habitat_sim.bindings as hsim


class SensorSuite(dict):
    def add(self, sensor: hsim.Sensor):
        self[sensor.specification().uuid] = sensor
