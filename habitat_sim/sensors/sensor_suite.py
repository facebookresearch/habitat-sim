from habitat_sim import bindings as hsim


class SensorSuite(dict):
    r"""Holds all the agents sensors. Simply a dictionary with an extra method
    to lookup the name of a sensor as the key
    """

    def add(self, sensor: hsim.Sensor) -> None:
        self[sensor.specification().uuid] = sensor
