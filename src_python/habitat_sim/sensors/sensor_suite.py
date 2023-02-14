# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.from typing import Dict

from typing import Dict

from habitat_sim import bindings as hsim


class SensorSuite(Dict[str, hsim.Sensor]):
    r"""Holds all the agents sensors. Simply a dictionary with an extra method
    to lookup the name of a sensor as the key
    """

    def add(self, sensor: hsim.Sensor) -> None:
        self[sensor.specification().uuid] = sensor
