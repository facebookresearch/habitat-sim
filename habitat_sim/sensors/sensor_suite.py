#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from collections import OrderedDict

from habitat_sim.sensor import Sensor


class SensorSuite(OrderedDict):
    r"""Holds all the agents sensors. Simply a dictionary with an extra method
    to lookup the name of a sensor as the key
    """

    def add(self, sensor: Sensor):
        self[sensor.specification().uuid] = sensor
