#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import numpy as np

from habitat_sim.registry import registry
from habitat_sim.sensors.noise_models.sensor_noise_model import SensorNoiseModel


@registry.register_noise_model(name="None")
class NoSensorNoiseModel(SensorNoiseModel):
    @staticmethod
    def is_valid_sensor_type(sensor_type):
        return True

    def apply(self, x):
        if isinstance(x, np.ndarray):
            return x.copy()
        else:
            return x.clone()
