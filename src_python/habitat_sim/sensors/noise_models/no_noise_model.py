#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from typing import Union

import attr
import numpy as np
from numpy import ndarray

from habitat_sim.registry import registry
from habitat_sim.sensor import SensorType
from habitat_sim.sensors.noise_models.sensor_noise_model import SensorNoiseModel

try:
    import torch
except ImportError:
    torch = None


@registry.register_noise_model(name="None")
@attr.s(auto_attribs=True, slots=True)
class NoSensorNoiseModel(SensorNoiseModel):
    @staticmethod
    def is_valid_sensor_type(sensor_type: SensorType) -> bool:
        return True

    def apply(
        self, x: Union[ndarray, "torch.Tensor"]
    ) -> Union[ndarray, "torch.Tensor"]:
        if isinstance(x, np.ndarray):
            return x.copy()
        if torch is not None and torch.is_tensor(x):
            return x.clone()
        return x
