#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import attr
import numpy as np

from habitat_sim.registry import registry
from habitat_sim.sensor import SensorType
from habitat_sim.sensors.noise_models.sensor_noise_model import SensorNoiseModel


def _simulate(image):

    image = image / 255.0

    values = len(np.unique(image))
    values = 2 ** np.ceil(np.log2(values))

    noisy = np.random.poisson(image * values) / float(values)
    noisy_rgb = np.clip(noisy, 0, 1)
    noisy_rgb = (noisy_rgb * 255).astype(np.uint8)

    return noisy_rgb


@attr.s(auto_attribs=True)
class PoissonNoiseModelCPUImpl:
    @staticmethod
    def simulate(image):
        return _simulate(image)


@registry.register_noise_model
@attr.s(auto_attribs=True, kw_only=True)
class PoissonNoiseModel(SensorNoiseModel):
    def __attrs_post_init__(self):
        self._impl = PoissonNoiseModelCPUImpl()

    @staticmethod
    def is_valid_sensor_type(sensor_type: SensorType) -> bool:
        return sensor_type == SensorType.COLOR

    def simulate(self, image):
        return self._impl.simulate(image)

    def apply(self, image):
        r"""Alias of `simulate()` to conform to base-class and expected API
        """
        return self.simulate(image)
