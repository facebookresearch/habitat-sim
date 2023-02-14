#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import attr
import numpy as np
from numpy import ndarray

from habitat_sim.registry import registry
from habitat_sim.sensor import SensorType
from habitat_sim.sensors.noise_models.sensor_noise_model import SensorNoiseModel


def _simulate(
    image: ndarray, intensity_constant: float, mean: int, sigma: int
) -> ndarray:
    image = image / 255.0

    noise = np.random.normal(mean, sigma, image.shape)
    noisy_rgb = np.clip((image + (image * noise * intensity_constant)), 0, 1)

    noisy_rgb = (noisy_rgb * 255).astype(np.uint8)

    return noisy_rgb


@attr.s(auto_attribs=True, slots=True)
class SpeckleNoiseModelCPUImpl:
    intensity_constant: float
    mean: int
    sigma: int

    def simulate(self, image: ndarray) -> ndarray:
        return _simulate(image, self.intensity_constant, self.mean, self.sigma)


@registry.register_noise_model
@attr.s(auto_attribs=True, kw_only=True, slots=True)
class SpeckleNoiseModel(SensorNoiseModel):
    intensity_constant: float = 0.2
    mean: int = 0
    sigma: int = 1
    _impl: SpeckleNoiseModelCPUImpl = None

    def __attrs_post_init__(self) -> None:
        self._impl = SpeckleNoiseModelCPUImpl(
            self.intensity_constant, self.mean, self.sigma
        )

    @staticmethod
    def is_valid_sensor_type(sensor_type: SensorType) -> bool:
        return sensor_type == SensorType.COLOR

    def simulate(self, image: ndarray) -> ndarray:
        return self._impl.simulate(image)

    def apply(self, image: ndarray) -> ndarray:
        r"""Alias of `simulate()` to conform to base-class and expected API"""
        return self.simulate(image)
