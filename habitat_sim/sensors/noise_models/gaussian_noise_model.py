#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import attr
import numba
import numpy as np

from habitat_sim.registry import registry
from habitat_sim.sensor import SensorType
from habitat_sim.sensors.noise_models.sensor_noise_model import SensorNoiseModel


@numba.jit(nopython=True, parallel=True, fastmath=True)
def _simulate(image, intensity_constant, mean, sigma):
    noise = (
        np.random.randn(image.shape[0], image.shape[1], image.shape[2]) * sigma + mean
    ) * intensity_constant

    noisy_rgb = np.empty_like(image)

    # Parallelize just the outer loop.  This doesn't change the speed
    # noticably but reduces CPU usage compared to all parallel loops
    for i in numba.prange(image.shape[0]):
        for j in range(image.shape[1]):
            for k in range(image.shape[2]):
                noisy_val = image[i, j, k] / 255.0 + noise[i, j, k]
                noisy_val = max(min(noisy_val, 1.0), 0.0)
                noisy_val = noisy_val * 255.0
                noisy_rgb[j, i, k] = noisy_val

    return noisy_rgb


@attr.s(auto_attribs=True)
class GaussianNoiseModelCPUImpl:
    intensity_constant: float
    mean: int
    sigma: int

    def simulate(self, image):
        return _simulate(image, self.intensity_constant, self.mean, self.sigma)


@registry.register_noise_model
@attr.s(auto_attribs=True, kw_only=True)
class GaussianNoiseModel(SensorNoiseModel):
    intensity_constant: float = 0.2
    mean: int = 0
    sigma: int = 1

    def __attrs_post_init__(self):
        self._impl = GaussianNoiseModelCPUImpl(
            self.intensity_constant, self.mean, self.sigma
        )

    @staticmethod
    def is_valid_sensor_type(sensor_type: SensorType) -> bool:
        return sensor_type == SensorType.COLOR

    def simulate(self, image):
        return self._impl.simulate(image)

    def apply(self, image):
        r"""Alias of `simulate()` to conform to base-class and expected API
        """
        return self.simulate(image)
