#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from os import path as osp
from typing import Union

import attr
import numba
import numpy as np
from numpy import ndarray

try:
    import torch
    from torch import Tensor
except ImportError:
    torch = None

from habitat_sim._ext.habitat_sim_bindings import SensorType
from habitat_sim.bindings import cuda_enabled
from habitat_sim.registry import registry
from habitat_sim.sensors.noise_models.sensor_noise_model import SensorNoiseModel

if cuda_enabled:
    from habitat_sim._ext.habitat_sim_bindings import RedwoodNoiseModelGPUImpl


# Read about the noise model here: http://www.alexteichman.com/octo/clams/
# Original source code: http://redwood-data.org/indoor/data/simdepth.py
@numba.jit(nopython=True, fastmath=True)
def _undistort(x, y, z, model):
    i2 = int((z + 1) / 2)
    i1 = int(i2 - 1)
    a = (z - (i1 * 2.0 + 1.0)) / 2.0
    x = x // 8
    y = y // 6
    f = (1.0 - a) * model[y, x, min(max(i1, 0), 4)] + a * model[y, x, min(i2, 4)]

    if f < 1e-5:
        return 0.0

    return z / f


@numba.jit(nopython=True, parallel=True, fastmath=True)
def _simulate(gt_depth, model, noise_multiplier):
    noisy_depth = np.empty_like(gt_depth)

    H, W = gt_depth.shape
    ymax, xmax = H - 1.0, W - 1.0

    rand_nums = np.random.randn(H, W, 3).astype(np.float32)

    # Parallelize just the outer loop.  This doesn't change the speed
    # noticably but reduces CPU usage compared to two parallel loops
    for j in numba.prange(H):
        for i in range(W):
            y = int(
                min(max(j + rand_nums[j, i, 0] * 0.25 * noise_multiplier, 0.0), ymax)
                + 0.5
            )
            x = int(
                min(max(i + rand_nums[j, i, 1] * 0.25 * noise_multiplier, 0.0), xmax)
                + 0.5
            )

            # Downsample
            d = gt_depth[y - y % 2, x - x % 2]
            # If the depth is greater than 10, the sensor will just return 0
            if d >= 10.0:
                noisy_depth[j, i] = 0.0
            else:
                # Distort
                # The noise model was originally made for a 640x480 sensor,
                # so re-map our arbitrarily sized sensor to that size!
                undistorted_d = _undistort(
                    int(x / xmax * 639.0 + 0.5), int(y / ymax * 479.0 + 0.5), d, model
                )

                if undistorted_d == 0.0:
                    noisy_depth[j, i] = 0.0
                else:
                    denom = round(
                        (
                            35.130 / undistorted_d
                            + rand_nums[j, i, 2] * 0.027778 * noise_multiplier
                        )
                        * 8.0
                    )
                    if denom <= 1e-5:
                        noisy_depth[j, i] = 0.0
                    else:
                        noisy_depth[j, i] = 35.130 * 8.0 / denom

    return noisy_depth


@attr.s(auto_attribs=True)
class RedwoodNoiseModelCPUImpl:
    model: np.ndarray
    noise_multiplier: float

    def __attrs_post_init__(self):
        self.model = self.model.reshape(80, 80, 5)

    def simulate(self, gt_depth):
        return _simulate(gt_depth, self.model, self.noise_multiplier)


@registry.register_noise_model
@attr.s(auto_attribs=True, kw_only=True)
class RedwoodDepthNoiseModel(SensorNoiseModel):
    noise_multiplier: float = 1.0

    def __attrs_post_init__(self) -> None:
        dist = np.load(
            osp.join(osp.dirname(__file__), "data", "redwood-depth-dist-model.npy")
        )

        if cuda_enabled:
            self._impl = RedwoodNoiseModelGPUImpl(
                dist, self.gpu_device_id, self.noise_multiplier
            )
        else:
            self._impl = RedwoodNoiseModelCPUImpl(dist, self.noise_multiplier)

    @staticmethod
    def is_valid_sensor_type(sensor_type: SensorType) -> bool:
        return sensor_type == SensorType.DEPTH

    def simulate(self, gt_depth: Union[ndarray, "Tensor"]) -> Union[ndarray, "Tensor"]:
        if cuda_enabled:
            if isinstance(gt_depth, np.ndarray):
                return self._impl.simulate_from_cpu(gt_depth)
            noisy_depth = torch.empty_like(gt_depth)
            rows, cols = gt_depth.size()
            self._impl.simulate_from_gpu(
                gt_depth.data_ptr(), rows, cols, noisy_depth.data_ptr()  # type: ignore[attr-defined]
            )
            return noisy_depth
        else:
            return self._impl.simulate(gt_depth)

    def apply(self, gt_depth: Union[ndarray, "Tensor"]) -> Union[ndarray, "Tensor"]:
        r"""Alias of `simulate()` to conform to base-class and expected API"""
        return self.simulate(gt_depth)
