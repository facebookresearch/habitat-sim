import os.path as osp

import attr
import numpy as np

import numba
from habitat_sim.bindings import SensorType, cuda_enabled
from habitat_sim.sensors.noise_models.registration import (
    SensorNoiseModel,
    register_sensor_noise_model,
)

if cuda_enabled:
    from habitat_sim._ext.habitat_sim_bindings import RedwoodNoiseModelGPUImpl
    import torch


@numba.jit(nopython=True)
def _undistort(x, y, z, model):
    i2 = int((z + 1) / 2)
    i1 = int(i2 - 1)
    a = (z - (i1 * 2.0 + 1.0)) / 2.0
    x = x // 8
    y = y // 6
    f = (1 - a) * model[y, x, min(max(i1, 0), 4)] + a * model[y, x, min(i2, 4)]

    if f < 1e-5:
        return 0
    else:
        return z / f


@numba.jit(nopython=True, parallel=True)
def _simulate(gt_depth, model):
    noisy_depth = np.empty_like(gt_depth)

    H, W = gt_depth.shape
    ymax, xmax = H - 1, W - 1

    rand_nums = np.random.randn(H, W, 3).astype(np.float32)
    for j in range(H):
        for i in range(W):
            y = int(min(max(j + rand_nums[j, i, 0] * 0.25, 0.0), ymax) + 0.5)
            x = int(min(max(i + rand_nums[j, i, 1] * 0.25, 0.0), xmax) + 0.5)

            # Downsample
            d = gt_depth[y - y % 2, x - x % 2]
            # If the depth is greater than 10, the sensor will just return 0
            if d >= 10.0:
                noisy_depth[j, i] = 0.0
            else:
                # Distort
                undistorted_d = _undistort(
                    int(x / xmax * 639.0 + 0.5), int(y / ymax * 479.0 + 0.5), d, model
                )

                if undistorted_d == 0.0:
                    noisy_depth[j, i] = 0.0
                else:
                    denom = round(
                        (35.130 / undistorted_d + rand_nums[j, i, 2] * 0.027778) * 8.0
                    )
                    if denom <= 1e-5:
                        noisy_depth[j, i] = 0.0
                    else:
                        noisy_depth[j, i] = 35.130 * 8.0 / denom

    return noisy_depth


@attr.s(auto_attribs=True)
class RedwoodNoiseModelCPUImpl:
    model: np.ndarray

    def __attrs_post_init__(self):
        self.model = self.model.reshape(self.model.shape[0], -1, 4)

    def simulate(self, gt_depth):
        return _simulate(gt_depth, self.model)


@register_sensor_noise_model
class RedwoodDepthNoiseModel(SensorNoiseModel):
    def __init__(self, gpu_device_id):
        self._gpu_device_id = gpu_device_id

        dist = np.load(
            osp.join(osp.dirname(__file__), "data", "redwood-depth-dist-model.npy")
        )

        if cuda_enabled:
            self._impl = RedwoodNoiseModelGPUImpl(dist, self._gpu_device_id)
        else:
            self._impl = RedwoodNoiseModelCPUImpl(dist)

    @staticmethod
    def is_valid_sensor_type(sensor_type):
        return sensor_type == SensorType.DEPTH

    def simulate(self, gt_depth):
        if cuda_enabled:
            if torch.is_tensor(gt_depth):
                noisy_depth = torch.empty_like(gt_depth)
                rows, cols = gt_depth.size()
                self._impl.simulate_from_gpu(
                    gt_depth.data_ptr(), rows, cols, noisy_depth.data_ptr()
                )
                return noisy_depth
            else:
                return self._impl.simulate_from_cpu(gt_depth)
        else:
            return self._impl.simulate(gt_depth)

    def apply(self, gt_depth):
        return self.simulate(gt_depth)
