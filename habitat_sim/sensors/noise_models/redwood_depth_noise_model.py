import os.path as osp

import attr
import numpy as np

import numba
from habitat_sim._ext.habitat_sim_bindings import RedwoodNoiseModelCPUImpl
from habitat_sim.bindings import SensorType
from habitat_sim.sensors.noise_models.registration import (
    SensorNoiseModel,
    register_sensor_noise_model,
)

try:
    from habitat_sim._ext.habitat_sim_bindings import RedwoodNoiseModelGPUImpl

    has_gpu = True
except ImportError:
    has_gpu = False


@register_sensor_noise_model
class RedwoodDepthNoiseModel(SensorNoiseModel):
    def __init__(self):
        dist = np.load(osp.join(osp.dirname(__file__), "data", "dist-model.npy"))

        if has_gpu:
            self._impl = RedwoodNoiseModelGPUImpl(dist)
        else:
            self._impl = RedwoodNoiseModelCPUImpl(dist)

    @staticmethod
    def is_valid_sensor_type(sensor_type):
        return sensor_type == SensorType.DEPTH

    def simulate(self, gt_depth):
        if has_gpu:
            return self._impl.simulate_from_cpu(gt_depth)
        else:
            return self._impl.simulate(gt_depth)

    def apply(self, gt_depth):
        return self.simulate(gt_depth)

    def __call__(self, gt_depth):
        return self.simulate(gt_depth)
