import os.path as osp

import attr
import numba
import numpy as np

from habitat_sim.bindings import simulate_redwood_depth_noise_cpu

try:
    from habitat_sim._ext.habitat_sim_bindings import simulate_redwood_depth_noise_gpu

    has_gpu = True
except ImportError:
    has_gpu = False


@attr.s
class _DepthNoiseModel:
    model = attr.ib(default=None, init=False)

    def __attrs_post_init__(self):

        data = np.loadtxt(
            osp.join(osp.dirname(__file__), "dist-model.txt"), comments="%", skiprows=5
        )
        dist = np.empty([80, 80, 5])

        for y in range(0, 80):
            for x in range(0, 80):
                idx = (y * 80 + x) * 23 + 3
                if (data[idx : idx + 5] < 8000).all():
                    dist[y, x, :] = 0
                else:
                    dist[y, x, :] = data[idx + 15 : idx + 20]

        self.model = dist

    def simulate(self, gt_depth):
        if has_gpu:
            pass
        else:
            return simulate_redwood_depth_noise_cpu(
                gt_depth, self.model.reshape(self.model.shape[0], -1)
            )

    def __call__(self, gt_depth):
        return self.simulate(gt_depth)


depth_noise_model = _DepthNoiseModel()
