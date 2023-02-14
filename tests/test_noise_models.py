#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from os import path as osp

import numpy as np
import pytest

import habitat_sim
from habitat_sim.sensors.noise_models import redwood_depth_noise_model
from habitat_sim.sensors.noise_models.redwood_depth_noise_model import (
    RedwoodDepthNoiseModel,
    RedwoodNoiseModelCPUImpl,
)


@pytest.mark.gfxtest
@pytest.mark.skipif(not habitat_sim.cuda_enabled, reason="Test requires cuda")
@pytest.mark.parametrize("noise_multiplier,tolerance", [(0.0, 1e-5), (1.0, 1e-2)])
def test_compare_gpu_cpu_redwood_depth(noise_multiplier: float, tolerance: float):
    depth = np.linspace(0, 20, num=(256 * 256), dtype=np.float32).reshape(256, 256)

    cuda_impl = RedwoodDepthNoiseModel(
        noise_multiplier=noise_multiplier, gpu_device_id=0
    )
    cpu_impl = RedwoodNoiseModelCPUImpl(
        np.load(
            osp.join(
                osp.dirname(redwood_depth_noise_model.__file__),
                "data",
                "redwood-depth-dist-model.npy",
            )
        ),
        noise_multiplier=noise_multiplier,
    )

    NUM_SIMS = 100
    cuda_depths = [cuda_impl(depth) for _ in range(NUM_SIMS)]
    cpu_depths = [cpu_impl.simulate(depth) for _ in range(NUM_SIMS)]

    cuda_depth = np.mean(np.stack(cuda_depths, 0), 0)
    cpu_depth = np.mean(np.stack(cpu_depths, 0), 0)

    assert np.abs(cuda_depth - cpu_depth).mean() <= tolerance
