import timeit

import numpy as np
import torch

from habitat_sim.sensors.noise_models import make_sensor_noise_model

gt_depth = (
    torch.rand(1024, 1024, device=torch.device("cuda", 1), dtype=torch.float32) * 10
)
depth_noise_model = make_sensor_noise_model(
    "RedwoodDepthNoiseModel", dict(gpu_device_id=1)
)

print(torch.abs(gt_depth - depth_noise_model(gt_depth)).mean())


print(
    1000
    / np.min(timeit.repeat(lambda: depth_noise_model(gt_depth), repeat=10, number=1000))
)
