import timeit

import numpy as np

from habitat_sim.sensors.noise_models import make_sensor_noise_model

gt_depth = np.random.uniform(0, 10, size=(256, 256))
depth_noise_model = make_sensor_noise_model(
    "RedwoodDepthNoiseModel", dict(gpu_device_id=1)
)

print(np.abs(gt_depth - depth_noise_model(gt_depth)).mean())


print(
    1000
    / np.min(timeit.repeat(lambda: depth_noise_model(gt_depth), repeat=10, number=1000))
)
