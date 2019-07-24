import timeit

import numpy as np

from habitat_sim.sensors.depth_noise_model import depth_noise_model

gt_depth = np.random.uniform(0, 10, size=(256, 256))

print(np.abs(gt_depth - depth_noise_model(gt_depth)).mean())


print(
    100
    / np.mean(timeit.repeat(lambda: depth_noise_model(gt_depth), repeat=5, number=100))
)
