import timeit

import numpy as np

from habitat_sim.sensors.noise_models import make_sensor_noise_model

img = np.random.randint(255, size=(256, 256))

salt_and_pepper_model = make_sensor_noise_model(
    "SaltAndPepperNoiseModel", dict(gpu_device_id=1)
)
gaussian_model = make_sensor_noise_model("GaussianNoiseModel", dict(gpu_device_id=1))
speckle_model = make_sensor_noise_model("SpeckleNoiseModel", dict(gpu_device_id=1))
poisson_model = make_sensor_noise_model("PoissonNoiseModel", dict(gpu_device_id=1))

print("Salt and Pepper Noise:")
print("Mean difference between image and noisy image:")
print(np.mean(np.absolute(img - salt_and_pepper_model(img))))
print("Average executions of noise model per second:")
print(
    1
    / (
        np.min(
            timeit.repeat(lambda: salt_and_pepper_model(img), repeat=10, number=1000)
        )
        / 1000
    )
)
print("\n")

print("Gaussian Noise:")
print("Mean difference between image and noisy image:")
print(np.mean(np.absolute(img - gaussian_model(img))))
print("Average executions of noise model per second:")
print(
    1
    / (
        np.min(timeit.repeat(lambda: gaussian_model(img), repeat=10, number=1000))
        / 1000
    )
)
print("\n")

print("Speckle Noise:")
print("Mean difference between image and noisy image:")
print(np.mean(np.absolute(img - speckle_model(img))))
print("Average executions of noise model per second:")
print(
    1
    / (np.min(timeit.repeat(lambda: speckle_model(img), repeat=10, number=1000)) / 1000)
)
print("\n")

print("Poisson Noise:")
print("Mean difference between image and noisy image:")
print(np.mean(np.absolute(img - poisson_model(img))))
print("Average executions of noise model per second:")
print(
    1
    / (np.min(timeit.repeat(lambda: poisson_model(img), repeat=10, number=1000)) / 1000)
)
