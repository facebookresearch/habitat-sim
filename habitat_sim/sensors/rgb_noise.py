'''
Parameters:
image : ndarray, input image data. Will be converted to float.
mode : str
    One of the following strings, selecting the type of noise to add:

    'gauss'     Gaussian-distributed additive noise.
    'poisson'   Poisson-distributed noise generated from the data.
    's&p'       Replaces random pixels with 0 or 1.
    'speckle'   Multiplicative noise using out = image + n*image, where
                n is uniform noise with specified mean & variance.
'''

import numpy as np
import os
import numba
from utils_noise import numpy_arr_to_PIL_image, PIL_image_to_numpy_arr
from PIL import Image

# presample numbers and pass in tensor to numba look into numpy implementation

# pyhthon registry

# imput is numpy array

# implement in pure numpy / torch

# image aug** little rotations, look into image aug to see if other cool features are available

#habitat sim api

# test w/ timeit ~ go for few 1000 frames per second


def rgb_noise(noise_typ, image):
    if noise_typ == "gauss":

        intensity_constant = 0.2
        mean = 0
        sigma = 1

        image = image / 255.0
        noise =  np.random.normal(mean, sigma, image.shape)
        noisy = np.clip((image + (noise * intensity_constant)), 0, 1)
        noisy = np.rint(noisy * 255)

        im = numpy_arr_to_PIL_image(noisy, False)
        im.save('results/test_gauss.png')

        return noisy

    elif noise_typ == "s&p":

        s_vs_p = 0.5
        amount = 0.05
        out = np.copy(image)

        # Salt
        num_salt = np.ceil(amount * image.size * s_vs_p)
        coords = [np.random.randint(0, i - 1, int(num_salt)) for i in image.shape]
        out[tuple(coords)] = 1

        # Pepper
        num_pepper = np.ceil(amount * image.size * (1. - s_vs_p))
        coords = [np.random.randint(0, i - 1, int(num_pepper)) for i in image.shape]
        out[tuple(coords)] = 0

        noisy = out

        im = numpy_arr_to_PIL_image(noisy, False)
        im.save('results/test_s&p.png')

        return out

    elif noise_typ == "poisson":

        image = image / 255.0

        vals = len(np.unique(image))
        vals = 2 ** np.ceil(np.log2(vals))

        noisy = np.random.poisson(image * vals) / float(vals)
        noisy = np.clip(noisy, 0, 1)
        noisy = np.rint(noisy * 255)

        im = numpy_arr_to_PIL_image(noisy, False)
        im.save('results/test_poisson.png')

        return noisy

    elif noise_typ == "speckle":

        intensity_constant = 0.2
        mean = 0
        sigma = 1

        image = image / 255.0
        noise = np.random.normal(mean, sigma, image.shape)
        noisy = image + (image * noise * intensity_constant)
        noisy = np.clip(noisy, 0, 1)
        noisy = np.rint(noisy * 255)

        im = numpy_arr_to_PIL_image(noisy, False)
        im.save('results/test_speckle.png')

        return noisy




images = sorted(os.listdir('images/'))

noise_models = ["gauss", "s&p", "poisson", "speckle"]

for image in images:
    img = Image.open('images/' + image)
    img = PIL_image_to_numpy_arr(img, False)
    for model in noise_models:
        rgb_noise(model, img)
