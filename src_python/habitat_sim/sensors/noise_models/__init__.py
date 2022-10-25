#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from typing import Any, Dict

from habitat_sim.registry import registry
from habitat_sim.sensors.noise_models.gaussian_noise_model import GaussianNoiseModel
from habitat_sim.sensors.noise_models.no_noise_model import NoSensorNoiseModel
from habitat_sim.sensors.noise_models.poisson_noise_model import PoissonNoiseModel
from habitat_sim.sensors.noise_models.redwood_depth_noise_model import (
    RedwoodDepthNoiseModel,
)
from habitat_sim.sensors.noise_models.salt_and_pepper_noise_model import (
    SaltAndPepperNoiseModel,
)
from habitat_sim.sensors.noise_models.sensor_noise_model import SensorNoiseModel
from habitat_sim.sensors.noise_models.speckle_noise_model import SpeckleNoiseModel


def make_sensor_noise_model(name: str, kwargs: Dict[str, Any]) -> SensorNoiseModel:
    r"""Constructs a noise model using the given name and keyword arguments

    :param name: The name of the noise model in the `habitat_sim.registry`
    :param kwargs: The keyword arguments to be passed to the constructor of the noise model
    """

    model = registry.get_noise_model(name)
    assert model is not None, "Could not find a noise model for name '{}'".format(name)

    return model(**kwargs)


__all__ = [
    "make_sensor_noise_model",
    "SensorNoiseModel",
    "RedwoodDepthNoiseModel",
    "NoSensorNoiseModel",
    "GaussianNoiseModel",
    "SaltAndPepperNoiseModel",
    "PoissonNoiseModel",
    "SpeckleNoiseModel",
]
