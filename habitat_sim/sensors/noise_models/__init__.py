#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from habitat_sim.sensors.noise_models.no_noise_model import NoSensorNoiseModel
from habitat_sim.sensors.noise_models.redwood_depth_noise_model import (
    RedwoodDepthNoiseModel,
)
from habitat_sim.sensors.noise_models.sensor_noise_model import SensorNoiseModel

#  from habitat_sim.registry import registry


def make_sensor_noise_model(name: str, kwargs) -> SensorNoiseModel:
    model = registry.get_noise_model(name)
    assert model is not None, "Could not find a noise model for name '{}'".format(name)

    return model(**kwargs)


__all__ = ["make_sensor_noise_model", "SensorNoiseModel"]
