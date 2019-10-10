#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import abc
from typing import Dict, Optional, Type

import attr

from habitat_sim.sensor import SensorType


@attr.s(auto_attribs=True, kw_only=True)
class SensorNoiseModel(abc.ABC):
    r"""Base class for all sensor noise models

    :param gpu_device_id: The ID of the CUDA device to use (only applicable to
    noise models that proivde a CUDA implementation and, generally, if habitat-sim was build with ``--with-cuda``)
    """
    gpu_device_id: Optional[int] = None

    @staticmethod
    @abc.abstractmethod
    def is_valid_sensor_type(sensor_type: SensorType) -> bool:
        r"""Used to determine whether or not the noise model
        is applicable to the sensor type

        :return: True if this noise model can be applied to this sensor input type
        """
        pass

    @abc.abstractmethod
    def apply(self, sensor_observation):
        r"""Applies the noise model to the sensor observation

        :param sensor_observation: The clean sensor observation.
            Should not be modified.

        :return: The sensor observation with noise applied.
        """
        pass

    def __call__(self, sensor_observation):
        r"""Alias of `apply()`
        """
        return self.apply(sensor_observation)
