#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import abc
from typing import Dict, Optional, Type


class SensorNoiseModel(abc.ABC):
    def __init__(self, gpu_device_id: Optional[int] = None):
        self._gpu_device_id = gpu_device_id

    @staticmethod
    @abc.abstractmethod
    def is_valid_sensor_type(sensor_type):
        pass

    @abc.abstractmethod
    def apply(self, x):
        pass

    def __call__(self, x):
        return self.apply(x)
