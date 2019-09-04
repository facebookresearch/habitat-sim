import abc
from typing import Dict, Optional, Type

import numpy as np


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


_noise_model_map: Dict[str, SensorNoiseModel] = dict()


def register_sensor_noise_model(
    noise_model: Optional[Type[SensorNoiseModel]] = None, *, name: Optional[str] = None
):
    r"""Registers a new sensor noise model with Habitat-Sim

    Args:
        noise_model: The class of the noise model to register
            If none, will return a wrapper for use with decorator syntax
        name: The name to register the noise model with
            If none, will register with the name of the controller converted to snake case
            i.e. a controller with class name MoveForward will be registered as move_forward
    """

    def _wrapper(noise_model: Type[SensorNoiseModel]):
        assert issubclass(
            noise_model, SensorNoiseModel
        ), "All noise models must inherit from habitat_sim.sensors.SensorNoiseModel"

        _noise_model_map[noise_model.__name__ if name is None else name] = noise_model

        return noise_model

    if noise_model is None:
        return _wrapper
    else:
        return _wrapper(noise_model)


def make_sensor_noise_model(name: str, kwargs) -> SensorNoiseModel:
    assert (
        name in _noise_model_map
    ), "Could not find a noise model for name '{}'".format(name)

    return _noise_model_map[name](**kwargs)


class NoSensorNoiseModel(SensorNoiseModel):
    @staticmethod
    def is_valid_sensor_type(sensor_type):
        return True

    def apply(self, x):
        if isinstance(x, np.ndarray):
            return x.copy()
        else:
            return x.clone()


register_sensor_noise_model(NoSensorNoiseModel, name="None")
