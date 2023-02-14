#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import collections
import re
from typing import DefaultDict, Optional, Type

__all__ = ["registry"]


def _camel_to_snake(name):
    s1 = re.sub("(.)([A-Z][a-z]+)", r"\1_\2", name)
    return re.sub("([a-z0-9])([A-Z])", r"\1_\2", s1).lower()


class _Registry:
    r"""registry is a central source of truth in Habitat-Sim

    Taken from Pythia, it is inspired from Redux's
    concept of global store. registry maintains mappings of various information
    to unique keys. Special functions in registry can be used as decorators to
    register different kind of classes.

    Import the global registry object using ``from habitat_sim import registry``.
    Then use various decorators for registering
    different kind of classes with unique keys

    - Register a movement function : ``@registry.register_move_fn``
    """
    _mapping: DefaultDict[str, dict] = collections.defaultdict(dict)

    @classmethod
    def register_move_fn(
        cls,
        controller: Optional[Type] = None,
        *,
        name: Optional[str] = None,
        body_action: Optional[bool] = None,
    ):
        r"""Registers a new control with Habitat-Sim. Registered controls can
        then be retrieved via `get_move_fn()`

        See `new-actions <new-actions.html>`_ for an example of how to add new actions
        *outside* the core habitat_sim package.

        :param controller: The class of the controller to register. Must inherit from `agent.SceneNodeControl`.
            If :py:`None`, will return a wrapper for use with decorator syntax.
        :param name: The name to register the control with. If :py:`None`, will
            register with the name of the controller converted to snake case,
            i.e. a controller with class name ``MoveForward`` will be registered as
            ``move_forward``.
        :param body_action: Whether or not this action manipulates the agent's body
            (thereby also moving the sensors) or manipulates just the sensors.
            This is a non-optional keyword arguement and must be set (this is done
            for readability).
        """
        assert (
            body_action is not None
        ), "body_action must be explicitly set to True or False"
        from habitat_sim.agent.controls.controls import SceneNodeControl

        def _wrapper(controller: Type[SceneNodeControl]):
            assert issubclass(
                controller, SceneNodeControl
            ), "All controls must inherit from habitat_sim.agent.SceneNodeControl"

            cls._mapping["move_fn"][
                _camel_to_snake(controller.__name__) if name is None else name
            ] = controller(body_action)

            return controller

        if controller is None:
            return _wrapper

        return _wrapper(controller)

    @classmethod
    def register_noise_model(
        cls, noise_model: Optional[Type] = None, *, name: Optional[str] = None
    ):
        r"""Registers a new sensor noise model with Habitat-Sim

        :param noise_model: The class of the noise model to register
            If `None`, will return a wrapper for use with decorator syntax
        :param name: The name to register the noise model with
            If `None`, will register with the name of the noise_model
        """
        from habitat_sim.sensors.noise_models.sensor_noise_model import SensorNoiseModel

        def _wrapper(noise_model: Type[SensorNoiseModel]):
            assert issubclass(
                noise_model, SensorNoiseModel
            ), "All noise_models must inherit from habitat_sim.sensor.SensorNoiseModel"

            cls._mapping["sensor_noise_model"][
                noise_model.__name__ if name is None else name
            ] = noise_model

            return noise_model

        if noise_model is None:
            return _wrapper

        return _wrapper(noise_model)

    @classmethod
    def register_pose_extractor(
        cls, pose_extractor: Optional[Type] = None, *, name: Optional[str] = None
    ):
        r"""Registers a new pose extractor model with Habitat-Sim

        :param pose_extractor: The class of the pose extractor to register
            If `None`, will return a wrapper for use with decorator syntax
        :param name: The name to register the noise model with
            If `None`, will register with the name of the pose_extractor
        """
        from habitat_sim.utils.data.pose_extractor import PoseExtractor

        def _wrapper(pose_extractor: Type[PoseExtractor]):
            assert issubclass(
                pose_extractor, PoseExtractor
            ), "All pose_extractors must inherit from habitat_sim.utils.data.PoseExtractor"

            cls._mapping["pose_extractor"][
                pose_extractor.__name__ if name is None else name
            ] = pose_extractor

            return pose_extractor

        if pose_extractor is None:
            return _wrapper

        return _wrapper(pose_extractor)

    @classmethod
    def _get_impl(cls, _type, name: str):
        return cls._mapping[_type].get(name, None)

    @classmethod
    def get_move_fn(cls, name: str):
        r"""Retrieve the move_fn register under ``name``

        :param name: The name provided to `register_move_fn`
        """
        return cls._get_impl("move_fn", name)

    @classmethod
    def get_noise_model(cls, name: str):
        r"""Retrieve the noise_model registered under ``name``

        :param name: The name provided to `register_noise_model`
        """
        return cls._get_impl("sensor_noise_model", name)

    @classmethod
    def get_pose_extractor(cls, name: str):
        r"""Retrieve the pose_extractor registered under ``name``

        :param name: The name provided to `register_pose_extractor`
        """
        return cls._get_impl("pose_extractor", name)


registry = _Registry()
