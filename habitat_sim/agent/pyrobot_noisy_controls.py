#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


import attr
import numpy as np

import habitat_sim.bindings as hsim
from habitat_sim import utils

from .controls import ActuationSpec, SceneNodeControl, register_move_fn

__all__ = ["PyRobotNoisyActuationSpec"]


@attr.s(auto_attribs=True)
class _MeanStdev:
    mean: np.array
    cov: np.array


@attr.s(auto_attribs=True)
class NoiseModel:
    linear: _MeanStdev
    rotation: _MeanStdev


@attr.s(auto_attribs=True)
class ControllerNoiseModel:
    linear_motion: NoiseModel
    rotational_motion: NoiseModel


@attr.s(auto_attribs=True)
class RobotNoiseModel:
    ILQR: ControllerNoiseModel

    def __getitem__(self, key):
        return getattr(self, key, None)


noise_models = {
    "LoCoBot": RobotNoiseModel(
        ILQR=ControllerNoiseModel(
            linear_motion=NoiseModel(
                _MeanStdev(np.sqrt(17) * np.ones((2,)) / 1e3, np.eye(2) * 5 / 1e3),
                _MeanStdev(0.43 * np.ones((1,)), np.eye(1) * 0.25),
            ),
            rotational_motion=NoiseModel(
                _MeanStdev(np.sqrt(6) * np.ones((2,)) / 1e3, np.eye(2) * 0),
                _MeanStdev(1.32 * np.ones((1,)), np.eye(1) * 0.68),
            ),
        )
    ),
    "LoCoBot-Lite": RobotNoiseModel(
        ILQR=ControllerNoiseModel(
            linear_motion=NoiseModel(
                _MeanStdev(np.sqrt(144) * np.ones((2,)) / 1e3, np.eye(2) * 8 / 1e3),
                _MeanStdev(1.79 * np.ones((1,)), np.eye(1) * 1.59),
            ),
            rotational_motion=NoiseModel(
                _MeanStdev(np.sqrt(3) * np.ones((2,)) / 1e3, np.eye(2) * 2 / 1e3),
                _MeanStdev(6.97 * np.ones((1,)), np.eye(1) * 1.71),
            ),
        )
    ),
}


@attr.s(auto_attribs=True)
class PyRobotNoisyActuationSpec(ActuationSpec):
    r"""Struct to hold parameters for pyrobot noise model

    Args:
        amount (float): The amount the control moves the scene node by
        robot (str): Which robot to simulate noise for.  Valid values
            are LoCoBot and LoCoBot-Lite
        controller (str): Which controller to simulate noise models for,
            Valid values are ILQR, Proportional, Movebase
            ILQR is the default (as that is the best)
        noise_multiplier (float): Multiplier on the noise amount,
            useful for ablating the effect of noise
    """
    robot: str = attr.ib(default="LoCoBot")

    @robot.validator
    def check(self, attribute, value):
        assert value in noise_models.keys()

    controller: str = attr.ib(default="ILQR")

    @controller.validator
    def check(self, attribute, value):
        assert value in ["ILQR"]

    noise_multiplier: float = 1.0


_x_axis = 0
_y_axis = 1
_z_axis = 2


def _noisy_action(
    scene_node: hsim.SceneNode,
    translate_amount: float,
    rotate_amount: float,
    multiplier: float,
    model: NoiseModel,
):
    ax = scene_node.absolute_transformation()[0:3, _z_axis]
    prep_ax = np.cross(ax, hsim.geo.UP)

    translation_noise = multiplier * np.random.multivariate_normal(
        model.linear.mean, model.linear.cov
    )
    scene_node.translate_local(
        ax * (translate_amount + translation_noise[0]) + prep_ax * translation_noise[1]
    )

    rot_noise = multiplier * np.random.multivariate_normal(
        model.rotation.mean, model.rotation.cov
    )

    ax = np.zeros(3, dtype=np.float32)
    ax[_y_axis] = 1

    scene_node.rotate_local(np.deg2rad(rotate_amount + rot_noise[0]), ax)
    scene_node.normalize()


@register_move_fn(body_action=True)
class PyrobotNoisyMoveBackward(SceneNodeControl):
    def __call__(
        self, scene_node: hsim.SceneNode, actuation_spec: PyRobotNoisyActuationSpec
    ):
        _noisy_action(
            scene_node,
            actuation_spec.amount,
            0.0,
            actuation_spec.noise_multiplier,
            noise_models[actuation_spec.robot][actuation_spec.controller].linear_motion,
        )


@register_move_fn(body_action=True)
class PyrobotNoisyMoveForward(SceneNodeControl):
    def __call__(
        self, scene_node: hsim.SceneNode, actuation_spec: PyRobotNoisyActuationSpec
    ):
        _noisy_action(
            scene_node,
            -actuation_spec.amount,
            0.0,
            actuation_spec.noise_multiplier,
            noise_models[actuation_spec.robot][actuation_spec.controller].linear_motion,
        )


@register_move_fn(body_action=True)
class PyrobotNoisyTurnLeft(SceneNodeControl):
    def __call__(
        self, scene_node: hsim.SceneNode, actuation_spec: PyRobotNoisyActuationSpec
    ):
        _noisy_action(
            scene_node,
            0.0,
            actuation_spec.amount,
            actuation_spec.noise_multiplier,
            noise_models[actuation_spec.robot][
                actuation_spec.controller
            ].rotational_motion,
        )


@register_move_fn(body_action=True)
class PyrobotNoisyTurnRight(SceneNodeControl):
    def __call__(
        self, scene_node: hsim.SceneNode, actuation_spec: PyRobotNoisyActuationSpec
    ):
        _noisy_action(
            scene_node,
            0.0,
            -actuation_spec.amount,
            actuation_spec.noise_multiplier,
            noise_models[actuation_spec.robot][
                actuation_spec.controller
            ].rotational_motion,
        )
