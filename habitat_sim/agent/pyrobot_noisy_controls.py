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
class _MultivariateGaussian:
    mean: np.array
    cov: np.array

    def __attrs_post_init__(self):
        self.mean = np.array(self.mean)
        self.cov = np.array(self.cov)
        if len(self.cov.shape) == 1:
            self.cov = np.diag(self.cov)


@attr.s(auto_attribs=True)
class MotionNoiseModel:
    linear: _MultivariateGaussian
    rotation: _MultivariateGaussian


@attr.s(auto_attribs=True)
class ControllerNoiseModel:
    linear_motion: MotionNoiseModel
    rotational_motion: MotionNoiseModel


@attr.s(auto_attribs=True)
class RobotNoiseModel:
    ILQR: ControllerNoiseModel
    Proportional: ControllerNoiseModel
    Movebase: ControllerNoiseModel

    def __getitem__(self, key):
        return getattr(self, key)


noise_models = {
    "LoCoBot": RobotNoiseModel(
        ILQR=ControllerNoiseModel(
            linear_motion=MotionNoiseModel(
                _MultivariateGaussian([0.014, 0.009], [0.006, 0.005]),
                _MultivariateGaussian([0.008], [0.004]),
            ),
            rotational_motion=MotionNoiseModel(
                _MultivariateGaussian([0.003, 0.003], [0.002, 0.003]),
                _MultivariateGaussian([0.023], [0.012]),
            ),
        ),
        Proportional=ControllerNoiseModel(
            linear_motion=MotionNoiseModel(
                _MultivariateGaussian([0.017, 0.042], [0.007, 0.023]),
                _MultivariateGaussian([0.031], [0.026]),
            ),
            rotational_motion=MotionNoiseModel(
                _MultivariateGaussian([0.001, 0.005], [0.001, 0.004]),
                _MultivariateGaussian([0.043], [0.017]),
            ),
        ),
        Movebase=ControllerNoiseModel(
            linear_motion=MotionNoiseModel(
                _MultivariateGaussian([0.074, 0.036], [0.019, 0.033]),
                _MultivariateGaussian([0.189], [0.038]),
            ),
            rotational_motion=MotionNoiseModel(
                _MultivariateGaussian([0.002, 0.003], [0.0, 0.002]),
                _MultivariateGaussian([0.219], [0.019]),
            ),
        ),
    ),
    "LoCoBot-Lite": RobotNoiseModel(
        ILQR=ControllerNoiseModel(
            linear_motion=MotionNoiseModel(
                _MultivariateGaussian([0.142, 0.023], [0.008, 0.008]),
                _MultivariateGaussian([0.031], [0.028]),
            ),
            rotational_motion=MotionNoiseModel(
                _MultivariateGaussian([0.002, 0.002], [0.001, 0.002]),
                _MultivariateGaussian([0.122], [0.03]),
            ),
        ),
        Proportional=ControllerNoiseModel(
            linear_motion=MotionNoiseModel(
                _MultivariateGaussian([0.135, 0.043], [0.007, 0.009]),
                _MultivariateGaussian([0.049], [0.009]),
            ),
            rotational_motion=MotionNoiseModel(
                _MultivariateGaussian([0.002, 0.002], [0.002, 0.001]),
                _MultivariateGaussian([0.054], [0.061]),
            ),
        ),
        Movebase=ControllerNoiseModel(
            linear_motion=MotionNoiseModel(
                _MultivariateGaussian([0.192, 0.117], [0.055, 0.144]),
                _MultivariateGaussian([0.128], [0.143]),
            ),
            rotational_motion=MotionNoiseModel(
                _MultivariateGaussian([0.002, 0.001], [0.001, 0.001]),
                _MultivariateGaussian([0.173], [0.025]),
            ),
        ),
    ),
}


@attr.s(auto_attribs=True)
class PyRobotNoisyActuationSpec(ActuationSpec):
    r"""Struct to hold parameters for pyrobot noise model

    https://pyrobot.org/
    https://github.com/facebookresearch/pyrobot

    Please cite PyRobot if you use this noise model

    Args:
        amount (float): The amount the control moves the scene node by
        robot (str): Which robot to simulate noise for.  Valid values
            are LoCoBot and LoCoBot-Lite
        controller (str): Which controller to simulate noise models for,
            Valid values are ILQR, Proportional, Movebase
            ILQR is the default
        noise_multiplier (float): Multiplier on the noise amount,
            useful for ablating the effect of noise
    """
    robot: str = attr.ib(default="LoCoBot")

    @robot.validator
    def check(self, attribute, value):
        assert value in noise_models.keys(), f"{value} not a known robot"

    controller: str = attr.ib(default="ILQR")

    @controller.validator
    def check(self, attribute, value):
        assert value in [
            "ILQR",
            "Proportional",
            "Movebase",
        ], f"{value} not a known controller"

    noise_multiplier: float = 1.0


_x_axis = 0
_y_axis = 1
_z_axis = 2


def _noisy_action(
    scene_node: hsim.SceneNode,
    translate_amount: float,
    rotate_amount: float,
    multiplier: float,
    model: MotionNoiseModel,
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

    scene_node.rotate_local(np.deg2rad(rotate_amount) + rot_noise[0], ax)
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
