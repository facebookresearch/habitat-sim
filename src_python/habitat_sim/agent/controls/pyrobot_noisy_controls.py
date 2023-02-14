#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

r"""
Parameters contributed from PyRobot
https://pyrobot.org/
https://github.com/facebookresearch/pyrobot

Please cite PyRobot if you use this noise model
"""

from typing import Any, List, Optional, Sequence, Tuple

import attr
import magnum as mn
import numpy as np
import scipy.stats
from attr import Attribute
from numpy import ndarray

from habitat_sim import bindings as hsim
from habitat_sim.agent.controls.controls import ActuationSpec, SceneNodeControl
from habitat_sim.registry import registry


@attr.s(auto_attribs=True, init=False, slots=True)
class _TruncatedMultivariateGaussian:
    mean: np.ndarray
    cov: np.ndarray

    def __init__(self, mean: Sequence, cov: Sequence) -> None:
        self.mean = np.array(mean)
        self.cov = np.array(cov)
        if len(self.cov.shape) == 1:
            self.cov = np.diag(self.cov)

        assert (
            np.count_nonzero(self.cov - np.diag(np.diagonal(self.cov))) == 0
        ), "Only supports diagonal covariance"

    def sample(
        self,
        truncation: Optional[
            List[Optional[Tuple[Optional[Any], Optional[Any]]]]
        ] = None,
    ) -> ndarray:
        if truncation is not None:
            assert len(truncation) == len(self.mean)

        sample = np.zeros_like(self.mean)
        for i in range(len(self.mean)):
            stdev = np.sqrt(self.cov[i, i])
            mean = self.mean[i]
            # Always truncate to 3 standard deviations
            a, b = -3, 3

            if truncation is not None and truncation[i] is not None:
                trunc = truncation[i]
                if trunc[0] is not None:
                    a = max((trunc[0] - mean) / stdev, a)
                if trunc[1] is not None:
                    b = min((trunc[1] - mean) / stdev, b)

            sample[i] = scipy.stats.truncnorm.rvs(a, b, mean, stdev)

        return sample


@attr.s(auto_attribs=True, slots=True)
class MotionNoiseModel:
    linear: _TruncatedMultivariateGaussian
    rotation: _TruncatedMultivariateGaussian


@attr.s(auto_attribs=True, slots=True)
class ControllerNoiseModel:
    linear_motion: MotionNoiseModel
    rotational_motion: MotionNoiseModel


@attr.s(auto_attribs=True, slots=True)
class RobotNoiseModel:
    ILQR: ControllerNoiseModel
    Proportional: ControllerNoiseModel
    Movebase: ControllerNoiseModel

    def __getitem__(self, key: str) -> ControllerNoiseModel:
        return getattr(self, key)


pyrobot_noise_models = {
    "LoCoBot": RobotNoiseModel(
        ILQR=ControllerNoiseModel(
            linear_motion=MotionNoiseModel(
                _TruncatedMultivariateGaussian([0.014, 0.009], [0.006, 0.005]),
                _TruncatedMultivariateGaussian([0.008], [0.004]),
            ),
            rotational_motion=MotionNoiseModel(
                _TruncatedMultivariateGaussian([0.003, 0.003], [0.002, 0.003]),
                _TruncatedMultivariateGaussian([0.023], [0.012]),
            ),
        ),
        Proportional=ControllerNoiseModel(
            linear_motion=MotionNoiseModel(
                _TruncatedMultivariateGaussian([0.017, 0.042], [0.007, 0.023]),
                _TruncatedMultivariateGaussian([0.031], [0.026]),
            ),
            rotational_motion=MotionNoiseModel(
                _TruncatedMultivariateGaussian([0.001, 0.005], [0.001, 0.004]),
                _TruncatedMultivariateGaussian([0.043], [0.017]),
            ),
        ),
        Movebase=ControllerNoiseModel(
            linear_motion=MotionNoiseModel(
                _TruncatedMultivariateGaussian([0.074, 0.036], [0.019, 0.033]),
                _TruncatedMultivariateGaussian([0.189], [0.038]),
            ),
            rotational_motion=MotionNoiseModel(
                _TruncatedMultivariateGaussian([0.002, 0.003], [0.0, 0.002]),
                _TruncatedMultivariateGaussian([0.219], [0.019]),
            ),
        ),
    ),
    "LoCoBot-Lite": RobotNoiseModel(
        ILQR=ControllerNoiseModel(
            linear_motion=MotionNoiseModel(
                _TruncatedMultivariateGaussian([0.142, 0.023], [0.008, 0.008]),
                _TruncatedMultivariateGaussian([0.031], [0.028]),
            ),
            rotational_motion=MotionNoiseModel(
                _TruncatedMultivariateGaussian([0.002, 0.002], [0.001, 0.002]),
                _TruncatedMultivariateGaussian([0.122], [0.03]),
            ),
        ),
        Proportional=ControllerNoiseModel(
            linear_motion=MotionNoiseModel(
                _TruncatedMultivariateGaussian([0.135, 0.043], [0.007, 0.009]),
                _TruncatedMultivariateGaussian([0.049], [0.009]),
            ),
            rotational_motion=MotionNoiseModel(
                _TruncatedMultivariateGaussian([0.002, 0.002], [0.002, 0.001]),
                _TruncatedMultivariateGaussian([0.054], [0.061]),
            ),
        ),
        Movebase=ControllerNoiseModel(
            linear_motion=MotionNoiseModel(
                _TruncatedMultivariateGaussian([0.192, 0.117], [0.055, 0.144]),
                _TruncatedMultivariateGaussian([0.128], [0.143]),
            ),
            rotational_motion=MotionNoiseModel(
                _TruncatedMultivariateGaussian([0.002, 0.001], [0.001, 0.001]),
                _TruncatedMultivariateGaussian([0.173], [0.025]),
            ),
        ),
    ),
}


@attr.s(auto_attribs=True)
class PyRobotNoisyActuationSpec(ActuationSpec):
    r"""Struct to hold parameters for pyrobot noise model

    :property robot: Which robot to simulate noise for. Valid values are
        ``LoCoBot`` and ``LoCoBot-Lite``.
    :property controller: Which controller to simulate noise models for. Valid
        values are ``ILQR``, ``Proportional``; ``Movebase ILQR`` is the
        default.
    :property noise_multiplier: Multiplier on the noise amount, useful for
        ablating the effect of noise.

    -   https://pyrobot.org/
    -   https://github.com/facebookresearch/pyrobot

    Please cite PyRobot if you use this noise model.
    """
    robot: str = attr.ib(default="LoCoBot")

    @robot.validator
    def check_robot(self, attribute: Attribute, value: str) -> None:
        assert value in pyrobot_noise_models, f"{value} not a known robot"

    controller: str = attr.ib(default="ILQR")

    @controller.validator
    def check_controller(self, attribute: Attribute, value: str) -> None:
        assert value in [
            "ILQR",
            "Proportional",
            "Movebase",
        ], f"{value} not a known controller"

    noise_multiplier: float = 1.0


_X_AXIS = 0
_Y_AXIS = 1
_Z_AXIS = 2


def _noisy_action_impl(
    scene_node: hsim.SceneNode,
    translate_amount: float,
    rotate_amount: float,
    multiplier: float,
    model: MotionNoiseModel,
    motion_type: str,
) -> None:
    # Perform the action in the coordinate system of the node
    transform = scene_node.transformation
    move_ax = -transform[_Z_AXIS].xyz
    perp_ax = transform[_X_AXIS].xyz

    if motion_type == "rotational":
        translation_noise = multiplier * model.linear.sample()
    else:
        # The robot will always move a little bit.  This has to be defined based on the intended actuation
        # as otherwise small rotation amounts would be invalid.  However, pretty quickly, we'll
        # get to the truncation of 3 sigma
        trunc = [(-0.95 * np.abs(translate_amount), None), None]

        translation_noise = multiplier * model.linear.sample(trunc)

    # + EPS to make sure 0 is positive.  We multiply by the sign of the translation
    # as otherwise forward would overshoot on average and backward would undershoot, while
    # both should overshoot
    translation_noise *= np.sign(translate_amount + 1e-8)

    scene_node.translate_local(
        move_ax * (translate_amount + translation_noise[0])
        + perp_ax * translation_noise[1]
    )

    if motion_type == "linear":
        rot_noise = multiplier * model.rotation.sample()
    else:
        # The robot will always turn a little bit.  This has to be defined based on the intended actuation
        # as otherwise small rotation amounts would be invalid.  However, pretty quickly, we'll
        # get to the truncation of 3 sigma
        trunc = [(-0.95 * np.abs(np.deg2rad(rotate_amount)), None)]

        rot_noise = multiplier * model.rotation.sample(trunc)

    # Same deal with rotation about + EPS and why we multiply by the sign
    rot_noise *= np.sign(rotate_amount + 1e-8)

    scene_node.rotate_y_local(mn.Deg(rotate_amount) + mn.Rad(rot_noise))
    scene_node.rotation = scene_node.rotation.normalized()


@registry.register_move_fn(body_action=True)
class PyrobotNoisyMoveBackward(SceneNodeControl):
    def __call__(
        self, scene_node: hsim.SceneNode, actuation_spec: ActuationSpec
    ) -> None:
        assert isinstance(actuation_spec, PyRobotNoisyActuationSpec)
        _noisy_action_impl(
            scene_node,
            -actuation_spec.amount,
            0.0,
            actuation_spec.noise_multiplier,
            pyrobot_noise_models[actuation_spec.robot][
                actuation_spec.controller
            ].linear_motion,
            "linear",
        )


@registry.register_move_fn(body_action=True)
class PyrobotNoisyMoveForward(SceneNodeControl):
    def __call__(
        self, scene_node: hsim.SceneNode, actuation_spec: ActuationSpec
    ) -> None:
        assert isinstance(actuation_spec, PyRobotNoisyActuationSpec)
        _noisy_action_impl(
            scene_node,
            actuation_spec.amount,
            0.0,
            actuation_spec.noise_multiplier,
            pyrobot_noise_models[actuation_spec.robot][
                actuation_spec.controller
            ].linear_motion,
            "linear",
        )


@registry.register_move_fn(body_action=True)
class PyrobotNoisyTurnLeft(SceneNodeControl):
    def __call__(
        self, scene_node: hsim.SceneNode, actuation_spec: ActuationSpec
    ) -> None:
        assert isinstance(actuation_spec, PyRobotNoisyActuationSpec)
        _noisy_action_impl(
            scene_node,
            0.0,
            actuation_spec.amount,
            actuation_spec.noise_multiplier,
            pyrobot_noise_models[actuation_spec.robot][
                actuation_spec.controller
            ].rotational_motion,
            "rotational",
        )


@registry.register_move_fn(body_action=True)
class PyrobotNoisyTurnRight(SceneNodeControl):
    def __call__(
        self, scene_node: hsim.SceneNode, actuation_spec: ActuationSpec
    ) -> None:
        assert isinstance(actuation_spec, PyRobotNoisyActuationSpec)
        _noisy_action_impl(
            scene_node,
            0.0,
            -actuation_spec.amount,
            actuation_spec.noise_multiplier,
            pyrobot_noise_models[actuation_spec.robot][
                actuation_spec.controller
            ].rotational_motion,
            "rotational",
        )
