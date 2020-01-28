#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from typing import Union

import magnum as mn
import numpy as np

from habitat_sim.agent.controls.controls import (
    ActuationSpec,
    ConstrainedActuationSpec,
    SceneNodeControl,
)
from habitat_sim.registry import registry
from habitat_sim.scene import SceneNode

__all__ = []


_X_AXIS = 0
_Y_AXIS = 1
_Z_AXIS = 2

_rotate_local_fns = [
    SceneNode.rotate_x_local,
    SceneNode.rotate_y_local,
    SceneNode.rotate_z_local,
]


def _move_along(scene_node: SceneNode, distance: float, axis: int):
    ax = scene_node.transformation[axis].xyz
    scene_node.translate_local(ax * distance)


def _rotate_local(scene_node: SceneNode, theta: float, axis: int):
    _rotate_local_fns[axis](scene_node, mn.Deg(theta))
    scene_node.rotation = scene_node.rotation.normalized()


def _apply_look_constraint(scene_node: SceneNode, constraint: float):
    constraint = mn.Deg(constraint)
    rotation = scene_node.rotation
    look_axis = rotation.transform_vector(mn.Vector3(0, 0, -1))
    look_angle = mn.Rad(np.arctan2(look_axis[1], -look_axis[2]))

    if look_angle > constraint:
        _rotate_local(scene_node, constraint - look_angle, _X_AXIS)
    elif look_angle < -constraint:
        _rotate_local(scene_node, -look_angle - constraint, _X_AXIS)


@registry.register_move_fn(body_action=True)
class MoveBackward(SceneNodeControl):
    def __call__(self, scene_node: SceneNode, actuation_spec: ActuationSpec):
        _move_along(scene_node, actuation_spec.amount, _Z_AXIS)


@registry.register_move_fn(body_action=True)
class MoveForward(SceneNodeControl):
    def __call__(self, scene_node: SceneNode, actuation_spec: ActuationSpec):
        _move_along(scene_node, -actuation_spec.amount, _Z_AXIS)


@registry.register_move_fn(body_action=True)
class MoveRight(SceneNodeControl):
    def __call__(self, scene_node: SceneNode, actuation_spec: ActuationSpec):
        _move_along(scene_node, actuation_spec.amount, _X_AXIS)


@registry.register_move_fn(body_action=True)
class MoveLeft(SceneNodeControl):
    def __call__(self, scene_node: SceneNode, actuation_spec: ActuationSpec):
        _move_along(scene_node, -actuation_spec.amount, _X_AXIS)


@registry.register_move_fn(body_action=False)
class MoveUp(SceneNodeControl):
    def __call__(self, scene_node: SceneNode, actuation_spec: ActuationSpec):
        _move_along(scene_node, actuation_spec.amount, _Y_AXIS)


@registry.register_move_fn(body_action=False)
class MoveDown(SceneNodeControl):
    def __call__(self, scene_node: SceneNode, actuation_spec: ActuationSpec):
        _move_along(scene_node, -actuation_spec.amount, _Y_AXIS)


@registry.register_move_fn(body_action=False)
class LookLeft(SceneNodeControl):
    def __call__(self, scene_node: SceneNode, actuation_spec: ActuationSpec):
        _rotate_local(scene_node, actuation_spec.amount, _Y_AXIS)


@registry.register_move_fn(body_action=False)
class LookRight(SceneNodeControl):
    def __call__(self, scene_node: SceneNode, actuation_spec: ActuationSpec):
        _rotate_local(scene_node, -actuation_spec.amount, _Y_AXIS)


registry.register_move_fn(LookLeft, name="turn_left", body_action=True)
registry.register_move_fn(LookRight, name="turn_right", body_action=True)


@registry.register_move_fn(body_action=False)
class LookUp(SceneNodeControl):
    def __call__(
        self,
        scene_node: SceneNode,
        actuation_spec: Union[ActuationSpec, ConstrainedActuationSpec],
    ):
        _rotate_local(scene_node, actuation_spec.amount, _X_AXIS)

        if isinstance(actuation_spec, ConstrainedActuationSpec):
            _apply_look_constraint(scene_node, actuation_spec.constraint)


@registry.register_move_fn(body_action=False)
class LookDown(SceneNodeControl):
    def __call__(
        self,
        scene_node: SceneNode,
        actuation_spec: Union[ActuationSpec, ConstrainedActuationSpec],
    ):
        _rotate_local(scene_node, -actuation_spec.amount, _X_AXIS)

        if isinstance(actuation_spec, ConstrainedActuationSpec):
            _apply_look_constraint(scene_node, actuation_spec.constraint)
