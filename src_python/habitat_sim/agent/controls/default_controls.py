#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from typing import List, Optional

import magnum as mn
import numpy as np

from habitat_sim.agent.controls.controls import ActuationSpec, SceneNodeControl
from habitat_sim.geo import FRONT
from habitat_sim.registry import registry
from habitat_sim.scene import SceneNode

__all__: List[str] = []


_X_AXIS = 0
_Y_AXIS = 1
_Z_AXIS = 2

_rotate_local_fns = [
    SceneNode.rotate_x_local,
    SceneNode.rotate_y_local,
    SceneNode.rotate_z_local,
]


def _move_along(scene_node: SceneNode, distance: float, axis: int) -> None:
    ax = scene_node.transformation[axis].xyz
    scene_node.translate_local(ax * distance)


def _rotate_local(
    scene_node: SceneNode, theta: float, axis: int, constraint: Optional[float] = None
) -> None:
    if constraint is not None:
        rotation = scene_node.rotation

        if (
            abs(float(rotation.angle())) > 0
            and 1.0 - abs(rotation.axis().normalized()[axis]) > 1e-3
        ):
            raise RuntimeError(
                "Constrained look only works for a singular look action type"
            )

        look_vector = rotation.transform_vector(FRONT)
        if axis == 0:
            look_angle = mn.Rad(np.arctan2(look_vector[1], -look_vector[2]))
        elif axis == 1:
            look_angle = -mn.Rad(np.arctan2(look_vector[0], -look_vector[2]))

        new_angle = look_angle + mn.Deg(theta)

        constraint = mn.Deg(constraint)

        if new_angle > constraint:
            theta = constraint - look_angle
        elif new_angle < -constraint:
            theta = -constraint - look_angle

    _rotate_local_fns[axis](scene_node, mn.Deg(theta))
    scene_node.rotation = scene_node.rotation.normalized()


@registry.register_move_fn(body_action=True)
class MoveBackward(SceneNodeControl):
    def __call__(self, scene_node: SceneNode, actuation_spec: ActuationSpec) -> None:
        _move_along(scene_node, actuation_spec.amount, _Z_AXIS)


@registry.register_move_fn(body_action=True)
class MoveForward(SceneNodeControl):
    def __call__(self, scene_node: SceneNode, actuation_spec: ActuationSpec) -> None:
        _move_along(scene_node, -actuation_spec.amount, _Z_AXIS)


@registry.register_move_fn(body_action=True)
class MoveRight(SceneNodeControl):
    def __call__(self, scene_node: SceneNode, actuation_spec: ActuationSpec) -> None:
        _move_along(scene_node, actuation_spec.amount, _X_AXIS)


@registry.register_move_fn(body_action=True)
class MoveLeft(SceneNodeControl):
    def __call__(self, scene_node: SceneNode, actuation_spec: ActuationSpec) -> None:
        _move_along(scene_node, -actuation_spec.amount, _X_AXIS)


@registry.register_move_fn(body_action=False)
class MoveUp(SceneNodeControl):
    def __call__(self, scene_node: SceneNode, actuation_spec: ActuationSpec) -> None:
        _move_along(scene_node, actuation_spec.amount, _Y_AXIS)


@registry.register_move_fn(body_action=False)
class MoveDown(SceneNodeControl):
    def __call__(self, scene_node: SceneNode, actuation_spec: ActuationSpec) -> None:
        _move_along(scene_node, -actuation_spec.amount, _Y_AXIS)


@registry.register_move_fn(body_action=False)
class LookLeft(SceneNodeControl):
    def __call__(self, scene_node: SceneNode, actuation_spec: ActuationSpec) -> None:
        _rotate_local(
            scene_node, actuation_spec.amount, _Y_AXIS, actuation_spec.constraint
        )


@registry.register_move_fn(body_action=False)
class LookRight(SceneNodeControl):
    def __call__(self, scene_node: SceneNode, actuation_spec: ActuationSpec) -> None:
        _rotate_local(
            scene_node, -actuation_spec.amount, _Y_AXIS, actuation_spec.constraint
        )


registry.register_move_fn(LookLeft, name="turn_left", body_action=True)
registry.register_move_fn(LookRight, name="turn_right", body_action=True)


@registry.register_move_fn(body_action=False)
class LookUp(SceneNodeControl):
    def __call__(self, scene_node: SceneNode, actuation_spec: ActuationSpec) -> None:
        _rotate_local(
            scene_node, actuation_spec.amount, _X_AXIS, actuation_spec.constraint
        )


@registry.register_move_fn(body_action=False)
class LookDown(SceneNodeControl):
    def __call__(self, scene_node: SceneNode, actuation_spec: ActuationSpec) -> None:
        _rotate_local(
            scene_node, -actuation_spec.amount, _X_AXIS, actuation_spec.constraint
        )


@registry.register_move_fn(body_action=False)
class RotateSensorClockwise(SceneNodeControl):
    def __call__(self, scene_node: SceneNode, actuation_spec: ActuationSpec) -> None:
        _rotate_local(
            scene_node, actuation_spec.amount, _Z_AXIS, actuation_spec.constraint
        )


@registry.register_move_fn(body_action=False)
class RotateSensorAntiClockwise(SceneNodeControl):
    def __call__(self, scene_node: SceneNode, actuation_spec: ActuationSpec) -> None:
        _rotate_local(
            scene_node, -actuation_spec.amount, _Z_AXIS, actuation_spec.constraint
        )
