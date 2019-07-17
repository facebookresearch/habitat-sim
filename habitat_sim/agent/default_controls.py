#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import magnum as mn
import numpy as np

import habitat_sim.bindings as hsim
from habitat_sim import utils

from .controls import ActuationSpec, SceneNodeControl, register_move_fn

__all__ = []


_x_axis = 0
_y_axis = 1
_z_axis = 2

_rotate_local_fns = [
    hsim.SceneNode.rotate_x_local,
    hsim.SceneNode.rotate_y_local,
    hsim.SceneNode.rotate_z_local,
]


def _move_along(scene_node: hsim.SceneNode, distance: float, axis: int):
    ax = scene_node.transformation[axis].xyz
    scene_node.translate_local(ax * distance)


def _rotate_local(scene_node: hsim.SceneNode, theta: float, axis: int):
    _rotate_local_fns[axis](scene_node, mn.Deg(theta))
    scene_node.rotation = scene_node.rotation.normalized()


@register_move_fn(body_action=True)
class MoveBackward(SceneNodeControl):
    def __call__(self, scene_node: hsim.SceneNode, actuation_spec: ActuationSpec):
        _move_along(scene_node, actuation_spec.amount, _z_axis)


@register_move_fn(body_action=True)
class MoveForward(SceneNodeControl):
    def __call__(self, scene_node: hsim.SceneNode, actuation_spec: ActuationSpec):
        _move_along(scene_node, -actuation_spec.amount, _z_axis)


@register_move_fn(body_action=True)
class MoveRight(SceneNodeControl):
    def __call__(self, scene_node: hsim.SceneNode, actuation_spec: ActuationSpec):
        _move_along(scene_node, actuation_spec.amount, _x_axis)


@register_move_fn(body_action=True)
class MoveLeft(SceneNodeControl):
    def __call__(self, scene_node: hsim.SceneNode, actuation_spec: ActuationSpec):
        _move_along(scene_node, -actuation_spec.amount, _x_axis)


@register_move_fn(body_action=False)
class MoveUp(SceneNodeControl):
    def __call__(self, scene_node: hsim.SceneNode, actuation_spec: ActuationSpec):
        _move_along(scene_node, actuation_spec.amount, _y_axis)


@register_move_fn(body_action=False)
class MoveDown(SceneNodeControl):
    def __call__(self, scene_node: hsim.SceneNode, actuation_spec: ActuationSpec):
        _move_along(scene_node, -actuation_spec.amount, _y_axis)


@register_move_fn(body_action=False)
class LookLeft(SceneNodeControl):
    def __call__(self, scene_node: hsim.SceneNode, actuation_spec: ActuationSpec):
        _rotate_local(scene_node, actuation_spec.amount, _y_axis)


@register_move_fn(body_action=False)
class LookRight(SceneNodeControl):
    def __call__(self, scene_node: hsim.SceneNode, actuation_spec: ActuationSpec):
        _rotate_local(scene_node, -actuation_spec.amount, _y_axis)


register_move_fn(LookLeft, name="turn_left", body_action=True)
register_move_fn(LookRight, name="turn_right", body_action=True)


@register_move_fn(body_action=False)
class LookUp(SceneNodeControl):
    def __call__(self, scene_node: hsim.SceneNode, actuation_spec: ActuationSpec):
        _rotate_local(scene_node, actuation_spec.amount, _x_axis)


@register_move_fn(body_action=False)
class LookDown(SceneNodeControl):
    def __call__(self, scene_node: hsim.SceneNode, actuation_spec: ActuationSpec):
        _rotate_local(scene_node, -actuation_spec.amount, _x_axis)
