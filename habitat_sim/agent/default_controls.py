#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import numpy as np

import habitat_sim.bindings as hsim
from habitat_sim import utils

from .controls import ActuationSpec, Controller, register_move_fn

__all__ = []


_x_axis = 0
_y_axis = 1
_z_axis = 2


def _move_along(scene_node: hsim.SceneNode, distance: float, axis: int):
    ax = scene_node.absolute_transformation()[0:3, axis]
    scene_node.translate_local(ax * distance)


def _rotate_local(scene_node: hsim.SceneNode, theta: float, axis: int):
    ax = np.zeros(3, dtype=np.float32)
    ax[axis] = 1

    scene_node.rotate_local(np.deg2rad(theta), ax)
    scene_node.normalize()


@register_move_fn(body_action=True)
class MoveBackward(Controller):
    def __call__(self, scene_node: hsim.SceneNode, actuation_spec: ActuationSpec):
        _move_along(scene_node, actuation_spec.amount, _z_axis)


@register_move_fn(body_action=True)
class MoveForward(Controller):
    def __call__(self, scene_node: hsim.SceneNode, actuation_spec: ActuationSpec):
        _move_along(scene_node, -actuation_spec.amount, _z_axis)


@register_move_fn(body_action=True)
class MoveRight(Controller):
    def __call__(self, scene_node: hsim.SceneNode, actuation_spec: ActuationSpec):
        _move_along(scene_node, actuation_spec.amount, _x_axis)


@register_move_fn(body_action=True)
class MoveLeft(Controller):
    def __call__(self, scene_node: hsim.SceneNode, actuation_spec: ActuationSpec):
        _move_along(scene_node, -actuation_spec.amount, _x_axis)


@register_move_fn
class MoveUp(Controller):
    def __call__(self, scene_node: hsim.SceneNode, actuation_spec: ActuationSpec):
        _move_along(scene_node, actuation_spec.amount, _y_axis)


@register_move_fn
class MoveDown(Controller):
    def __call__(self, scene_node: hsim.SceneNode, actuation_spec: ActuationSpec):
        _move_along(scene_node, -actuation_spec.amount, _y_axis)


@register_move_fn
class LookLeft(Controller):
    def __call__(self, scene_node: hsim.SceneNode, actuation_spec: ActuationSpec):
        _rotate_local(scene_node, actuation_spec.amount, _y_axis)


@register_move_fn
class LookRight(Controller):
    def __call__(self, scene_node: hsim.SceneNode, actuation_spec: ActuationSpec):
        _rotate_local(scene_node, -actuation_spec.amount, _y_axis)


register_move_fn(LookLeft, name="turn_left", body_action=True)
register_move_fn(LookRight, name="turn_right", body_action=True)


@register_move_fn
class LookUp(Controller):
    def __call__(self, scene_node: hsim.SceneNode, actuation_spec: ActuationSpec):
        _rotate_local(scene_node, actuation_spec.amount, _x_axis)


@register_move_fn
class LookDown(Controller):
    def __call__(self, scene_node: hsim.SceneNode, actuation_spec: ActuationSpec):
        _rotate_local(scene_node, -actuation_spec.amount, _x_axis)
