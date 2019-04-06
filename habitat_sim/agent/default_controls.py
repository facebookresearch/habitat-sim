#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from .controls import register_move_fn, ActuationSpec
from habitat_sim import utils
import habitat_sim.bindings as hsim
import numpy as np

__all__ = []


_x_axis = 0
_y_axis = 1
_z_axis = 2


def _move_along(obj: hsim.SceneNode, distance: float, axis: int):
    ax = obj.absolute_transformation()[0:3, axis]
    obj.translate_local(ax * distance)


def _rotate_local(obj: hsim.SceneNode, theta: float, axis: int):
    ax = np.zeros(3, dtype=np.float32)
    ax[axis] = 1

    obj.rotate_local(np.deg2rad(theta), ax)
    obj.normalize()


@register_move_fn
def move_backward(obj: hsim.SceneNode, actuation_spec: ActuationSpec):
    _move_along(obj, actuation_spec.amount, _z_axis)


@register_move_fn
def move_forward(obj: hsim.SceneNode, actuation_spec: ActuationSpec):
    _move_along(obj, -actuation_spec.amount, _z_axis)


@register_move_fn
def move_right(obj: hsim.SceneNode, actuation_spec: ActuationSpec):
    _move_along(obj, actuation_spec.amount, _x_axis)


@register_move_fn
def move_left(obj: hsim.SceneNode, actuation_spec: ActuationSpec):
    _move_along(obj, -actuation_spec.amount, _x_axis)


@register_move_fn
def move_up(obj: hsim.SceneNode, actuation_spec: ActuationSpec):
    _move_along(obj, actuation_spec.amount, _y_axis)


@register_move_fn
def move_down(obj: hsim.SceneNode, actuation_spec: ActuationSpec):
    _move_along(obj, -actuation_spec.amount, _y_axis)


@register_move_fn
def look_left(obj: hsim.SceneNode, actuation_spec: ActuationSpec):
    _rotate_local(obj, actuation_spec.amount, _y_axis)


@register_move_fn
def look_right(obj: hsim.SceneNode, actuation_spec: ActuationSpec):
    _rotate_local(obj, -actuation_spec.amount, _y_axis)


register_move_fn(look_left, "turn_left")
register_move_fn(look_right, "turn_right")


@register_move_fn
def look_up(obj: hsim.SceneNode, actuation_spec: ActuationSpec):
    _rotate_local(obj, actuation_spec.amount, _x_axis)


@register_move_fn
def look_down(obj: hsim.SceneNode, actuation_spec: ActuationSpec):
    _rotate_local(obj, -actuation_spec.amount, _x_axis)
