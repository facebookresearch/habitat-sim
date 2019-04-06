#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import attr
import numpy as np
import quaternion
import habitat_sim.bindings as hsim
from typing import Dict
from habitat_sim import utils

__all__ = ["ActuationSpec", "ObjectControls"]


@attr.s(auto_attribs=True, slots=True)
class ActuationSpec(object):
    amount: float


move_func_map = dict()


def register_move_fn(fn, name=None):
    move_func_map[fn.__name__ if name is None else name] = fn
    return fn


def _noop_filter(start: np.array, end: np.array):
    return end


@attr.s
class ObjectControls(object):
    r"""Used to implement actions

    Args:
        move_filter_fn: A function that is applied after actions to handle collisions
            This should generally be `try_step` or the default
    """

    move_filter_fn = attr.ib(default=_noop_filter)

    def action(
        self,
        obj: hsim.SceneNode,
        action_name: str,
        actuation_spec: ActuationSpec,
        apply_filter: bool = True,
    ):
        r"""Performs the action specified by :py:attr:`action_name` on the object

        Args:
            obj (hsim.SceneNode): SceneNode to perform the action on
            action_name (str): Name of the action.  Used to index the move_func_map
                to retrieve the function which implements this action
            actuation_spec (ActuationSpec): Specifies the parameters needed by the function
            apply_filter (bool): Whether or not to apply the move_filter_fn after the action
        """
        assert (
            action_name in move_func_map
        ), f"No action named {action_name} in the move map"

        start_pos = obj.absolute_position()
        move_func_map[action_name](obj, actuation_spec)
        end_pos = obj.absolute_position()

        if apply_filter:
            filter_end = self.move_filter_fn(start_pos, end_pos)
            obj.translate(filter_end - end_pos)
