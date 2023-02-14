#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from typing import Callable, Tuple, Union

import attr
import magnum as mn
import numpy as np
import quaternion  # noqa: F401

from habitat_sim import bindings as hsim
from habitat_sim.agent.controls.controls import ActuationSpec
from habitat_sim.registry import registry

# epislon used to deal with machine precision
EPS = 1e-5
_3d_point = Union[np.ndarray, mn.Vector3, Tuple[float, float, float]]


def _noop_filter(start: _3d_point, end: _3d_point) -> _3d_point:
    return end


@attr.s(auto_attribs=True)
class ObjectControls:
    r"""Used to implement actions

    :property move_filter_fn: A function that is applied after actions to
        handle collisions
    """

    move_filter_fn: Callable[[_3d_point, _3d_point], _3d_point] = attr.ib(
        default=_noop_filter
    )

    @staticmethod
    def is_body_action(action_name: str):
        r"""Checks to see if :p:`action_name` is a body action

        :param action_name: Name of the action
        """
        move_fn = registry.get_move_fn(action_name)
        assert move_fn is not None, f"No move_fn for action '{action_name}'"

        return move_fn.body_action

    def action(
        self,
        obj: hsim.SceneNode,
        action_name: str,
        actuation_spec: ActuationSpec,
        apply_filter: bool = True,
    ) -> bool:
        r"""Performs the action specified by :p:`action_name` on the object

        :param obj: `scene.SceneNode` to perform the action on
        :param action_name: Name of the action. Used to query the
            `registry` to retrieve the function which implements
            this action
        :param actuation_spec: Specifies the parameters needed by the function
        :param apply_filter: Whether or not to apply the `move_filter_fn`
            after the action
        :return: Whether or not the action taken resulted in a collision
        """
        start_pos = obj.absolute_translation
        move_fn = registry.get_move_fn(action_name)
        assert move_fn is not None, f"No move_fn for action '{action_name}'"
        move_fn(obj, actuation_spec)
        end_pos = obj.absolute_translation

        collided = False
        if apply_filter:
            filter_end = self.move_filter_fn(start_pos, end_pos)
            # Update the position to respect the filter
            obj.translate(filter_end - end_pos)

            dist_moved_before_filter = (end_pos - start_pos).dot()
            dist_moved_after_filter = (filter_end - start_pos).dot()

            # NB: There are some cases where ||filter_end - end_pos|| > 0 when a
            # collision _didn't_ happen. One such case is going up stairs.  Instead,
            # we check to see if the the amount moved after the application of the filter
            # is _less_ the the amount moved before the application of the filter
            collided = (dist_moved_after_filter + EPS) < dist_moved_before_filter

        return collided

    def __call__(
        self,
        obj: hsim.SceneNode,
        action_name: str,
        actuation_spec: ActuationSpec,
        apply_filter: bool = True,
    ):
        return self.action(obj, action_name, actuation_spec, apply_filter)
