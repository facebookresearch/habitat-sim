#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import attr
import numpy as np
import quaternion

import habitat_sim.bindings as hsim
from habitat_sim.agent.controls.controls import ActuationSpec
from habitat_sim.registry import registry

# epislon used to deal with machine precision
EPS = 1e-5


def _noop_filter(start: np.array, end: np.array):
    return end


@attr.s
class ObjectControls(object):
    r"""Used to implement actions

    :property move_filter_fn: A function that is applied after actions to
        handle collisions
    """

    move_filter_fn: float = attr.ib(default=_noop_filter)

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
        start_orienation = obj.rotation
        move_fn = registry.get_move_fn(action_name)
        assert move_fn is not None, f"No move_fn for action '{action_name}'"
        move_fn(obj, actuation_spec)
        end_pos = obj.absolute_translation
        end_orienation = obj.rotation

        collided = False
        if apply_filter:
            filter_end_state = self.move_filter_fn(
                start_pos, end_pos, start_orienation, end_orienation
            )
            # Update the position to respect the filter
            obj.translation = filter_end_state[0]
            obj.rotation = filter_end_state[1]
            collided = filter_end_state[2]

        return collided

    def __call__(
        self,
        obj: hsim.SceneNode,
        action_name: str,
        actuation_spec: ActuationSpec,
        apply_filter: bool = True,
    ):
        return self.action(obj, action_name, actuation_spec, apply_filter)
