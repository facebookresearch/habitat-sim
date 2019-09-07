#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import abc
import re
from typing import Dict, Optional, Type

import numpy as np
import quaternion

import attr
import habitat_sim.bindings as hsim
from habitat_sim import utils
from habitat_sim.registry import registry

# epislon used to deal with machine precision
EPS = 1e-5


@attr.s(auto_attribs=True)
class ActuationSpec(object):
    r"""Struct to hold parameters for the default actions

    :property amount: The amount the control moves the scene node by

    The default actions only have one parameters, the amount
    they move the scene node by, however other actions may have any number of
    parameters and can define different structs to hold those parameters
    """

    amount: float


@attr.s(auto_attribs=True)
class SceneNodeControl(abc.ABC):
    r"""Base class for all controls

    :property body_action: Whether or not the control function manipulates the
        agents body or the sensors

    Control classes are used to implement agent actions.  Any new control
    must subclass this class.

    See ``default_controls.py`` for an example of adding new actions. (Note
    that this can be done *outside* the core `habitat_sim` codebase in exactly
    the same way.)

    See ``examples/new_actions.py`` for an example of how to add new actions
    *outside* the core habitat_sim package.
    """

    body_action: bool = False

    @abc.abstractmethod
    def __call__(self, scene_node: hsim.SceneNode, acutation_spec: ActuationSpec):
        r"""Abstract method to be overridden to implement the control

        :param scene_node: The scene node to control
        :param acutation_spec: Struct holding any parameters of the control
        """
        pass


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
