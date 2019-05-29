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

__all__ = ["ActuationSpec", "Controller", "ObjectControls"]


def _camel_to_snake(name):
    s1 = re.sub("(.)([A-Z][a-z]+)", r"\1_\2", name)
    return re.sub("([a-z0-9])([A-Z])", r"\1_\2", s1).lower()


@attr.s(auto_attribs=True, slots=True)
class ActuationSpec(object):
    amount: float


@attr.s(auto_attribs=True, slots=True)
class Controller(abc.ABC):
    body_action: bool = False

    @abc.abstractmethod
    def __call__(self, scene_node: hsim.SceneNode, actuation_spec: ActuationSpec):
        pass


move_func_map: Dict[str, Controller] = dict()


def register_move_fn(
    controller: Optional[Type[Controller]] = None,
    *,
    name: Optional[str] = None,
    body_action: bool = False,
):
    def _wrapper(controller: Type[Controller]):
        assert issubclass(
            controller, Controller
        ), "All controls must inherit from habitat_sim.agent.Controller"

        move_func_map[
            _camel_to_snake(controller.__name__) if name is None else name
        ] = controller(body_action)

        return controller

    if controller is None:
        return _wrapper
    else:
        return _wrapper(controller)


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

    @staticmethod
    def is_body_action(action_name: str):
        r"""Checks to see if :py:attr:`action_name` is a body action

        Args:
            action_name (str): Name of the action.
        """
        assert (
            action_name in move_func_map
        ), f"No action named {action_name} in the move map"

        return move_func_map[action_name].body_action

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
