#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import abc
import re
from typing import Dict, Optional, Type

import attr
import numpy as np
import quaternion

import habitat_sim.bindings as hsim
from habitat_sim import utils

__all__ = ["ActuationSpec", "SceneNodeControl", "ObjectControls"]


# epislon used to deal with machine precision
EPS = 1e-5


def _camel_to_snake(name):
    s1 = re.sub("(.)([A-Z][a-z]+)", r"\1_\2", name)
    return re.sub("([a-z0-9])([A-Z])", r"\1_\2", s1).lower()


@attr.s(auto_attribs=True)
class ActuationSpec(object):
    r"""Struct to hold parameters for the default actions

    The default actions only have one parameters, the amount
    they move the scene node by, however other actions may have any number of
    parameters and can define different structs to hold those parameters

    Args:
        amount (float): The amount the control moves the scene node by
    """
    amount: float


@attr.s(auto_attribs=True)
class SceneNodeControl(abc.ABC):
    r"""Base class for all controls

    Control classes are used to implement agent actions.  Any new control
    must subclass this class.

    See default_controls.py for an example of adding new actions
    (note that this can be done _outside_ the core habitat_sim codebase in exactly the same way)

    See examples/new_actions.py for an example of how to add new actions _outside_
    the core habitat_sim package
    """

    body_action: bool = False

    @abc.abstractmethod
    def __call__(self, scene_node: hsim.SceneNode, actuation_spec: ActuationSpec):
        r"""Abstract method to be overridden to implement the control

        Args:
            scene_node (hsim.SceneNode): The scene node to control
            actuation_spec (ActuationSpec): Struct holding any parameters of the control
        """
        pass


move_func_map: Dict[str, SceneNodeControl] = dict()


def register_move_fn(
    controller: Optional[Type[SceneNodeControl]] = None,
    *,
    name: Optional[str] = None,
    body_action: bool = None,
):
    r"""Registers a new control with Habitat-Sim

    See default_controls.py for an example of adding new actions
    (note that this can be done _outside_ the core habitat_sim codebase in exactly the same way)

    See examples/new_actions.py for an example of how to add new actions _outside_
    the core habitat_sim package

    Args:
        controller (Optional[Type[SceneNodeControl]]): The class of the controller to register
            If none, will return a wrapper for use with decorator syntax
        name (Optional[str]): The name to register the control with
            If none, will register with the name of the controller converted to snake case
            i.e. a controller with class name MoveForward will be registered as move_forward
        body_action (bool): Whether or not this action manipulates the agent's body
            (thereby also moving the sensors) or manipulates just the sensors.
            This is a non-optional keyword arguement and must be set (this is done for readability)
    """

    assert (
        body_action is not None
    ), "body_action must be explicitly set to True or False"

    def _wrapper(controller: Type[SceneNodeControl]):
        assert issubclass(
            controller, SceneNodeControl
        ), "All controls must inherit from habitat_sim.agent.SceneNodeControl"

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
    ) -> bool:
        r"""Performs the action specified by :py:attr:`action_name` on the object

        Args:
            obj (hsim.SceneNode): SceneNode to perform the action on
            action_name (str): Name of the action.  Used to index the move_func_map
                to retrieve the function which implements this action
            actuation_spec (ActuationSpec): Specifies the parameters needed by the function
            apply_filter (bool): Whether or not to apply the move_filter_fn after the action

        Returns:
            bool: Whether or not the action taken resulted in a collision
        """
        assert (
            action_name in move_func_map
        ), f"No action named {action_name} in the move map"

        start_pos = obj.absolute_transformation()._translation
        move_func_map[action_name](obj, actuation_spec)
        end_pos = obj.absolute_transformation()._translation

        collided = False
        if apply_filter:
            filter_end = self.move_filter_fn(start_pos, end_pos)
            # Update the position to respect the filter
            obj.translate(filter_end - end_pos)

            dist_moved_before_filter = np.linalg.norm(end_pos - start_pos)
            dist_moved_after_filter = np.linalg.norm(filter_end - start_pos)

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
