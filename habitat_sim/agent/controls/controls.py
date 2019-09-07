#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import abc
from typing import Dict, Optional, Type

import attr
import numpy as np
import quaternion

import habitat_sim.bindings as hsim


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
