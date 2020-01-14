#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from typing import Any, Dict, List, Union

import attr
import magnum as mn
import numpy as np

import habitat_sim.bindings as hsim
import habitat_sim.errors
from habitat_sim.sensors.sensor_suite import SensorSuite
from habitat_sim.utils.common import (
    quat_from_coeffs,
    quat_from_magnum,
    quat_rotate_vector,
    quat_to_magnum,
)

from .controls import ActuationSpec, ObjectControls

__all__ = ["ActionSpec", "SixDOFPose", "AgentState", "AgentConfiguration", "Agent"]


def _default_action_space():
    return dict(
        move_forward=ActionSpec("move_forward", ActuationSpec(amount=0.25)),
        turn_left=ActionSpec("turn_left", ActuationSpec(amount=10.0)),
        turn_right=ActionSpec("turn_right", ActuationSpec(amount=10.0)),
    )


@attr.s(auto_attribs=True)
class ActionSpec(object):
    r"""Defines how a specific action is implemented

    :property name: Name of the function implementing the action in the
        `registry`
    :property actuation: Arguments that will be passed to the function
    """
    name: str
    actuation: ActuationSpec = None


@attr.s(auto_attribs=True, slots=True)
class SixDOFPose(object):
    r"""Specifies a position with 6 degrees of freedom

    :property position: xyz position
    :property rotation: unit quaternion rotation
    """

    position: np.ndarray = np.zeros(3)
    rotation: Union[np.quaternion, List] = np.quaternion(1, 0, 0, 0)


@attr.s(auto_attribs=True, slots=True)
class AgentState(object):
    position: np.ndarray = np.zeros(3)
    rotation: Union[np.quaternion, List] = np.quaternion(1, 0, 0, 0)
    velocity: np.ndarray = np.zeros(3)
    angular_velocity: np.ndarray = np.zeros(3)
    force: np.ndarray = np.zeros(3)
    torque: np.ndarray = np.zeros(3)
    sensor_states: Dict[str, SixDOFPose] = attr.Factory(dict)


@attr.s(auto_attribs=True, slots=True)
class AgentConfiguration(object):
    height: float = 1.5
    radius: float = 0.1
    mass: float = 32.0
    linear_acceleration: float = 20.0
    angular_acceleration: float = 4 * np.pi
    linear_friction: float = 0.5
    angular_friction: float = 1.0
    coefficient_of_restitution: float = 0.0
    sensor_specifications: List[hsim.SensorSpec] = attr.Factory(
        lambda: [hsim.SensorSpec()]
    )
    action_space: Dict[Any, ActionSpec] = attr.Factory(_default_action_space)
    body_type: str = "cylinder"


@attr.s(init=False, auto_attribs=True)
class Agent(object):
    r"""Implements an agent with multiple sensors

    :property agent_config: The configuration of the agent

    .. block-warning:: Warning

        Agents are given controls over a node in the scene graph, but do
        **not** own this node. This means that errors will occur if the owner
        of the scene graph is deallocated. Generally the owner of the scene
        graph is the Simulator.

        If you'd like to have an agent to control without loading up the
        simulator, see unit tests for the agent in ``tests/test_agent.py``. We
        recommend letting the simulator create the agent and own the scene
        graph in almost all cases. Using the scene graph in python is dangerous
        due to differences in c++ and python memory management.
    """

    agent_config: AgentConfiguration
    _sensors: SensorSuite
    controls: ObjectControls
    body: mn.scenegraph.AbstractFeature3D

    def __init__(
        self,
        scene_node: hsim.SceneNode,
        agent_config=None,
        _sensors=None,
        controls=None,
    ):
        self.agent_config = agent_config if agent_config else AgentConfiguration()
        self._sensors = _sensors if _sensors else SensorSuite()
        self.controls = controls if controls else ObjectControls()
        self.body = mn.scenegraph.AbstractFeature3D(scene_node)
        scene_node.type = hsim.SceneNodeType.AGENT
        self.reconfigure(self.agent_config)

    def reconfigure(
        self, agent_config: AgentConfiguration, reconfigure_sensors: bool = True
    ):
        r"""Re-create the agent with a new configuration

        :param agent_config: New config
        :param reconfigure_sensors: Whether or not to also reconfigure the
            sensors. There are specific cases where :py:`False` makes sense,
            but most cases are covered by :py:`True`.
        """
        habitat_sim.errors.assert_obj_valid(self.body)
        self.agent_config = agent_config

        if reconfigure_sensors:
            self._sensors.clear()
            for spec in self.agent_config.sensor_specifications:
                self._sensors.add(
                    hsim.PinholeCamera(self.scene_node.create_child(), spec)
                )

    def act(self, action_id: Any) -> bool:
        r"""Take the action specified by action_id

        :param action_id: ID of the action. Retreives the action from
            `agent_config.action_space <AgentConfiguration.action_space>`
        :return: Whether or not the action taken resulted in a collision
        """

        habitat_sim.errors.assert_obj_valid(self.body)
        assert (
            action_id in self.agent_config.action_space
        ), f"No action {action_id} in action space"
        action = self.agent_config.action_space[action_id]

        did_collide = False
        if self.controls.is_body_action(action.name):
            did_collide = self.controls.action(
                self.scene_node, action.name, action.actuation, apply_filter=True
            )
        else:
            for _, v in self._sensors.items():
                habitat_sim.errors.assert_obj_valid(v)
                self.controls.action(
                    v.object, action.name, action.actuation, apply_filter=False
                )

        return did_collide

    def get_state(self) -> AgentState:
        habitat_sim.errors.assert_obj_valid(self.body)
        state = AgentState(
            np.array(self.body.object.absolute_translation), self.body.object.rotation
        )

        for k, v in self._sensors.items():
            habitat_sim.errors.assert_obj_valid(v)
            state.sensor_states[k] = SixDOFPose(
                np.array(v.node.absolute_translation),
                quat_from_magnum(state.rotation * v.node.rotation),
            )

        state.rotation = quat_from_magnum(state.rotation)

        return state

    def set_state(self, state: AgentState, reset_sensors: bool = True):
        r"""Sets the agents state

        :param state: The state to set the agent to
        :param reset_sensors: Whether or not to reset the sensors to their
            default intrinsic/extrinsic parameters before setting their
            extrinsic state
        """
        habitat_sim.errors.assert_obj_valid(self.body)

        if isinstance(state.rotation, list):
            state.rotation = quat_from_coeffs(state.rotation)

        self.body.object.reset_transformation()

        self.body.object.translate(state.position)
        self.body.object.rotation = quat_to_magnum(state.rotation)

        if reset_sensors:
            for _, v in self._sensors.items():
                v.set_transformation_from_spec()

        for k, v in state.sensor_states.items():
            assert k in self._sensors
            if isinstance(v.rotation, list):
                v.rotation = quat_from_coeffs(v.rotation)

            s = self._sensors[k]

            s.node.reset_transformation()
            s.node.translate(
                quat_rotate_vector(
                    state.rotation.inverse(), v.position - state.position
                )
            )
            s.node.rotation = quat_to_magnum(state.rotation.inverse() * v.rotation)

    @property
    def scene_node(self):
        habitat_sim.errors.assert_obj_valid(self.body)
        return self.body.object

    @property
    def state(self):
        return self.get_state()

    @state.setter
    def state(self, new_state):
        self.set_state(new_state, reset_sensors=True)

    def close(self):
        self._sensors = None
