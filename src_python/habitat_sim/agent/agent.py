#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from typing import Any, Dict, List, Optional, Union

import attr
import magnum as mn
import numpy as np
import quaternion as qt

import habitat_sim.errors
from habitat_sim import bindings as hsim
from habitat_sim._ext.habitat_sim_bindings import SceneNode
from habitat_sim.sensors.sensor_suite import SensorSuite
from habitat_sim.utils.common import (
    quat_from_coeffs,
    quat_from_magnum,
    quat_rotate_vector,
    quat_to_magnum,
)
from habitat_sim.utils.validators import (
    NoAttrValidationContext,
    all_is_finite,
    is_unit_length,
    value_is_validated,
)

from .controls import ActuationSpec, ObjectControls

__all__ = ["ActionSpec", "SixDOFPose", "AgentState", "AgentConfiguration", "Agent"]


@attr.s(auto_attribs=True)
class ActionSpec:
    r"""Defines how a specific action is implemented

    :property name: Name of the function implementing the action in the
        `registry`
    :property actuation: Arguments that will be passed to the function
    """
    name: str
    actuation: Optional[ActuationSpec] = None


def _default_action_space() -> Dict[str, ActionSpec]:
    return dict(
        move_forward=ActionSpec("move_forward", ActuationSpec(amount=0.25)),
        turn_left=ActionSpec("turn_left", ActuationSpec(amount=10.0)),
        turn_right=ActionSpec("turn_right", ActuationSpec(amount=10.0)),
    )


def _triple_zero() -> np.ndarray:
    return np.zeros(3)


def _default_quaternion() -> qt.quaternion:
    return qt.quaternion(1, 0, 0, 0)


@attr.s(auto_attribs=True, slots=True)
class SixDOFPose:
    r"""Specifies a position with 6 degrees of freedom

    :property position: xyz position
    :property rotation: unit quaternion rotation
    """

    position: np.ndarray = attr.ib(factory=_triple_zero, validator=all_is_finite)
    rotation: Union[qt.quaternion, List] = attr.ib(
        factory=_default_quaternion, validator=is_unit_length
    )


@attr.s(auto_attribs=True, slots=True)
class AgentState:
    position: np.ndarray = attr.ib(factory=_triple_zero, validator=all_is_finite)
    rotation: Union[qt.quaternion, List, np.ndarray] = attr.ib(
        factory=_default_quaternion, validator=is_unit_length
    )
    sensor_states: Dict[str, SixDOFPose] = attr.ib(
        factory=dict,
        validator=attr.validators.deep_mapping(
            key_validator=attr.validators.instance_of(str),
            value_validator=value_is_validated,
            mapping_validator=attr.validators.instance_of(dict),
        ),
    )


@attr.s(auto_attribs=True, slots=True)
class AgentConfiguration:
    height: float = 1.5
    radius: float = 0.1
    sensor_specifications: List[hsim.SensorSpec] = attr.Factory(list)
    action_space: Dict[Any, ActionSpec] = attr.Factory(_default_action_space)
    body_type: str = "cylinder"


@attr.s(init=False, auto_attribs=True)
class Agent:
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
        agent_config: Optional[AgentConfiguration] = None,
        _sensors: Optional[SensorSuite] = None,
        controls: Optional[ObjectControls] = None,
    ) -> None:
        self.agent_config = agent_config if agent_config else AgentConfiguration()
        self._sensors = _sensors if _sensors else SensorSuite()
        self.controls = controls if controls else ObjectControls()
        self.body = mn.scenegraph.AbstractFeature3D(scene_node)
        scene_node.type = hsim.SceneNodeType.AGENT
        self.reconfigure(self.agent_config)
        self.initial_state: Optional[AgentState] = None

    def reconfigure(
        self, agent_config: AgentConfiguration, reconfigure_sensors: bool = True
    ) -> None:
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
                self._add_sensor(spec, modify_agent_config=False)

    def _add_sensor(
        self, spec: hsim.SensorSpec, modify_agent_config: bool = True
    ) -> None:
        assert (
            spec.uuid not in self._sensors
        ), f"Error, {spec.uuid} already exists in the sensor suite"
        if modify_agent_config:
            assert spec not in self.agent_config.sensor_specifications
            self.agent_config.sensor_specifications.append(spec)
        sensor_suite = hsim.SensorFactory.create_sensors(self.scene_node, [spec])
        self._sensors.add(sensor_suite[spec.uuid])

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

    @NoAttrValidationContext()
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

    def set_state(
        self,
        state: AgentState,
        reset_sensors: bool = True,
        infer_sensor_states: bool = True,
        is_initial: bool = False,
    ) -> None:
        r"""Sets the agents state

        :param state: The state to set the agent to
        :param reset_sensors: Whether or not to reset the sensors to their
            default intrinsic/extrinsic parameters before setting their extrinsic state.
        :param infer_sensor_states: Whether or not to infer the location of sensors based on
            the new location of the agent base state.
        :param is_initial: Whether this state is the initial state of the
            agent in the scene. Used for resetting the agent at a later time

        Setting ``reset_sensors`` to :py:`False`
        allows the agent base state to be moved and the new
        sensor locations inferred without changing the configuration of the sensors
        with respect to the base state of the agent.

        Setting ``infer_sensor_states``
        to :py:`False` is useful if you'd like to directly control
        the state of a sensor instead of moving the agent.

        """
        attr.validate(state)
        habitat_sim.errors.assert_obj_valid(self.body)

        if isinstance(state.rotation, (list, np.ndarray)):
            state.rotation = quat_from_coeffs(state.rotation)

        self.body.object.reset_transformation()

        self.body.object.translate(state.position)
        self.body.object.rotation = quat_to_magnum(state.rotation)

        if reset_sensors:
            for _, v in self._sensors.items():
                v.set_transformation_from_spec()

        if not infer_sensor_states:
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

        if is_initial:
            self.initial_state = state

    @property
    def scene_node(self) -> SceneNode:
        habitat_sim.errors.assert_obj_valid(self.body)
        return self.body.object

    @property
    def state(self):
        r"""Get/set the agent's state.

        Getting the state is equivalent to :ref:`get_state`

        Setting the state is equivalent calling :ref:`set_state`
        and only providing the state.
        """
        return self.get_state()

    @state.setter
    def state(self, new_state):
        self.set_state(
            new_state, reset_sensors=True, infer_sensor_states=True, is_initial=False
        )

    def close(self) -> None:
        self._sensors = None
