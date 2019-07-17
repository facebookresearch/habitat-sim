#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import attr
import numpy as np
import pytest
import quaternion

import habitat_sim
import habitat_sim.bindings as hsim
import habitat_sim.errors
import habitat_sim.utils


def test_no_action():
    scene_graph = hsim.SceneGraph()
    agent_config = habitat_sim.AgentConfiguration()
    agent_config.action_space = dict(
        move_backward=habitat_sim.ActionSpec(
            "move_backward", habitat_sim.ActuationSpec(amount=0.25)
        )
    )
    agent = habitat_sim.Agent(scene_graph.get_root_node().create_child(), agent_config)

    with pytest.raises(AssertionError):
        agent.act("move_forward")


def test_no_move_fun():
    scene_graph = hsim.SceneGraph()
    agent_config = habitat_sim.AgentConfiguration()
    agent_config.action_space = dict(
        move_forward=habitat_sim.ActionSpec(
            "DNF", habitat_sim.ActuationSpec(amount=0.25)
        )
    )
    agent = habitat_sim.Agent(scene_graph.get_root_node().create_child(), agent_config)

    with pytest.raises(AssertionError):
        agent.act("move_forward")


@attr.s(auto_attribs=True)
class ExpectedDelta:
    delta_pos: np.array = np.array([0, 0, 0])
    delta_rot: np.quaternion = np.quaternion(1, 0, 0, 0)


def _check_state_same(s1, s2):
    assert np.allclose(s1.position, s2.position)
    assert habitat_sim.utils.angle_between_quats(s1.rotation, s2.rotation) < 1e-5


def _check_state_expected(s1, s2, expected: ExpectedDelta):
    assert np.linalg.norm(s2.position - s1.position - expected.delta_pos) < 1e-5
    assert (
        habitat_sim.utils.angle_between_quats(
            s2.rotation * expected.delta_rot.inverse(), s1.rotation
        )
        < 1e-5
    )


default_body_control_testdata = [
    ("move_backward", ExpectedDelta(delta_pos=0.25 * hsim.geo.BACK)),
    ("move_forward", ExpectedDelta(delta_pos=0.25 * hsim.geo.FRONT)),
    ("move_right", ExpectedDelta(delta_pos=0.25 * hsim.geo.RIGHT)),
    ("move_left", ExpectedDelta(delta_pos=0.25 * hsim.geo.LEFT)),
    (
        "turn_right",
        ExpectedDelta(
            delta_rot=habitat_sim.utils.quat_from_angle_axis(
                np.deg2rad(10.0), hsim.geo.GRAVITY
            )
        ),
    ),
    (
        "turn_left",
        ExpectedDelta(
            delta_rot=habitat_sim.utils.quat_from_angle_axis(
                np.deg2rad(10.0), hsim.geo.UP
            )
        ),
    ),
]


@pytest.mark.parametrize("action,expected", default_body_control_testdata)
def test_default_body_contorls(action, expected):
    scene_graph = hsim.SceneGraph()
    agent_config = habitat_sim.AgentConfiguration()
    agent_config.action_space = dict(
        move_backward=habitat_sim.ActionSpec(
            "move_backward", habitat_sim.ActuationSpec(amount=0.25)
        ),
        move_forward=habitat_sim.ActionSpec(
            "move_forward", habitat_sim.ActuationSpec(amount=0.25)
        ),
        move_left=habitat_sim.ActionSpec(
            "move_left", habitat_sim.ActuationSpec(amount=0.25)
        ),
        move_right=habitat_sim.ActionSpec(
            "move_right", habitat_sim.ActuationSpec(amount=0.25)
        ),
        turn_left=habitat_sim.ActionSpec(
            "turn_left", habitat_sim.ActuationSpec(amount=10.0)
        ),
        turn_right=habitat_sim.ActionSpec(
            "turn_right", habitat_sim.ActuationSpec(amount=10.0)
        ),
    )
    agent = habitat_sim.Agent(scene_graph.get_root_node().create_child(), agent_config)

    state = agent.state
    agent.act(action)
    new_state = agent.state

    _check_state_expected(state, new_state, expected)
    for k, v in state.sensor_states.items():
        assert k in new_state.sensor_states
        _check_state_expected(v, new_state.sensor_states[k], expected)


default_sensor_control_testdata = [
    ("move_up", ExpectedDelta(delta_pos=0.25 * hsim.geo.UP)),
    ("move_down", ExpectedDelta(delta_pos=0.25 * hsim.geo.GRAVITY)),
    (
        "look_right",
        ExpectedDelta(
            delta_rot=habitat_sim.utils.quat_from_angle_axis(
                np.deg2rad(-10.0), hsim.geo.UP
            )
        ),
    ),
    (
        "look_left",
        ExpectedDelta(
            delta_rot=habitat_sim.utils.quat_from_angle_axis(
                np.deg2rad(10.0), hsim.geo.UP
            )
        ),
    ),
    (
        "look_up",
        ExpectedDelta(
            delta_rot=habitat_sim.utils.quat_from_angle_axis(
                np.deg2rad(10.0), hsim.geo.RIGHT
            )
        ),
    ),
    (
        "look_down",
        ExpectedDelta(
            delta_rot=habitat_sim.utils.quat_from_angle_axis(
                np.deg2rad(-10.0), hsim.geo.RIGHT
            )
        ),
    ),
]


@pytest.mark.parametrize("action,expected", default_sensor_control_testdata)
def test_default_sensor_contorls(action, expected):
    scene_graph = hsim.SceneGraph()
    agent_config = habitat_sim.AgentConfiguration()
    agent_config.action_space = dict(
        move_up=habitat_sim.ActionSpec(
            "move_up", habitat_sim.ActuationSpec(amount=0.25)
        ),
        move_down=habitat_sim.ActionSpec(
            "move_down", habitat_sim.ActuationSpec(amount=0.25)
        ),
        look_left=habitat_sim.ActionSpec(
            "look_left", habitat_sim.ActuationSpec(amount=10.0)
        ),
        look_right=habitat_sim.ActionSpec(
            "look_right", habitat_sim.ActuationSpec(amount=10.0)
        ),
        look_up=habitat_sim.ActionSpec(
            "look_up", habitat_sim.ActuationSpec(amount=10.0)
        ),
        look_down=habitat_sim.ActionSpec(
            "look_down", habitat_sim.ActuationSpec(amount=10.0)
        ),
    )
    agent = habitat_sim.Agent(scene_graph.get_root_node().create_child(), agent_config)

    state = agent.state
    agent.act(action)
    new_state = agent.state

    _check_state_same(state, new_state)
    for k, v in state.sensor_states.items():
        assert k in new_state.sensor_states
        _check_state_expected(v, new_state.sensor_states[k], expected)
