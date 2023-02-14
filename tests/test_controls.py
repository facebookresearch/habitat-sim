#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import attr
import hypothesis
import magnum as mn
import numpy as np
import pytest
import quaternion as qt
from hypothesis import strategies as st

import habitat_sim
import habitat_sim.errors
from habitat_sim.utils.common import angle_between_quats, quat_from_angle_axis


def test_no_action():
    scene_graph = habitat_sim.SceneGraph()
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
    scene_graph = habitat_sim.SceneGraph()
    agent_config = habitat_sim.AgentConfiguration()
    agent_config.action_space = dict(
        move_forward=habitat_sim.ActionSpec(
            "DNF", habitat_sim.ActuationSpec(amount=0.25)
        )
    )
    agent = habitat_sim.Agent(scene_graph.get_root_node().create_child(), agent_config)

    with pytest.raises(AssertionError):
        agent.act("move_forward")


@attr.s(auto_attribs=True, cmp=False)
class ExpectedDelta:
    delta_pos: np.ndarray = attr.Factory(lambda: np.array([0, 0, 0]))
    delta_rot: qt.quaternion = attr.Factory(lambda: qt.quaternion(1, 0, 0, 0))


def _check_state_same(s1, s2):
    assert np.allclose(s1.position, s2.position)
    assert angle_between_quats(s1.rotation, s2.rotation) < 1e-5


def _check_state_expected(s1, s2, expected: ExpectedDelta):
    assert np.linalg.norm(s2.position - s1.position - expected.delta_pos) < 1e-5
    assert (
        angle_between_quats(s2.rotation * expected.delta_rot.inverse(), s1.rotation)
        < 1e-5
    )


default_body_control_testdata = [
    ("move_backward", ExpectedDelta(delta_pos=0.25 * habitat_sim.geo.BACK)),
    ("move_forward", ExpectedDelta(delta_pos=0.25 * habitat_sim.geo.FRONT)),
    ("move_right", ExpectedDelta(delta_pos=0.25 * habitat_sim.geo.RIGHT)),
    ("move_left", ExpectedDelta(delta_pos=0.25 * habitat_sim.geo.LEFT)),
    (
        "turn_right",
        ExpectedDelta(
            delta_rot=quat_from_angle_axis(np.deg2rad(10.0), habitat_sim.geo.GRAVITY)
        ),
    ),
    (
        "turn_left",
        ExpectedDelta(
            delta_rot=quat_from_angle_axis(np.deg2rad(10.0), habitat_sim.geo.UP)
        ),
    ),
]


@pytest.mark.parametrize("action,expected", default_body_control_testdata)
def test_default_body_contorls(action, expected):
    scene_graph = habitat_sim.SceneGraph()
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
    ("move_up", ExpectedDelta(delta_pos=0.25 * habitat_sim.geo.UP)),
    ("move_down", ExpectedDelta(delta_pos=0.25 * habitat_sim.geo.GRAVITY)),
    (
        "look_right",
        ExpectedDelta(
            delta_rot=quat_from_angle_axis(np.deg2rad(-10.0), habitat_sim.geo.UP)
        ),
    ),
    (
        "look_left",
        ExpectedDelta(
            delta_rot=quat_from_angle_axis(np.deg2rad(10.0), habitat_sim.geo.UP)
        ),
    ),
    (
        "look_up",
        ExpectedDelta(
            delta_rot=quat_from_angle_axis(np.deg2rad(10.0), habitat_sim.geo.RIGHT)
        ),
    ),
    (
        "look_down",
        ExpectedDelta(
            delta_rot=quat_from_angle_axis(np.deg2rad(-10.0), habitat_sim.geo.RIGHT)
        ),
    ),
    (
        "rotate_sensor_clockwise",
        ExpectedDelta(
            delta_rot=quat_from_angle_axis(np.deg2rad(-10.0), habitat_sim.geo.FRONT)
        ),
    ),
    (
        "rotate_sensor_anti_clockwise",
        ExpectedDelta(
            delta_rot=quat_from_angle_axis(np.deg2rad(10.0), habitat_sim.geo.FRONT)
        ),
    ),
]


@pytest.mark.parametrize("action,expected", default_sensor_control_testdata)
def test_default_sensor_contorls(action, expected):
    scene_graph = habitat_sim.SceneGraph()
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
        rotate_sensor_clockwise=habitat_sim.ActionSpec(
            "rotate_sensor_clockwise", habitat_sim.ActuationSpec(amount=10.0)
        ),
        rotate_sensor_anti_clockwise=habitat_sim.ActionSpec(
            "rotate_sensor_anti_clockwise", habitat_sim.ActuationSpec(amount=10.0)
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


@pytest.fixture()
def scene_graph():
    return habitat_sim.SceneGraph()


@hypothesis.given(
    scene_graph=st.builds(habitat_sim.SceneGraph),
    control_pairs=st.sampled_from(
        [("look_up", 0), ("look_down", 0), ("look_left", 1), ("look_right", 1)]
    ),
    actuation_amount=st.floats(0, 60),
    actuation_constraint=st.floats(0, 60),
)
def test_constrainted(
    scene_graph, control_pairs, actuation_amount, actuation_constraint
):
    control_name, control_axis = control_pairs
    initial_look_angle = mn.Deg(
        np.random.uniform(-actuation_constraint, actuation_constraint)
    )
    rotation_vector = mn.Vector3()
    rotation_vector[control_axis] = 1
    initial_rotation = mn.Quaternion.rotation(
        mn.Rad(initial_look_angle), rotation_vector
    )

    node = scene_graph.get_root_node().create_child()
    node.rotation = initial_rotation

    spec = habitat_sim.agent.controls.ActuationSpec(
        actuation_amount, actuation_constraint
    )
    habitat_sim.registry.get_move_fn(control_name)(node, spec)

    expected_angle = initial_look_angle + mn.Deg(
        -actuation_amount
        if control_name in {"look_down", "look_right"}
        else actuation_amount
    )

    if expected_angle > mn.Deg(actuation_constraint):
        expected_angle = mn.Deg(actuation_constraint)
    elif expected_angle < mn.Deg(-actuation_constraint):
        expected_angle = mn.Deg(-actuation_constraint)

    final_rotation = node.rotation

    look_vector = final_rotation.transform_vector(habitat_sim.geo.FRONT)
    if control_axis == 0:
        look_angle = mn.Deg(mn.Rad(np.arctan2(look_vector[1], -look_vector[2])))
    elif control_axis == 1:
        look_angle = -mn.Deg(mn.Rad(np.arctan2(look_vector[0], -look_vector[2])))

    assert np.abs(float(expected_angle - look_angle)) < 1e-1
