#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import numpy as np
import pytest
import quaternion  # noqa: F401

import habitat_sim
import habitat_sim.errors
from habitat_sim.utils.common import angle_between_quats, quat_from_angle_axis


def _check_state_same(s1, s2):
    assert np.allclose(s1.position, s2.position, atol=1e-5)
    assert angle_between_quats(s1.rotation, s2.rotation) < 1e-5


def test_reconfigure():
    scene_graph = habitat_sim.SceneGraph()
    agent = habitat_sim.Agent(scene_graph.get_root_node().create_child())

    habitat_sim.errors.assert_obj_valid(agent.body)
    for v in agent._sensors.values():
        habitat_sim.errors.assert_obj_valid(v)

    agent.reconfigure(agent.agent_config)
    for v in agent._sensors.values():
        habitat_sim.errors.assert_obj_valid(v)

    agent.reconfigure(agent.agent_config, True)
    for v in agent._sensors.values():
        habitat_sim.errors.assert_obj_valid(v)


def test_set_state():
    scene_graph = habitat_sim.SceneGraph()
    agent = habitat_sim.Agent(scene_graph.get_root_node().create_child())

    state = agent.state
    agent.set_state(state, infer_sensor_states=True)
    new_state = agent.state

    _check_state_same(state, new_state)

    for k, v in state.sensor_states.items():
        assert k in new_state.sensor_states
        _check_state_same(v, new_state.sensor_states[k])


def test_set_state_error():
    scene_graph = habitat_sim.SceneGraph()
    agent = habitat_sim.Agent(scene_graph.get_root_node().create_child())
    state = agent.state
    state.position = np.array([float("NaN")] * 3)
    with pytest.raises(ValueError):
        agent.set_state(state)


def test_change_state():
    random_state = np.random.get_state()
    np.random.seed(233)
    scene_graph = habitat_sim.SceneGraph()
    agent = habitat_sim.Agent(scene_graph.get_root_node().create_child())

    for _ in range(100):
        state = agent.state
        state.position += np.random.uniform(-1, 1, size=3)
        state.rotation *= quat_from_angle_axis(
            np.random.uniform(0, 2 * np.pi), np.array([0.0, 1.0, 0.0])
        )
        for v in state.sensor_states.values():
            v.position += np.random.uniform(-1, 1, size=3)
            v.rotation *= quat_from_angle_axis(
                np.random.uniform(0, 2 * np.pi), np.array([1.0, 1.0, 1.0])
            )

        agent.set_state(state, infer_sensor_states=False)
        new_state = agent.state

        _check_state_same(state, new_state)

        for k, v in state.sensor_states.items():
            assert k in new_state.sensor_states
            _check_state_same(v, new_state.sensor_states[k])

    np.random.set_state(random_state)
