#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import numpy as np
import pytest
import quaternion

import habitat_sim
import habitat_sim.bindings as hsim
import habitat_sim.errors
import habitat_sim.utils


def _check_state_same(s1, s2):
    assert np.allclose(s1.position, s2.position, atol=1e-5)
    assert habitat_sim.utils.angle_between_quats(s1.rotation, s2.rotation) < 1e-5


def test_reconfigure():
    scene_graph = hsim.SceneGraph()
    agent = habitat_sim.Agent(scene_graph.get_root_node().create_child())

    habitat_sim.errors.assert_obj_valid(agent.body)
    for _, v in agent.sensors.items():
        habitat_sim.errors.assert_obj_valid(v)

    agent.reconfigure(agent.agent_config)
    for _, v in agent.sensors.items():
        habitat_sim.errors.assert_obj_valid(v)

    agent.reconfigure(agent.agent_config, True)
    for _, v in agent.sensors.items():
        habitat_sim.errors.assert_obj_valid(v)


def test_set_state():
    scene_graph = hsim.SceneGraph()
    agent = habitat_sim.Agent(scene_graph.get_root_node().create_child())

    state = agent.state
    agent.state = state
    new_state = agent.state

    _check_state_same(state, new_state)

    for k, v in state.sensor_states.items():
        assert k in new_state.sensor_states
        _check_state_same(v, new_state.sensor_states[k])


def test_change_state():
    scene_graph = hsim.SceneGraph()
    agent = habitat_sim.Agent(scene_graph.get_root_node().create_child())

    for _ in range(100):
        state = agent.state
        state.position += np.random.uniform(-1, 1, size=3)
        state.rotation *= habitat_sim.utils.quat_from_angle_axis(
            np.random.uniform(0, 2 * np.pi), np.array([0.0, 1.0, 0.0])
        )
        for k, v in state.sensor_states.items():
            v.position += np.random.uniform(-1, 1, size=3)
            v.rotation *= habitat_sim.utils.quat_from_angle_axis(
                np.random.uniform(0, 2 * np.pi), np.array([1.0, 1.0, 1.0])
            )

        agent.state = state
        new_state = agent.state

        _check_state_same(state, new_state)

        for k, v in state.sensor_states.items():
            assert k in new_state.sensor_states
            _check_state_same(v, new_state.sensor_states[k])
