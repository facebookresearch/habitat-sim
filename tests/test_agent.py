#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import numpy as np
import pytest
import quaternion  # noqa: F401

import habitat_sim
import habitat_sim.errors
from habitat_sim.sensor import SensorFactory
from habitat_sim.utils.common import (
    angle_between_quats,
    quat_from_angle_axis,
    quat_to_magnum,
)


def _check_state_same(s1, s2):
    assert np.allclose(s1.position, s2.position, atol=1e-5)
    # remove quat_to_magnum once AgentState is refactored to use magnum quaternions
    assert (
        angle_between_quats(quat_to_magnum(s1.rotation), quat_to_magnum(s2.rotation))
        < 1e-5
    )


def test_reconfigure():
    scene_graph = habitat_sim.SceneGraph()
    agent = habitat_sim.Agent(scene_graph.get_root_node().create_child())

    habitat_sim.errors.assert_obj_valid(agent.body)
    for v in agent.sensors.values():
        habitat_sim.errors.assert_obj_valid(v)

    agent.reconfigure(agent.agent_config)
    for v in agent.sensors.values():
        habitat_sim.errors.assert_obj_valid(v)

    agent.reconfigure(agent.agent_config, True)
    for v in agent.sensors.values():
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


def test_sensor_subtree_discovery():
    """Sensors created on child nodes auto-appear in agent.sensors."""
    scene_graph = habitat_sim.SceneGraph()
    agent_node = scene_graph.get_root_node().create_child()
    agent = habitat_sim.Agent(agent_node)

    # Agent starts with default sensors from its config.
    initial_count = len(agent.sensors)

    # Add a new sensor on a child of the agent node.
    child_node = agent_node.create_child()
    spec = habitat_sim.CameraSensorSpec()
    spec.uuid = "extra_rgb"
    spec.sensor_type = habitat_sim.SensorType.COLOR
    spec.resolution = [64, 64]
    SensorFactory.create_sensors(child_node, [spec])

    # The new sensor should appear in agent.sensors automatically.
    assert "extra_rgb" in agent.sensors
    assert len(agent.sensors) == initial_count + 1

    # Remove the sensor — it should disappear from agent.sensors.
    SensorFactory.delete_subtree_sensor(child_node, "extra_rgb")
    assert "extra_rgb" not in agent.sensors
    assert len(agent.sensors) == initial_count


def test_backward_compat_sensors_alias():
    """agent._sensors is the same live view as agent.sensors."""
    scene_graph = habitat_sim.SceneGraph()
    agent = habitat_sim.Agent(scene_graph.get_root_node().create_child())

    # Both should return the same set of keys.
    assert set(agent.sensors.keys()) == set(agent._sensors.keys())
    for k in agent.sensors:
        assert agent.sensors[k] is agent._sensors[k]
