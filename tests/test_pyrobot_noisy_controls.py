#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import numpy as np
import pytest
import quaternion as qt

import habitat_sim
import habitat_sim.errors
import habitat_sim.utils.common
from habitat_sim.agent.controls.pyrobot_noisy_controls import pyrobot_noise_models
from habitat_sim.scene import SceneGraph


def _delta_translation(a, b):
    d = b.position - a.position
    return np.array([-d[2], d[0]])


def _delta_rotation(a, b):
    look_dir = habitat_sim.utils.common.quat_rotate_vector(
        b.rotation.inverse() * a.rotation, habitat_sim.geo.FRONT
    )

    return np.arctan2(look_dir[0], -look_dir[2])


@pytest.mark.parametrize("noise_multiplier", [1.0, 0.0])
@pytest.mark.parametrize("robot", ["LoCoBot", "LoCoBot-Lite"])
@pytest.mark.parametrize("controller", ["ILQR", "Proportional", "Movebase"])
def test_pyrobot_noisy_actions(noise_multiplier, robot, controller):
    np.random.seed(0)
    scene_graph = SceneGraph()
    agent_config = habitat_sim.AgentConfiguration()
    agent_config.action_space = dict(
        noisy_move_backward=habitat_sim.ActionSpec(
            "pyrobot_noisy_move_backward",
            habitat_sim.PyRobotNoisyActuationSpec(
                amount=1.0,
                robot=robot,
                controller=controller,
                noise_multiplier=noise_multiplier,
            ),
        ),
        noisy_move_forward=habitat_sim.ActionSpec(
            "pyrobot_noisy_move_forward",
            habitat_sim.PyRobotNoisyActuationSpec(
                amount=1.0,
                robot=robot,
                controller=controller,
                noise_multiplier=noise_multiplier,
            ),
        ),
        noisy_turn_left=habitat_sim.ActionSpec(
            "pyrobot_noisy_turn_left",
            habitat_sim.PyRobotNoisyActuationSpec(
                amount=90.0,
                robot=robot,
                controller=controller,
                noise_multiplier=noise_multiplier,
            ),
        ),
        noisy_turn_right=habitat_sim.ActionSpec(
            "pyrobot_noisy_turn_right",
            habitat_sim.PyRobotNoisyActuationSpec(
                amount=90.0,
                robot=robot,
                controller=controller,
                noise_multiplier=noise_multiplier,
            ),
        ),
        move_backward=habitat_sim.ActionSpec(
            "move_backward", habitat_sim.ActuationSpec(amount=1.0)
        ),
        move_forward=habitat_sim.ActionSpec(
            "move_forward", habitat_sim.ActuationSpec(amount=1.0)
        ),
        turn_left=habitat_sim.ActionSpec(
            "turn_left", habitat_sim.ActuationSpec(amount=90.0)
        ),
        turn_right=habitat_sim.ActionSpec(
            "turn_right", habitat_sim.ActuationSpec(amount=90.0)
        ),
    )
    agent = habitat_sim.Agent(scene_graph.get_root_node().create_child(), agent_config)

    for base_action in {act.replace("noisy_", "") for act in agent_config.action_space}:
        state = agent.state
        state.rotation = qt.quaternion(1, 0, 0, 0)
        agent.state = state
        agent.act(base_action)
        base_state = agent.state

        delta_translations = []
        delta_rotations = []
        for _ in range(300):
            agent.state = state
            agent.act(f"noisy_{base_action}")
            noisy_state = agent.state

            delta_translations.append(_delta_translation(base_state, noisy_state))
            delta_rotations.append(_delta_rotation(base_state, noisy_state))

        delta_translations_arr = np.stack(delta_translations)
        delta_rotations_arr = np.stack(delta_rotations)
        if "move" in base_action:
            noise_model = pyrobot_noise_models[robot][controller].linear_motion
        else:
            noise_model = pyrobot_noise_models[robot][controller].rotational_motion
        EPS = 5e-2
        assert (
            np.linalg.norm(
                noise_model.linear.mean * noise_multiplier
                - np.abs(delta_translations_arr.mean(0))
            )
            < EPS
        )
        assert (
            np.linalg.norm(
                noise_model.rotation.mean * noise_multiplier
                - np.abs(delta_rotations_arr.mean(0))
            )
            < EPS
        )

        assert (
            np.linalg.norm(
                noise_model.linear.cov * noise_multiplier
                - np.diag(delta_translations_arr.std(0) ** 2)
            )
            < EPS
        )
        assert (
            np.linalg.norm(
                noise_model.rotation.cov * noise_multiplier
                - (delta_rotations_arr.std(0) ** 2)
            )
            < EPS
        )
