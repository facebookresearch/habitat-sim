#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import habitat_sim.bindings as hsim
import habitat_sim
import habitat_sim.utils
import habitat_sim.errors
import numpy as np
import quaternion
import pytest
import os.path as osp


_test_scene = osp.abspath(
    osp.join(osp.dirname(__file__), "17DRP5sb8fy", "17DRP5sb8fy.glb")
)
hasdata = pytest.mark.skipif(
    not osp.exists(_test_scene),
    reason="Could not find test scene.  Test expects mp3d scene 17DRP5sb8fy to be at '/path/to/tests/17DRP5sb8fy'",
)


@hasdata
@pytest.mark.gfxtest
@pytest.mark.parametrize(
    "sensor_type", ["color_sensor", "depth_sensor", "semantic_sensor"]
)
def test_sensors(sensor_type, sim):
    agent_state = habitat_sim.AgentState()
    sim.initialize_agent(0, agent_state)

    obs = sim.step("move_forward")

    assert sensor_type in obs, f"{sensor_type} not in obs"

    gt = np.load(
        osp.abspath(osp.join(osp.dirname(__file__), "gt_data", f"{sensor_type}.npy"))
    )
    if sensor_type == "semantic_sensor":
        assert np.all(obs[sensor_type] == gt), f"Incorrect {sensor_type} output"
    else:
        # Different GPUs and different driver version will produce slightly different
        # images for RGB and depth, so use a big tolerance
        assert np.linalg.norm(
            obs[sensor_type].astype(np.float) - gt.astype(np.float)
        ) < 5e-3 * np.linalg.norm(
            gt.astype(np.float)
        ), f"Incorrect {sensor_type} output"
