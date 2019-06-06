#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import itertools
import json
import os.path as osp

import numpy as np
import pytest
import quaternion

import habitat_sim
import habitat_sim.bindings as hsim
import habitat_sim.errors
import habitat_sim.utils
from examples.settings import make_cfg

_test_scenes = [
    osp.abspath(
        osp.join(
            osp.dirname(__file__),
            "../data/scene_datasets/mp3d/17DRP5sb8fy/17DRP5sb8fy.glb",
        )
    ),
    osp.abspath(
        osp.join(
            osp.dirname(__file__),
            "../data/scene_datasets/habitat-test-scenes/skokloster-castle.glb",
        )
    ),
    osp.abspath(
        osp.join(
            osp.dirname(__file__),
            "../data/scene_datasets/habitat-test-scenes/van-gogh-room.glb",
        )
    ),
]


@pytest.mark.gfxtest
@pytest.mark.parametrize(
    "scene,has_sem,sensor_type",
    list(
        itertools.product(
            _test_scenes[0:1],
            [True],
            ["color_sensor", "depth_sensor", "semantic_sensor"],
        )
    )
    + list(
        itertools.product(_test_scenes[1:], [False], ["color_sensor", "depth_sensor"])
    ),
)
def test_sensors(scene, has_sem, sensor_type, sim, make_cfg_settings):
    if not osp.exists(scene):
        pytest.skip("Skipping {}".format(scene))

    make_cfg_settings = {k: v for k, v in make_cfg_settings.items()}
    make_cfg_settings["semantic_sensor"] = has_sem
    make_cfg_settings["scene"] = scene
    sim.reconfigure(make_cfg(make_cfg_settings))
    with open(
        osp.abspath(
            osp.join(
                osp.dirname(__file__),
                "gt_data",
                "{}-state.json".format(osp.basename(osp.splitext(scene)[0])),
            )
        ),
        "r",
    ) as f:
        render_state = json.load(f)
        state = habitat_sim.AgentState()
        state.position = render_state["pos"]
        state.rotation = habitat_sim.utils.quat_from_coeffs(render_state["rot"])

    sim.initialize_agent(0, state)
    obs = sim.step("move_forward")

    assert sensor_type in obs, f"{sensor_type} not in obs"

    gt = np.load(
        osp.abspath(
            osp.join(
                osp.dirname(__file__),
                "gt_data",
                "{}-{}.npy".format(osp.basename(osp.splitext(scene)[0]), sensor_type),
            )
        )
    )

    # Different GPUs and different driver version will produce slightly different images
    assert np.linalg.norm(
        obs[sensor_type].astype(np.float) - gt.astype(np.float)
    ) < 1.5e-2 * np.linalg.norm(gt.astype(np.float)), f"Incorrect {sensor_type} output"
