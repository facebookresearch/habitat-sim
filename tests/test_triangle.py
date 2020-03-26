#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os.path as osp

import matplotlib.pyplot as plt
import numpy as np
import pytest
import quaternion

import habitat_sim
import habitat_sim.errors
from examples.settings import make_cfg
from habitat_sim.agent import AgentState


def test_triangle_id(make_cfg_settings):
    # if not osp.exists(scene):
    #     pytest.skip("Skipping {}".format(scene))

    make_cfg_settings = {k: v for k, v in make_cfg_settings.items()}
    make_cfg_settings["triangle_sensor"] = True
    # make_cfg_settings["scene"] = scene
    cfg = make_cfg(make_cfg_settings)
    print("made config")
    sim = habitat_sim.Simulator(cfg)
    print(sim.config.agents[0].sensor_specifications[3].sensor_type)
    print(sim.config.agents[0].sensor_specifications[3].sensor_subtype)
    print(sim.config.agents[0].sensor_specifications[3].uuid)
    print("made sim")

    # pos = np.array([-0.3,0,3.2])
    # new_state = AgentState()
    ##new_state.position = pos
    # sim.agents[0].set_state(new_state)
    # obs = sim.get_sensor_observations()
    # for i in range(40):
    #     obs = sim.step("turn_right")
    #     tri = obs["triangle_sensor"]
    #     rgb = obs["color_sensor"]
    #     sem = obs["semantic_sensor"]
    #     print(f"iter: {i}, tri values: {np.unique(tri)}, sem vals: {np.unique(sem)}")
    obs = sim.reset()
    rgb = obs["color_sensor"]
    tri = obs["triangle_sensor"]
    sem = obs["semantic_sensor"]

    # plt.imshow(rgb)
    # plt.show()
    print(np.unique(tri))
    print(np.unique(sem))

    # assert np.average(triangle1) == np.average(triangle2)
    assert False
