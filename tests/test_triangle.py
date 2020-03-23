#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os.path as osp

import numpy as np
import pytest
import quaternion

import habitat_sim
import habitat_sim.errors
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
            "../data/scene_datasets/mp3d/ur6pFq6Qu1A/ur6pFq6Qu1A.glb",
        )
    ),
]


@pytest.mark.parametrize("scene", _test_scenes)
def test_triangle_id(scene, make_cfg_settings):
    if not osp.exists(scene):
        pytest.skip("Skipping {}".format(scene))

    make_cfg_settings = {k: v for k, v in make_cfg_settings.items()}
    make_cfg_settings["triangle_sensor"] = True
    make_cfg_settings["scene"] = scene
    cfg = make_cfg(make_cfg_settings)
    sim = habitat_sim.Simulator(cfg)

    obs = sim.reset()
    triangle1 = obs["triangle_sensor"]
    sim.step("move_forward")
    obs = sim.reset()
    triangle2 = obs["triangle_sensor"]
    assert np.average(triangle1) == np.average(triangle2)
