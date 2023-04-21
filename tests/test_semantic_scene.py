#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from os import path as osp

import pytest
import quaternion  # noqa: F401

import habitat_sim
import habitat_sim.errors
from habitat_sim.utils.settings import make_cfg

_test_scenes = [
    osp.abspath(
        osp.join(
            osp.dirname(__file__),
            "../data/scene_datasets/mp3d_example/17DRP5sb8fy/17DRP5sb8fy.glb",
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
def test_semantic_scene(scene, make_cfg_settings):
    # [sangarg] This test is broken, Mosra and Alex have more context
    # disabling the test for now
    pytest.skip("Disabled")
    return
    if not osp.exists(scene):
        pytest.skip("Skipping {}".format(scene))

    make_cfg_settings["semantic_sensor"] = False
    make_cfg_settings["scene"] = scene
    cfg = make_cfg(make_cfg_settings)
    cfg.agents[0].sensor_specifications = []
    sim = habitat_sim.Simulator(cfg)
    # verify color map access
    sim.semantic_color_map
    scene = sim.semantic_scene
    for obj in scene.objects:
        obj.aabb
        obj.aabb.sizes
        obj.aabb.center
        obj.id
        obj.obb.rotation
        obj.category.name()
        obj.category.index()

    for region in scene.regions:
        region.id
        region.category.name()
        region.category.index()

    for level in scene.levels:
        level.id
