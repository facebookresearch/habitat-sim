#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from os import path as osp
from typing import List

import magnum as mn
import pytest

import habitat_sim
import habitat_sim.errors
from habitat_sim.utils.settings import default_sim_settings, make_cfg

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

TEST_SCENE_DATASET = osp.abspath(
    osp.join(
        osp.dirname(__file__),
        "../data/test_assets/dataset_tests/dataset_0/test_dataset_0.scene_dataset_config.json",
    )
)


def test_semantic_regions():
    ## load scene dataset
    cfg_settings = default_sim_settings.copy()
    cfg_settings["scene_dataset_config_file"] = TEST_SCENE_DATASET
    cfg_settings["scene"] = "dataset_test_scene"
    hab_cfg = make_cfg(cfg_settings)
    with habitat_sim.Simulator(hab_cfg) as sim:
        semantic_scene = sim.semantic_scene
        assert semantic_scene != None
        regions = semantic_scene.regions
        assert regions != None
        assert len(regions) == 2

        # Build a list of points within the region
        hit_test_points: List[mn.Vector3] = 6 * [None]
        hit_test_points[0] = mn.Vector3(-5.1, 0.0, 0.01)
        hit_test_points[1] = mn.Vector3(-5.1, 1.0, 0.01)
        hit_test_points[2] = mn.Vector3(-5.1, -1.0, 0.01)
        hit_test_points[3] = mn.Vector3(-24.9, 0.0, 0.01)
        hit_test_points[4] = mn.Vector3(-15.1, 0.0, 9.9)
        hit_test_points[5] = mn.Vector3(-15.1, 0.0, -9.9)
        # Build a list of points outside the region
        miss_test_points = 6 * [None]
        miss_test_points[0] = mn.Vector3(0.0, 0.0, 0.01)
        miss_test_points[1] = mn.Vector3(-6.0, 0.0, 10.01)
        miss_test_points[2] = mn.Vector3(-5.1, -2.1, 0.01)
        miss_test_points[3] = mn.Vector3(-5.1, 2.1, 0.01)
        miss_test_points[4] = mn.Vector3(-15.1, -1.5, 10.01)
        miss_test_points[5] = mn.Vector3(-15.1, 1.5, -10.01)

        # Check region construction
        # negative X region
        region = regions[0]
        assert region.id == "test_region_negativeX"
        assert len(region.poly_loop_points) == 6
        assert region.extrusion_height == 4.0
        assert region.floor_height == -2.0

        regionCat = region.category
        assert regionCat.name() == "bedroom"
        assert regionCat.index() == 0
        # verify containment
        assert region.contains(hit_test_points[0])
        assert region.contains(hit_test_points[1])
        assert region.contains(hit_test_points[2])
        assert region.contains(hit_test_points[3])
        assert region.contains(hit_test_points[4])
        assert region.contains(hit_test_points[5])
        # verify non-containment
        assert not region.contains(miss_test_points[0])
        assert not region.contains(miss_test_points[1])
        assert not region.contains(miss_test_points[2])
        assert not region.contains(miss_test_points[3])
        assert not region.contains(miss_test_points[4])
        assert not region.contains(miss_test_points[5])

        # check batch containment routines
        for point in hit_test_points:
            assert 0 in semantic_scene.get_regions_for_point(point)
        for point in miss_test_points:
            assert 0 not in semantic_scene.get_regions_for_point(point)

        regions_weights = semantic_scene.get_regions_for_points(
            hit_test_points + miss_test_points
        )
        assert regions_weights[0][0] == 0
        assert regions_weights[0][1] >= 0.49  # half or more points contained

        # positive X region
        region = regions[1]
        assert region.id == "test_region_positiveX"
        assert len(region.poly_loop_points) == 6
        assert region.extrusion_height == 4.0
        assert region.floor_height == -2.0

        regionCat = region.category
        assert regionCat.name() == "bathroom"
        assert regionCat.index() == 1
        # verify containment
        assert region.contains(-1 * hit_test_points[0])
        assert region.contains(-1 * hit_test_points[1])
        assert region.contains(-1 * hit_test_points[2])
        assert region.contains(-1 * hit_test_points[3])
        assert region.contains(-1 * hit_test_points[4])
        assert region.contains(-1 * hit_test_points[5])
        # verify non-containment
        assert not region.contains(-1 * miss_test_points[0])
        assert not region.contains(-1 * miss_test_points[1])
        assert not region.contains(-1 * miss_test_points[2])
        assert not region.contains(-1 * miss_test_points[3])
        assert not region.contains(-1 * miss_test_points[4])
        assert not region.contains(-1 * miss_test_points[5])

        # mix the bathroom and kitchen points
        mixed_points: List[mn.Vector3] = list(hit_test_points[1:]) + [
            (-1 * p) for p in hit_test_points
        ]  # add one less to create imbalance
        regions_weights = semantic_scene.get_regions_for_points(mixed_points)
        assert regions_weights[0][0] == 1  # bathroom with more points comes first
        assert (
            regions_weights[0][1] >= 0.51
        )  # more than half points contained in bathroom
        assert regions_weights[1][0] == 0  # kitchen with fewer points is second
        assert (
            abs(regions_weights[1][1] - (1.0 - regions_weights[0][1])) < 0.001
        )  # kitchen should have the remainder of total percent


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
        obj.aabb.size
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
