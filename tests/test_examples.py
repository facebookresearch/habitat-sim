# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import itertools
from os import path as osp

import pytest

import habitat_sim.bindings
from utils import run_main_subproc


def powerset(iterable):
    s = list(iterable)
    return itertools.chain.from_iterable(
        itertools.combinations(s, r) for r in range(len(s) + 1)
    )


@pytest.mark.gfxtest
@pytest.mark.skipif(
    not osp.exists("data/scene_datasets/habitat-test-scenes/skokloster-castle.glb")
    or not osp.exists("data/scene_datasets/habitat-test-scenes/van-gogh-room.glb"),
    reason="Requires the habitat-test-scenes",
)
@pytest.mark.skipif(
    not habitat_sim.bindings.built_with_bullet,
    reason="Bullet physics used for validation.",
)
@pytest.mark.parametrize(
    "args",
    [
        (
            "examples/tutorials/nb_python/ECCV_2020_Interactivity.py",
            "--no-make-video",
            "--no-display",
        ),
        (
            "examples/tutorials/nb_python/ECCV_2020_Advanced_Features.py",
            "--no-make-video",
            "--no-display",
        ),
    ],
)
def test_example_modules_required_bullet(args):
    run_main_subproc(args)


@pytest.mark.gfxtest
@pytest.mark.skipif(
    not osp.exists("data/scene_datasets/habitat-test-scenes/skokloster-castle.glb")
    or not osp.exists("data/scene_datasets/habitat-test-scenes/van-gogh-room.glb"),
    reason="Requires the habitat-test-scenes",
)
@pytest.mark.parametrize(
    "args",
    [
        (
            "examples/tutorials/nb_python/asset_viewer.py",
            "--no-show-video",
            "--no-make-video",
        ),
        ("examples/tutorials/stereo_agent.py", "--no-display"),
        ("examples/tutorials/lighting_tutorial.py", "--no-show-images"),
        (
            "examples/tutorials/nb_python/managed_rigid_object_tutorial.py",
            "--no-show-video",
            "--no-make-video",
        ),
        ("examples/tutorials/new_actions.py",),
        (
            "examples/tutorials/nb_python/ECCV_2020_Navigation.py",
            "--no-make-video",
            "--no-display",
        ),
        (
            "examples/tutorials/nb_python/replay_tutorial.py",
            "--no-show-video",
            "--no-make-video",
        ),
        ("examples/tutorials/semantic_id_tutorial.py", "--no-show-images"),
        ("examples/tutorials/async_rendering.py",),
    ],
)
def test_example_modules_optional_bullet(args):
    run_main_subproc(args)


@pytest.mark.skipif(
    not habitat_sim.bindings.built_with_bullet,
    reason="Bullet physics required for ReplicaCAD Articulated Objects.",
)
@pytest.mark.skipif(
    not osp.exists("data/replica_cad/"),
    reason="Requires ReplicaCAD dataset.",
)
@pytest.mark.parametrize(
    "args",
    [
        (
            "examples/tutorials/nb_python/ReplicaCAD_quickstart.py",
            "--no-show-video",
            "--no-make-video",
        )
    ],
)
def test_replica_cad_quickstart(args):
    run_main_subproc(args)


@pytest.mark.gfxtest
@pytest.mark.skipif(
    not osp.exists("data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"),
    reason="Requires the habitat-test-scenes",
)
@pytest.mark.skipif(
    not habitat_sim.bindings.built_with_bullet,
    reason="Bullet physics used for validation.",
)
@pytest.mark.parametrize(
    "args",
    [
        ["examples/example.py"] + list(p)
        for p in powerset(
            [
                "--compute_shortest_path",
                "--compute_action_shortest_path",
                "--enable_physics",
                "--semantic_sensor",
                "--depth_sensor",
                "--recompute_navmesh",
            ]
        )
        if not (("--compute_action_shortest_path" in p) and ("--enable_physics" in p))
    ],
    ids=str,
)
def test_example_script_with_bullet(args):
    run_main_subproc(args)


# Perform tests without physics enabled flag if no bullet, skip these if bullet exists
@pytest.mark.gfxtest
@pytest.mark.skipif(
    not osp.exists("data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"),
    reason="Requires the habitat-test-scenes",
)
@pytest.mark.skipif(
    habitat_sim.bindings.built_with_bullet,
    reason="",
)
@pytest.mark.parametrize(
    "args",
    [
        ["examples/example.py"] + list(p)
        for p in powerset(
            [
                "--compute_shortest_path",
                "--compute_action_shortest_path",
                "--semantic_sensor",
                "--depth_sensor",
                "--recompute_navmesh",
            ]
        )
    ],
    ids=str,
)
def test_example_script_no_bullet(args):
    run_main_subproc(args)
