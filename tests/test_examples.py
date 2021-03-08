import itertools
from os import path as osp

import pytest

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
@pytest.mark.parametrize(
    "args",
    [
        ("examples/tutorials/stereo_agent.py", "--no-display"),
        ("examples/tutorials/lighting_tutorial.py", "--no-show-images"),
        ("examples/tutorials/new_actions.py",),
        (
            "examples/tutorials/nb_python/rigid_object_tutorial.py",
            "--no-show-video",
            "--no-make-video",
        ),
        (
            "examples/tutorials/nb_python/ECCV_2020_Navigation.py",
            "--no-make-video",
            "--no-display",
        ),
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
        (
            "examples/tutorials/nb_python/replay_tutorial.py",
            "--no-show-video",
            "--no-make-video",
        ),
        ("examples/tutorials/semantic_id_tutorial.py", "--no-show-images"),
    ],
)
def test_example_modules(args):
    run_main_subproc(args)


@pytest.mark.gfxtest
@pytest.mark.skipif(
    not osp.exists("data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"),
    reason="Requires the habitat-test-scenes",
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
)
def test_example_script(args):
    run_main_subproc(args)
