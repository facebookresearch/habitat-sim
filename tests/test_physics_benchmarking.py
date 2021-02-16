from os import path as osp

import pytest

import utils


@pytest.mark.skipif(
    not osp.exists("src/deps/v-hacd"),
    reason="Requires VHACD to be built (try building with --vhacd)",
)
@pytest.mark.parametrize(
    "args",
    [
        (
            "examples/tutorials/physics_benchmarking.py",
            "--no-make-video",
            "--no-show-video",
        ),
    ],
)
def test_example_modules(args):
    utils.run_main_subproc(args)
