import pytest

import utils


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
