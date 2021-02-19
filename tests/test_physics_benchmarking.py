import pytest

import utils
from habitat_sim import vhacd_enabled


@pytest.mark.skipif(not vhacd_enabled, reason="Test requires vhacd")
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
