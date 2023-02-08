#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


import importlib
import os
from io import StringIO
from unittest.mock import patch

import pytest

from habitat_sim.utils import profiling_utils

_ENV_VAR_NAME = "HABITAT_PROFILING"

# skip all tests if torch not installed
torch = pytest.importorskip("torch")

# skip specific tests if torch does not have cuda available
test_requires_torch_cuda = pytest.mark.skipif(
    not torch.cuda.is_available(),
    reason="Torch not installed with CUDA support so skipping test",
)


# Based on the env var, reloading the profiling_utils module should set
# profiling_utils._enable_profiling to True or False.
@test_requires_torch_cuda
def test_env_var_enable():
    # test with env var not set
    os.environ.pop(_ENV_VAR_NAME, None)
    importlib.reload(profiling_utils)
    assert not profiling_utils._enable_profiling
    # We also call range_push/range_pop to verify they run without error.
    profiling_utils.range_push("test, env var not set")
    assert profiling_utils._helper.range_depth == 0
    profiling_utils.range_pop()

    # test with env var set to "0". This is equivalent to
    # `export HABITAT_PROFILING=0` on the command line.
    os.environ[_ENV_VAR_NAME] = "0"
    importlib.reload(profiling_utils)
    assert not profiling_utils._enable_profiling
    profiling_utils.range_push("test, HABITAT_PROFILING='0'")
    assert profiling_utils._helper.range_depth == 0
    profiling_utils.range_pop()

    # test with env var set to "1"
    os.environ[_ENV_VAR_NAME] = "1"
    importlib.reload(profiling_utils)
    assert profiling_utils._enable_profiling
    profiling_utils.range_push("test, HABITAT_PROFILING=True")
    assert profiling_utils._helper.range_depth == 1
    profiling_utils.range_pop()


# Create nested ranges and verify the code runs without error.
@test_requires_torch_cuda
def test_nested_range_push_pop():
    os.environ[_ENV_VAR_NAME] = "1"
    importlib.reload(profiling_utils)

    assert profiling_utils._helper.range_depth == 0
    profiling_utils.range_push("A")
    profiling_utils.range_push("B")
    profiling_utils.range_push("C")
    assert profiling_utils._helper.range_depth == 3
    profiling_utils.range_pop()
    profiling_utils.range_pop()
    profiling_utils.range_pop()
    assert profiling_utils._helper.range_depth == 0


# Create ranges via RangeContext and verify the code runs without error.
@test_requires_torch_cuda
def test_range_context():
    os.environ[_ENV_VAR_NAME] = "1"
    importlib.reload(profiling_utils)

    with profiling_utils.RangeContext("A"):
        assert profiling_utils._helper.range_depth == 1
    assert profiling_utils._helper.range_depth == 0

    @profiling_utils.RangeContext("B")
    def my_example_profiled_function():
        pass

    my_example_profiled_function()
    assert profiling_utils._helper.range_depth == 0

    with profiling_utils.RangeContext("C"):
        assert profiling_utils._helper.range_depth == 1
        my_example_profiled_function()
        assert profiling_utils._helper.range_depth == 1
        with profiling_utils.RangeContext("D"):
            assert profiling_utils._helper.range_depth == 2
            my_example_profiled_function()
        assert profiling_utils._helper.range_depth == 1
    assert profiling_utils._helper.range_depth == 0


# Use configure() to capture a desired range of train steps.
def test_configure_and_on_start_step():
    os.environ[_ENV_VAR_NAME] = "1"
    importlib.reload(profiling_utils)

    # Use mock range_push/range_pop. This test only looks to confirm that these
    # functions get called at the right time.
    def fake_range_push(msg):
        print("range_push " + msg)

    def fake_range_pop():
        print("range_pop")

    with patch("sys.stdout", new=StringIO()) as fake_out, patch(
        "habitat_sim.utils.profiling_utils.range_push", new=fake_range_push
    ), patch("habitat_sim.utils.profiling_utils.range_pop", new=fake_range_pop):
        # Capture train steps 2 through 6.
        profiling_utils.configure(capture_start_step=2, num_steps_to_capture=5)
        for step in range(8):
            profiling_utils.on_start_step()  # Mark start of train step
            print("step {}".format(step))

        # Expect the capture range to span steps 2 through 6.
        expected_out = "step 0\nstep 1\nrange_push habitat_capture_range\nstep 2\nstep 3\nstep 4\nstep 5\nstep 6\nrange_pop\nstep 7\n"
        assert fake_out.getvalue() == expected_out
