#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


import importlib
import os

from habitat_sim.utils import profiling_utils

env_var_name = "HABITAT_PROFILING"

# Based on the env var, reloading the profiling_utils module should set
# profiling_utils.enable_profiling to True or False.
def test_env_var_enable():

    # test with env var not set
    os.environ.pop(env_var_name, None)
    importlib.reload(profiling_utils)
    assert not profiling_utils.enable_profiling
    # We also call range_push/range_pop to verify they run without error.
    profiling_utils.range_push("test, env var not set")
    profiling_utils.range_pop()

    # test with env var set to "0". This is equivalent to
    # `export HABITAT_PROFILING=0` on the command line.
    os.environ[env_var_name] = "0"
    importlib.reload(profiling_utils)
    assert not profiling_utils.enable_profiling
    profiling_utils.range_push("test, HABITAT_PROFILING='0'")
    profiling_utils.range_pop()

    # test with env var set to "1"
    os.environ[env_var_name] = "1"
    importlib.reload(profiling_utils)
    assert profiling_utils.enable_profiling
    profiling_utils.range_push("test, HABITAT_PROFILING=True")
    profiling_utils.range_pop()


# Create nested ranges and verify the code runs without error.
def test_nested_range_push_pop():
    # Unfortunately, there isn't a way to verify correct behavior here. Ranges
    # get recorded in the Nvidia driver/profiler. Documentation for
    # torch.cuda.nvtx.range_push claims that it returns range stack depth, so I
    # hoped to use it as a behavior checker, but it seems to just return -1 or
    # -2 in practice.

    os.environ[env_var_name] = "1"
    importlib.reload(profiling_utils)

    profiling_utils.range_push("A")
    profiling_utils.range_push("B")
    profiling_utils.range_push("C")
    profiling_utils.range_pop()
    profiling_utils.range_pop()
    profiling_utils.range_pop()


# Create ranges via RangeContext and verify the code runs without error.
def test_range_context():

    os.environ[env_var_name] = "1"
    importlib.reload(profiling_utils)

    with profiling_utils.RangeContext("A"):
        pass

    @profiling_utils.RangeContext("B")
    def my_example_profiled_function():
        pass

    my_example_profiled_function()

    with profiling_utils.RangeContext("C"):
        my_example_profiled_function()
        with profiling_utils.RangeContext("D"):
            my_example_profiled_function()
