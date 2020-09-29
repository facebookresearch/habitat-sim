#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
r"""Helper functions for profiling. Profilers like Nvidia Nsight can capture the
time spent in annotated ranges.

Example usage for marking ranges. For convenience, there are three syntax
options; they're all functionally equivalent:

# use the decorator to mark an entire function as a range
@profiling_utils.RangeContext("my_function")
def my_function():

    # use range_push/range_pop explicitly
    range_push("create and sort widgets")
    create_widgets()
    sort_widgets()
    range_pop()

    # use RangeContext and a with statement
    with profiling_utils.RangeContext("print and save widgets"):
        print_widgets()
        save_widgets()

Example of capturing an Nsight Systems profile:
apt-get install nvidia-nsight
export HABITAT_PROFILING=1
path/to/nvidia/nsight-systems/bin/nsys profile --sample=none --trace=cuda,nvtx
--trace-fork-before-exec=true --output=my_profile python my_program.py
# look for my_profile.qdrep in working directory
"""
import os
from contextlib import ContextDecorator

from habitat_sim.logging import logger

env_var = os.environ.get("HABITAT_PROFILING", "0")
enable_profiling = env_var != "0"
if enable_profiling:
    logger.info("HABITAT_PROFILING={}".format(env_var))
    logger.info("profiling_utils.py range_push/range_pop annotation is enabled")
    from torch.cuda import nvtx


def range_push(msg: str) -> None:
    r"""Annotates the start of a range for profiling. Requires HABITAT_PROFILING
    environment variable to be set, otherwise the function is a no-op. Pushes a
    range onto a stack of nested ranges. Every range_push should have a
    corresponding range_pop. Attached profilers can capture the time spent in
    ranges."
    """
    if enable_profiling:
        nvtx.range_push(msg)


def range_pop() -> None:
    r"""Annotates the end of a range for profiling. See also range_push."""
    if enable_profiling:
        nvtx.range_pop()


class RangeContext(ContextDecorator):
    r"""Annotate a range for profiling. Use as a function decorator or in a with
    statement. See also range_push.
    """

    def __init__(self, msg: str) -> None:
        self._msg = msg

    def __enter__(self) -> "RangeContext":
        range_push(self._msg)
        return self

    def __exit__(self, *exc) -> None:
        range_pop()
