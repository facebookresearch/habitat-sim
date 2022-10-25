#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
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

Example usage for capturing a certain range of train steps (from step 200 to
step 300):

@profiling_utils.RangeContext("train")
def train():
    profiling_utils.configure(capture_start_step=200, num_steps_to_capture=100)
    initialize_training()
    for _ in range(10000)):
        profiling_utils.on_start_step()
        with profiling_utils.RangeContext("collect_rollout_steps"):
            collect_rollout_steps()
        with profiling_utils.RangeContext("update_agent"):
            update_agent()

Example of capturing an Nsight Systems profile using your capture range:
export HABITAT_PROFILING=1
export NSYS_NVTX_PROFILER_REGISTER_ONLY=0  # required when using capture range
path/to/nvidia/nsight-systems/bin/nsys profile --sample=none --trace=cuda,nvtx
--trace-fork-before-exec=true ---trace=nvtx --capture-range=nvtx -p
"habitat_capture_range" --output=my_profile python my_program.py
# look for my_profile.qdrep in working directory
"""
import os
from contextlib import ContextDecorator

import attr

from habitat_sim.logging import logger

_env_var = os.environ.get("HABITAT_PROFILING", "0")
_enable_profiling = _env_var != "0"
if _enable_profiling:
    logger.info("HABITAT_PROFILING={}".format(_env_var))
    logger.info("profiling_utils.py range_push/range_pop annotation is enabled")
    from torch.cuda import nvtx


@attr.s(auto_attribs=True)
class _ProfilingHelper:
    step_count: int = -1
    capture_start_step: int = -1
    capture_end_step: int = -1
    range_depth: int = 0


_helper = _ProfilingHelper()


def configure(capture_start_step=-1, num_steps_to_capture=-1):
    r"""Configures profiling_utils to push a "habitat_capture_range" range to
    span the desired train steps. See also on_start_step(). See
    profiling_utils.py file-level documentation for example usage with Nvidia
    Nsight.

    Your program can have many processes that call range_push/range_pop, but
    only your main process (the one executing your train loop) should call
    configure() and on_start_step().

    If capture_start_step == 0, then capturing starts at the start of step 0
    (after initialization code). If capture_start_step == -1, then capturing
    starts immediately after your call to configure() and includes any
    initialization code that runs prior to step 0. If num_steps_to_capture ==
    -1, the range won't be popped and capture will continue until your program
    terminates.
    """

    if not _enable_profiling:
        return

    assert _helper.step_count == -1  # must call configure before on_start_step
    _helper.capture_start_step = capture_start_step

    if _helper.capture_start_step == -1:
        range_push("habitat_capture_range")

    assert num_steps_to_capture == -1 or num_steps_to_capture > 0
    if num_steps_to_capture == -1:
        _helper.capture_end_step = -1
    elif _helper.capture_start_step == -1:
        _helper.capture_end_step = num_steps_to_capture
    else:
        _helper.capture_end_step = capture_start_step + num_steps_to_capture


def on_start_step():
    r"""Marks the start of a train step. This is required for profiling_utils
    internal bookkeeping. See also configure().
    """

    if not _enable_profiling:
        return

    _helper.step_count += 1

    if _helper.step_count == _helper.capture_start_step:
        range_push("habitat_capture_range")
    elif _helper.step_count == _helper.capture_end_step:
        range_pop()


def range_push(msg: str) -> None:
    r"""Annotates the start of a range for profiling. Requires HABITAT_PROFILING
    environment variable to be set, otherwise the function is a no-op. Pushes a
    range onto a stack of nested ranges. Every range_push should have a
    corresponding range_pop. Attached profilers can capture the time spent in
    ranges."
    """
    if not _enable_profiling:
        return

    nvtx.range_push(msg)
    _helper.range_depth += 1
    max_depth = 64
    # In practice, there is little need to go deeper than 5 or 10. By asserting
    # here, we'll catch improper range_push/range_pop usage. Specifically,
    # we'll (eventually) catch an unmatched range_push.
    assert _helper.range_depth < max_depth


def range_pop() -> None:
    r"""Annotates the end of a range for profiling. See also range_push."""
    if not _enable_profiling:
        return

    assert _helper.range_depth > 0
    nvtx.range_pop()
    _helper.range_depth -= 1


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
