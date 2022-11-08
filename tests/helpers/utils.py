# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import multiprocessing
import runpy
import sys
from os import path as osp


def run_main(*args):
    # patch sys.args
    sys.argv = list(args)
    target = args[0]
    # run_path has one difference with invoking Python from command-line:
    # if the target is a file (rather than a directory), it does not add its
    # parent directory to sys.path. Thus, importing other modules from the
    # same directory is broken unless sys.path is patched here
    if osp.isfile(target):
        sys.path.insert(0, osp.dirname(target))
    runpy.run_path(target, run_name="__main__")


def run_main_subproc(args):
    # This test needs to be done in its own process as there is a potentially for
    # an OpenGL context clash otherwise
    mp_ctx = multiprocessing.get_context("spawn")
    proc = mp_ctx.Process(target=run_main, args=args)
    proc.start()
    proc.join()
    assert proc.exitcode == 0


def simulate(sim, dt, get_observations=False):
    r"""Runs physics simulation at 60FPS for a given duration (dt) optionally collecting and returning sensor observations."""
    observations = []
    target_time = sim.get_world_time() + dt
    while sim.get_world_time() < target_time:
        sim.step_physics(1.0 / 60.0)
        if get_observations:
            observations.append(sim.get_sensor_observations())
    return observations
