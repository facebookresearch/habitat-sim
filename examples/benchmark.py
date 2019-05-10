#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


import argparse

import numpy as np

import demo_runner as dr

parser = argparse.ArgumentParser("Running benchmarks on simulator")
parser.add_argument("--scene", type=str, default="test.glb")
parser.add_argument(
    "--max_frames",
    type=int,
    default=2000,
    help="Max number of frames simulated."
    "Default or larger value is suggested for accurate results.",
)
parser.add_argument("--seed", type=int, default=1)
args = parser.parse_args()

default_settings = dr.default_sim_settings.copy()
default_settings["silent"] = True
default_settings["seed"] = args.seed

default_settings["save_png"] = False
default_settings["print_semantic_scene"] = False
default_settings["print_semantic_mask_stats"] = False
default_settings["compute_shortest_path"] = False
default_settings["compute_action_shortest_path"] = False

default_settings["max_frames"] = args.max_frames

demo_runner = dr.DemoRunner(default_settings, dr.DemoRunnerType.BENCHMARK)

benchmark_items = {
    "rgb": {},
    "rgbd": {"depth_sensor": True},
    "depth_only": {"color_sensor": False, "depth_sensor": True},
    "semantic_only": {"color_sensor": False, "semantic_sensor": True},
    "rgbd_semantic": {"depth_sensor": True, "semantic_sensor": True},
}

# resolutions = [128] # (debug)
resolutions = [128, 256, 512]

performance = []
for resolution in resolutions:
    default_settings["width"] = default_settings["height"] = resolution
    perf = {}
    for key, value in benchmark_items.items():
        print(" ---------------------- %s ------------------------ " % key)
        settings = default_settings.copy()
        settings.update(value)
        perf[key] = demo_runner.benchmark(settings).get("fps")
        print(
            " ====== FPS (%d x %d, %s): %0.1f ======"
            % (settings["width"], settings["height"], key, perf[key])
        )
    performance.append(perf)

print(" ================ Performance (FPS) ===========================================")
title = "Resolution "
for key, value in perf.items():
    title += "%15s" % key
print(title)
for idx in range(len(performance)):
    row = "%d x %d" % (resolutions[idx], resolutions[idx])
    for key, value in performance[idx].items():
        row += "%15.1f" % value
    print(row)
print(" ==============================================================================")
