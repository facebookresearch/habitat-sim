#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


import argparse
import distutils
from distutils import util

import demo_runner as dr
import numpy as np

parser = argparse.ArgumentParser(
    description="Running AB test on simulator", add_help=True
)
parser.add_argument("--scene", type=str, default=dr.default_sim_settings["test_scene"])
parser.add_argument(
    "--max_frames",
    type=int,
    default=2000,
    help="Max number of frames simulated."
    "Default or larger value is suggested for accurate results.",
)
parser.add_argument(
    "--resolution",
    type=int,
    nargs="+",
    default=[128, 256, 512],
    help="Resolution r for frame (r x r).",
)
parser.add_argument(
    "--num_procs",
    type=int,
    nargs="+",
    default=[1, 3, 5],
    help="Number of concurrent processes.",
)
parser.add_argument(
    "--semantic_sensor",
    action="store_true",
    help="Whether to enable semantic sensor in the test.",
)
parser.add_argument("--seed", type=int, default=1)
parser.add_argument(
    "--enable_physics",
    action="store_true",
    help="Whether to enable phyiscs (kinematic by default or dynamics if installed with bullet) during the test or not.",
)
parser.add_argument(
    "--num_objects",
    type=int,
    default=10,
    help="Number of objects to spawn if enable_physics is true.",
)
parser.add_argument(
    "--test_object_index",
    type=int,
    default=0,
    help="Index the objects to spawn if enable_physics is true. -1 indicates random.",
)

parser.add_argument(
    "--feature",
    type=str,
    required=True,
    help="the feature that is to be tested. (it must be defined in default_sim_settings",
)

group = parser.add_mutually_exclusive_group(required=True)
group.add_argument(
    "-i", "--integer", action="store_true", help="the feature type is integer."
)
group.add_argument(
    "-f", "--float", action="store_true", help="the feature type is float."
)
group.add_argument(
    "-b", "--boolean", action="store_true", help="the feature type is boolean."
)
group.add_argument(
    "-s", "--string", action="store_true", help="the feature type is string."
)

parser.add_argument(
    "--control_value",
    type=str,
    default=argparse.SUPPRESS,
    help="the feature value in control group (default: default value in default_settings)",
)

parser.add_argument(
    "--test_value", type=str, required=True, help="the feature value in test group."
)

args = parser.parse_args()

control_val = None
if args.integer:
    test_val = int(args.test_value)
    if "control_value" in args:
        control_val = int(args.control_value)
elif args.float:
    test_val = float(args.test_value)
    if "control_value" in args:
        control_val = float(args.control_value)
elif args.boolean:
    test_val = distutils.util.strtobool(args.test_value)
    if "control_value" in args:
        control_val = distutils.util.strtobool(args.control_value)
elif args.string:
    test_val = args.test_value
    if "control_value" in args:
        control_val = args.control_value

default_settings = dr.default_sim_settings.copy()

if not (args.feature in default_settings.keys()):
    raise RuntimeError("Feature to be tested is not defined in default_sim_settings.")
if args.feature == "max_frames":
    raise RuntimeError("Feature cannot be the max_frames.")

default_settings["scene"] = args.scene
default_settings["silent"] = True
default_settings["seed"] = args.seed

default_settings["save_png"] = False
default_settings["print_semantic_scene"] = False
default_settings["print_semantic_mask_stats"] = False
default_settings["compute_shortest_path"] = False
default_settings["compute_action_shortest_path"] = False

default_settings["max_frames"] = args.max_frames

# set the control value into the default setting
if control_val != None:
    default_settings[args.feature] = control_val
else:
    control_val = default_settings[args.feature]
print(
    "==== feature %s, control value: %s, test value: %s ===="
    % (args.feature, control_val, test_val)
)

benchmark_items = {
    "rgb": {},
    "rgbd": {"depth_sensor": True},
    "depth_only": {"color_sensor": False, "depth_sensor": True},
}
if args.semantic_sensor:
    benchmark_items["semantic_only"] = {"color_sensor": False, "semantic_sensor": True}
    benchmark_items["rgbd_semantic"] = {"depth_sensor": True, "semantic_sensor": True}

if args.enable_physics:
    # TODO: cannot benchmark physics with no sensors as this won't create a renderer or load the scene.
    # benchmark_items["enable_physics_no_obs"] = {"color_sensor": False, "enable_physics": True}
    benchmark_items["phys_rgb"] = {"enable_physics": True}
    benchmark_items["phys_rgbd"] = {"depth_sensor": True, "enable_physics": True}
    default_settings["num_objects"] = args.num_objects
    default_settings["test_object_index"] = args.test_object_index

resolutions = args.resolution
nprocs_tests = args.num_procs

performance_all = {}
title_list = []
collect_title_list = True
for nprocs in nprocs_tests:
    default_settings["num_processes"] = nprocs
    performance = []
    for resolution in resolutions:
        default_settings["width"] = default_settings["height"] = resolution
        perf = {}
        for key, value in benchmark_items.items():
            for g in dr.ABTestGroup:
                demo_runner = dr.DemoRunner(default_settings, dr.DemoRunnerType.AB_TEST)
                print(" ---------------------- %s ------------------------ " % key)
                settings = default_settings.copy()
                settings.update(value)
                # set new value before the test group run
                if g == dr.ABTestGroup.TEST:
                    settings[args.feature] = test_val
                k = key + "_" + str(g)
                perf[k] = demo_runner.benchmark(settings, g)
                print(
                    " ====== FPS (%d x %d, %s): %0.1f ======"
                    % (settings["width"], settings["height"], k, perf[k].get("fps"))
                )
                if g == dr.ABTestGroup.CONTROL and collect_title_list:
                    title_list.append(key)
        collect_title_list = False
        performance.append(perf)

    performance_all[nprocs] = performance

for nproc, performance in performance_all.items():
    print(
        " ================ Performance (FPS) NPROC={} ===================================".format(
            nproc
        )
    )
    title = "Resolution "
    for t in title_list:
        title += "\t%-24s" % t
    print(title)
    # break down by resolutions
    for idx in range(len(performance)):
        row = "%d x %d" % (resolutions[idx], resolutions[idx])
        # break down by benchmark items
        for t in title_list:
            control_fps = performance[idx][t + "_" + str(dr.ABTestGroup.CONTROL)]["fps"]
            test_fps = performance[idx][t + "_" + str(dr.ABTestGroup.TEST)]["fps"]
            ratio = (float(test_fps) - float(control_fps)) / float(control_fps) * 100.0
            row += "\t%-6.1f/%-6.1f (%6.1f%%)" % (control_fps, test_fps, ratio)
        print(row)
    print(
        " =============================================================================="
    )

    # also print the average time per simulation step (including object perturbations)
    if args.enable_physics:
        print(
            " ================ Performance (step time: milliseconds) NPROC={} ===================================".format(
                nproc
            )
        )
        title = "Resolution "
        for key, value in perf.items():
            title += "\t%-10s" % key
        print(title)
        for idx in range(len(performance)):
            row = "%d x %d" % (resolutions[idx], resolutions[idx])
            for key, value in performance[idx].items():
                row += "\t%-8.2f" % (value.get("avg_sim_step_time") * 1000)
            print(row)
        print(
            " =============================================================================="
        )
