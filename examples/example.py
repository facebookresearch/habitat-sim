#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


import argparse

import numpy as np

import demo_runner as dr

parser = argparse.ArgumentParser()
parser.add_argument("--scene", type=str, default=dr.default_sim_settings["test_scene"])
parser.add_argument("--width", type=int, default=640)
parser.add_argument("--height", type=int, default=480)
parser.add_argument("--max_frames", type=int, default=1000)
parser.add_argument("--save_png", action="store_true")
parser.add_argument("--sensor_height", type=float, default=1.5)
parser.add_argument("--disable_color_sensor", action="store_true")
parser.add_argument("--semantic_sensor", action="store_true")
parser.add_argument("--depth_sensor", action="store_true")
parser.add_argument("--print_semantic_scene", action="store_true")
parser.add_argument("--print_semantic_mask_stats", action="store_true")
parser.add_argument("--compute_shortest_path", action="store_true")
parser.add_argument("--compute_action_shortest_path", action="store_true")
parser.add_argument("--seed", type=int, default=1)
parser.add_argument("--silent", action="store_true")
parser.add_argument("--test_fps_regression", type=int, default=0)
args = parser.parse_args()


def make_settings():
    settings = dr.default_sim_settings.copy()
    settings["max_frames"] = args.max_frames
    settings["width"] = args.width
    settings["height"] = args.height
    settings["scene"] = args.scene
    settings["save_png"] = args.save_png
    settings["sensor_height"] = args.sensor_height
    settings["color_sensor"] = not args.disable_color_sensor
    settings["semantic_sensor"] = args.semantic_sensor
    settings["depth_sensor"] = args.depth_sensor
    settings["print_semantic_scene"] = args.print_semantic_scene
    settings["print_semantic_mask_stats"] = args.print_semantic_mask_stats
    settings["compute_shortest_path"] = args.compute_shortest_path
    settings["compute_action_shortest_path"] = args.compute_action_shortest_path
    settings["seed"] = args.seed
    settings["silent"] = args.silent

    return settings


settings = make_settings()

demo_runner = dr.DemoRunner(settings, dr.DemoRunnerType.EXAMPLE)
perf = demo_runner.example()

print(" ============ Performance ======================== ")
print(
    " %d x %d, total time: %0.3f sec."
    % (settings["width"], settings["height"], perf["total_time"]),
    "FPS: %0.1f" % perf["fps"],
)
print(" ================================================= ")

assert perf["fps"] > args.test_fps_regression, (
    "FPS is below regression threshold: %0.1f < %0.1f"
    % (perf["fps"], args.test_fps_regression)
)
