#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


import argparse
import csv
import distutils

import demo_runner as dr

parser = argparse.ArgumentParser(
    description="Running AB test on simulator", add_help=True
)
parser.add_argument("--scene", type=str, default=dr.default_sim_settings["scene"])
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
    help="Whether to enable physics (kinematic by default or dynamics if installed with bullet) during the test or not.",
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

parser.add_argument("--csv", type=str, help="csv output file")
parser.add_argument(
    "--speedup", action="store_true", help="display speedup instead of percent change"
)


def get_percent_diff_str(test_val, control_val):
    return f"{((test_val - control_val) / control_val) * 100.0:.1f}%"


def get_speedup_str(test_val, control_val):
    return f"{test_val / control_val:.1f}x"


def seconds_to_ms(seconds):
    return seconds * 1000.0


def print_metric(
    performance_data,
    resolutions,
    title_list,
    metric="fps",
    comparison_label_generator=None,
    metric_transformer=None,
):
    for nproc, performance in performance_all.items():
        header = f" Performance ({metric}) NPROC={nproc} "
        print(f"{header:=^100}")
        title = "Resolution".ljust(16)
        title += "".join(t.ljust(24) for t in title_list)
        print(title)
        # break down by resolutions
        for resolution, perf in zip(resolutions, performance):
            row = f"{resolution} x {resolution}".ljust(16)
            # break down by benchmark items
            for t in title_list:
                control_metric = perf[t][dr.ABTestGroup.CONTROL][metric]
                test_metric = perf[t][dr.ABTestGroup.TEST][metric]
                comparison_str = comparison_label_generator(test_metric, control_metric)
                if metric_transformer:
                    control_metric = metric_transformer(control_metric)
                    test_metric = metric_transformer(test_metric)

                row += f"{control_metric:6.1f}/{test_metric:6.1f} ({comparison_str:>6})"
            print(row)
        print(f"{' END ':=^100}")


def get_csv_data(
    performance_all, resolutions, title_list, metrics=None, metric_transformer=None
):
    if metrics is None:
        metrics = ["fps"]
    if metric_transformer is None:
        metric_transformer = {}
    fields = ["num_procs", "resolution", "sensor_types"]
    for metric in metrics:
        fields.append(f"{metric}_control")
        fields.append(f"{metric}_test")
    rows = []

    for nproc, performance in performance_all.items():
        for resolution, perf in zip(resolutions, performance):
            for t in title_list:
                control_perf = perf[t][dr.ABTestGroup.CONTROL]
                test_perf = perf[t][dr.ABTestGroup.TEST]
                row = dict(num_procs=nproc, resolution=resolution, sensor_types=t)
                for metric in metrics:
                    control_metric = control_perf[metric]
                    test_metric = test_perf[metric]
                    if metric_transformer and metric in metric_transformer:
                        control_metric = metric_transformer[metric](control_metric)
                        test_metric = metric_transformer[metric](test_metric)
                    row[f"{metric}_test"] = test_metric
                    row[f"{metric}_control"] = control_metric
                rows.append(row)

    return rows, fields


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

if args.feature not in default_settings.keys():
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
    f"==== feature {args.feature}, control value: {control_val}, test value: {test_val} ===="
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
        resolution_label = f"{resolution} x {resolution}"
        per_resolution_perf = {}
        for key, value in benchmark_items.items():
            per_group_perf = {}
            for g in dr.ABTestGroup:
                demo_runner = dr.DemoRunner(default_settings, dr.DemoRunnerType.AB_TEST)
                run_label = f"(nprocs={nprocs}, resolution={resolution_label}, sensors={key}, group={g.name})"
                print(f"{f' Starting run {run_label} ':-^100}")
                settings = default_settings.copy()
                settings.update(value)
                # set new value before the test group run
                if g == dr.ABTestGroup.TEST:
                    settings[args.feature] = test_val
                per_group_perf[g] = demo_runner.benchmark(settings, g)
                result = f" FPS {run_label}: {per_group_perf[g]['fps']:.1f} "
                print(f"{result:-^100}")
            if collect_title_list:
                title_list.append(key)
            per_resolution_perf[key] = per_group_perf
        collect_title_list = False
        performance.append(per_resolution_perf)

    performance_all[nprocs] = performance

comparison_label_generator = get_speedup_str if args.speedup else get_percent_diff_str
print_metric(
    performance_all,
    resolutions,
    title_list,
    metric="fps",
    comparison_label_generator=comparison_label_generator,
)
print_metric(
    performance_all,
    resolutions,
    title_list,
    metric="frame_time",
    metric_transformer=seconds_to_ms,
    comparison_label_generator=comparison_label_generator,
)
if args.enable_physics:
    print_metric(performance_all, resolutions, title_list, metric="avg_sim_step_time")

if args.csv:
    with open(args.csv, "w", newline="") as csv_file:
        print(f"Writing csv results to {args.csv}")
        metrics = ["fps"]
        if args.enable_physics:
            metrics.append("avg_sim_step_time")
        rows, fields = get_csv_data(
            performance_all,
            resolutions,
            title_list,
            metrics=metrics,
            metric_transformer={"avg_sim_step_time": seconds_to_ms},
        )
        writer = csv.DictWriter(csv_file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
