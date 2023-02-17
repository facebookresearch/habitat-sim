#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
r"""Helper script to print summary timings for one or more profiles captured by
Nvidia Nsight containing NVTX events. The script reads all sqlite files in the
working directory. To add NVTX ranges to your program, see profiling_utils.py.

Exampe usage:

apt-get install nvidia-nsight
export HABITAT_PROFILING=1
# capture two profiles, named profile1.sqlite and profile2.sqlite
path/to/nvidia/nsight-systems/bin/nsys profile --sample=none --trace=nvtx
--trace-fork-before-exec=true --output=profile1 --export=sqlite python
my_program.py
path/to/nvidia/nsight-systems/bin/nsys profile --sample=none --trace=nvtx
--trace-fork-before-exec=true --output=profile2 --export=sqlite python
my_program.py
python habitat/utils/compare_profiles.py --relative

Example output, from a PPO train script that was annotated using
profiling_utils.py:
                                profile1.sqlite             profile2.sqlite
event name                        incl (ms)     excl (ms)
_worker_env                          87,102        10,843        -2,615          -933
train                                44,376         7,820          -942          -272
_worker_env recv                     37,993        37,993          -784          -784
train loop body                      36,555           264          -669           -12
_collect_rollout_step                28,733         1,990          -469           -75
wait_step                            19,079        19,079          -143          -143
habitat_simulator.py step            18,523        18,523          -109          -109
_worker_env send                     10,597        10,597          -414          -414
nav.py update_metric                  9,145         9,145          -375          -375
_update_agent                         7,559         1,357          -188            -9
act (run policy)                      5,750         5,750          -106          -106
evaluate_actions                      3,058         3,058           -95           -95
batch_obs                             1,915         1,915          -144          -144
before_step (for optimize)            1,681         1,681           -62           -62
backward (for surrogate loss)         1,463         1,463           -22           -22
"""
import argparse
import glob
import os
import sqlite3
from argparse import ArgumentParser, Namespace
from collections import defaultdict
from sqlite3 import Connection
from typing import Any, DefaultDict, Dict, List, Optional, Set, Union

import attr


@attr.s(auto_attribs=True)
class Event:
    """Python in-memory representation for an NVTX event from an sqlite
    database"""

    name: str
    thread_id: int
    start: int
    end: int


@attr.s(auto_attribs=True)
class SummaryItem:
    """Summary item, for accumulating time for a set of events."""

    time_exclusive: int = 0
    time_inclusive: int = 0
    count: int = 0


def get_sqlite_events(conn: Connection) -> List[Event]:
    """Parse an sqlite database containing an NVTX_EVENTS table and return a
    list of Events."""
    events: List[Event] = []

    # check if table exists
    cursor = conn.execute(
        "SELECT count(*) FROM sqlite_master WHERE type='table' AND name='NVTX_EVENTS'"
    )
    does_table_exist = cursor.fetchone()[0] > 0
    if not does_table_exist:
        return events

    # select events
    cursor = conn.execute("SELECT text, globalTid, start, end from NVTX_EVENTS")

    for row in cursor:
        events.append(Event(*row))

    return events


def create_summary_from_events(events: List[Event]) -> DefaultDict[str, SummaryItem]:
    """From a list of events, group by name and create summary items. Returns a
    dictionary of items keyed by name."""
    # sort by start time (ascending). For ties, sort by end time (descending). In this way,
    # a (shorter) child event that starts at the same time as a (longer) parent event
    # will occur later in the sort.
    events.sort(reverse=True, key=lambda event: event.end)
    events.sort(reverse=False, key=lambda event: event.start)

    items: DefaultDict[str, SummaryItem] = defaultdict(lambda: SummaryItem())

    for i, event in enumerate(events):
        item = items[event.name]

        event_duration = event.end - event.start
        item.time_inclusive += event_duration

        exclusive_duration = 0

        # iterate chronologically through later events. Our accumulated
        #  "exclusive duration" is time during which we aren't inside any
        #  overlapping, same-thread event ("child event").
        recent_exclusive_start_time: Optional[int] = event.start
        child_end_times: Set[int] = set()
        for j in range(i + 1, len(events) + 1):  # note one extra iteration
            other_event: Optional[Event] = None if j == len(events) else events[j]
            if other_event:
                if other_event.thread_id != event.thread_id:
                    continue
                if other_event.start > event.end:
                    other_event = None

            current_time = other_event.start if other_event else event.end

            if len(child_end_times):
                latest_child_end_time = max(child_end_times)

                # remove any child which ends prior to current_time
                child_end_times = set(
                    filter(lambda t: t > current_time, child_end_times)
                )

                if len(child_end_times) == 0:
                    recent_exclusive_start_time = latest_child_end_time
                else:
                    recent_exclusive_start_time = None

            # handle exclusive time leading up to current_time
            if recent_exclusive_start_time:
                assert recent_exclusive_start_time <= current_time
                exclusive_duration += current_time - recent_exclusive_start_time

            if other_event:
                child_end_times.add(other_event.end)
            else:
                break

        assert event_duration >= exclusive_duration
        item.time_exclusive += exclusive_duration
        item.count += 1

    return items


def _display_time_ms(time: int, args: Namespace, show_sign: bool = False) -> str:
    seconds_to_ms = 0.001
    return "{}{:,.0f}".format(
        "+" if time > 0 and show_sign else "",
        time / (args.source_time_units_per_second * seconds_to_ms),
    )


def print_summaries(
    summaries: Union[List[DefaultDict[str, SummaryItem]], List[DefaultDict[Any, Any]]],
    args: Namespace,
    labels: Optional[List[str]] = None,
) -> None:
    """Print a dictionary of summaries to stdout. See create_arg_parser for
    formatting options available in the args object. See also
    create_summary_from_events."""
    sort_by_exclusive = args.sort_by == "exclusive"
    print_relative_timings = args.relative

    if len(summaries) == 0:
        print("no summaries to print")
        return

    all_names_with_times: Dict[str, int] = {}
    max_name_len = 0
    for summary in summaries:
        for name in summary:
            all_names_with_times.setdefault(
                name,
                summary[name].time_exclusive
                if sort_by_exclusive
                else summary[name].time_inclusive,
            )
            max_name_len = max(max_name_len, len(name))

    if len(all_names_with_times.items()) == 0:
        print("All summaries are empty. The sqlite databases are missing NVTX events.")
        return

    all_names_with_times_list = list(all_names_with_times.items())
    # sort by time, decreasing
    all_names_with_times_list.sort(reverse=True, key=lambda x: x[1])

    column_pad = 2
    time_width = 12
    count_width = 7

    if labels:
        assert len(labels) == len(summaries)
        max_label_len = time_width * 2 + column_pad
        if not args.hide_counts:
            max_label_len += count_width + column_pad
        print("".ljust(max_name_len + column_pad), end="")
        for label in labels:
            short_label = label[-max_label_len:]
            print(short_label.ljust(max_label_len + column_pad), end="")
        print("")

    print(
        "event name".ljust(max_name_len + column_pad)
        + (
            "count".rjust(count_width).ljust(count_width + column_pad)
            if not args.hide_counts
            else ""
        )
        + "incl (ms)".rjust(time_width).ljust(time_width + column_pad)
        + "excl (ms)".rjust(time_width).ljust(time_width + column_pad)
    )

    for tup in all_names_with_times_list:
        name = tup[0]
        print(name.ljust(max_name_len + column_pad), end="")
        for index, summary in enumerate(summaries):
            base_summary = summaries[0] if index > 0 else None
            if name in summary:
                item = summary[name]
                if base_summary and print_relative_timings and name in base_summary:
                    base_item = base_summary[name]
                    time_inclusive = item.time_inclusive - base_item.time_inclusive
                    time_exclusive = item.time_exclusive - base_item.time_exclusive
                    show_sign = True
                else:
                    time_inclusive = item.time_inclusive
                    time_exclusive = item.time_exclusive
                    show_sign = False
                if not args.hide_counts:
                    print(
                        str(item.count)
                        .rjust(count_width)
                        .ljust(count_width + column_pad),
                        end="",
                    )
                print(
                    _display_time_ms(time_inclusive, args, show_sign=show_sign)
                    .rjust(time_width)
                    .ljust(time_width + column_pad)
                    + _display_time_ms(time_exclusive, args, show_sign=show_sign)
                    .rjust(time_width)
                    .ljust(time_width + column_pad),
                    end="",
                )
            else:
                if not args.hide_counts:
                    print(
                        "-".rjust(count_width).ljust(count_width + column_pad), end=""
                    )
                print(
                    "-".rjust(time_width).ljust(time_width + column_pad)
                    + "-".rjust(time_width).ljust(time_width + column_pad),
                    end="",
                )
        print("")


def get_sqlite_filepaths_from_directory(directory):
    """Returns a list of filepaths."""
    filepaths = []
    os.chdir(directory)
    for filepath in glob.glob("*.sqlite"):
        filepaths.append(filepath)
    return filepaths


def create_arg_parser() -> ArgumentParser:
    """For compare_profiles.py script. Includes print formatting options."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--sort-by",
        default="inclusive",
        choices=["inclusive", "exclusive"],
        help="Sort rows by inclusive or exclusive time.",
    )
    parser.add_argument(
        "--source-time-units-per-second",
        type=int,
        default=1000 * 1000 * 1000,
        metavar="N",
        help="If your source NVTX event times were recorded as nanoseconds, use 1000000000 here.",
    )
    parser.add_argument(
        "--relative",
        action="store_true",
        default=False,
        help="When comparing 2+ profiles, display times as relative to the first profile's times.",
    )
    parser.add_argument(
        "--hide-counts",
        action="store_true",
        default=False,
        help="Hide event counts.",
    )
    return parser


def main():
    args = create_arg_parser().parse_args()

    filepaths = get_sqlite_filepaths_from_directory("./")

    if len(filepaths) == 0:
        print("No sqlite files were found in the working directory.")
        return

    filepaths.sort()  # sort alphabetically

    summaries = []
    for filepath in filepaths:
        events = get_sqlite_events(sqlite3.connect(filepath))
        summaries.append(create_summary_from_events(events))

    print_summaries(summaries, args, labels=filepaths)


if __name__ == "__main__":
    main()
