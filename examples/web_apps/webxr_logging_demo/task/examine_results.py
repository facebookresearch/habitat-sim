#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from mephisto.abstractions.databases.local_database import LocalMephistoDB
from mephisto.data_model.unit import Unit
from mephisto.data_model.worker import Worker
from mephisto.tools.examine_utils import print_results, run_examine_by_worker

db = None


def format_for_printing_data(data):
    global db
    # Custom tasks can define methods for how to display their data in a relevant way
    worker_name = Worker.get(db, data["worker_id"]).worker_name
    contents = data["data"]
    duration = contents["times"]["task_end"] - contents["times"]["task_start"]
    metadata_string = (
        f"Worker: {worker_name}\nUnit: {data['unit_id']}\n"
        f"Duration: {int(duration)}\nStatus: {data['status']}\n"
    )

    inputs = contents["inputs"]
    inputs_string = f"Character: {inputs['character_name']}\nDescription: {inputs['character_description']}\n"

    outputs = contents["outputs"]
    output_string = f"   Head poses: {outputs['head_poses_input']}\n"
    found_files = outputs.get("files")
    if found_files is not None:
        file_dir = Unit.get(db, data["unit_id"]).get_assigned_agent().get_data_dir()
        output_string += f"   Files: {found_files}\n"
        output_string += f"   File directory {file_dir}\n"
    else:
        output_string += f"   Files: No files attached\n"
    return f"-------------------\n{metadata_string}{inputs_string}{output_string}"


def main():
    global db
    db = LocalMephistoDB()

    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-r", "--review", help="review tasks", action="store_true", default=False
    )
    args = parser.parse_args()

    if args.review:
        run_examine_by_worker(db, format_for_printing_data, "hab-vr-task")
    else:
        print_results(db, "hab-vr-task", format_for_printing_data)


if __name__ == "__main__":
    main()
