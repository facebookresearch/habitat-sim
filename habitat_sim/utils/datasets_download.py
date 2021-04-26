#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import argparse
import os

# TODO: is there a better way to do this more generaly? Maybe pybind a cmake compiled DATA_DIR?
dir_path = os.path.dirname(os.path.realpath(__file__))
data_dir = os.path.abspath(os.path.join(dir_path, "../../data/")) + "/"

# dict keyed by uids with download specs for various test assets and datasets
data_sources = {
    # "example-uid": {
    #
    # }
    "habitat-test-scenes": {
        "source": "http://dl.fbaipublicfiles.com/habitat/habitat-test-scenes.zip",
        "package_name": "habitat-test-scenes.zip",
        "unpack_to": data_dir + "../",
        "root": data_dir + "scene_datasets/habitat-test-scenes/",
        "version": "1.0",
    },
    "habitat-example-objects": {
        "source": "http://dl.fbaipublicfiles.com/habitat/objects_v0.2.zip",
        "package_name": "objects_v0.2.zip",
        "unpack_to": data_dir + "objects/example_objects/",
        "root": data_dir + "objects/example_objects/",
        "version": "0.2",
    },
    "locobot-merged": {
        "source": "http://dl.fbaipublicfiles.com/habitat/locobot_merged_v0.2.zip",
        "package_name": "locobot_merged_v0.2.zip",
        "unpack_to": data_dir + "objects/locobot_merged/",
        "root": data_dir + "objects/locobot_merged/",
        "version": "0.2",
    },
    "mp3d-test-scene": {
        "source": "http://dl.fbaipublicfiles.com/habitat/mp3d_example.zip",
        "package_name": "mp3d_example.zip",
        "unpack_to": data_dir + "scene_datasets/mp3d_test/",
        "root": data_dir + "scene_datasets/mp3d_test/",
        "version": "1.0",
    },
    "coda-scene": {
        "source": "'https://docs.google.com/uc?export=download&id=1Pc-J6pZzXEd8RSeLM94t3iwO8q_RQ853'",
        "package_name": "coda.zip",
        "download_pre_args": "--no-check-certificate ",
        "download_post_args": " -O " + data_dir + "coda.zip",
        "unpack_to": data_dir + "scene_datasets/",
        "root": data_dir + "scene_datasets/coda/",
        "version": "1.0",
    },
}

data_groups = {
    "ci-test-assets": [
        "habitat-test-scenes",
        "habitat-example-objects",
        "locobot-merged",
        "mp3d-test-scene",
        "coda-scene",
    ]
}

# validatation: ids are unique between groups and sources
for key in data_groups:
    assert key not in data_sources, "Duplicate key: " + key


def prompt_yes_no(message):
    print("\n-------------------------")
    print(message)
    while True:
        answer = input("(y|n): ")
        if answer.lower() == "y":
            return True
        elif answer.lower() == "n":
            return False
        else:
            print("Invalid answer...")


def clean_data(uid):
    if uid not in data_sources:
        print("Data download failed, no datasource named " + uid)
        return
    print(
        "Cleaning datasource " + uid + ' directory: "' + data_sources[uid]["root"] + '"'
    )
    os.system("rm -r " + data_sources[uid]["root"])


def download_and_place(uid, replace=False):
    if uid not in data_sources:
        print("Data download failed, no datasource named " + uid)
        return

    # check for current version
    version_filepath = os.path.join(
        data_dir,
        data_sources[uid]["root"] + "DATA_VERSION_" + data_sources[uid]["version"],
    )
    if os.path.exists(data_sources[uid]["root"]):
        resolved_existing = False
        replace_existing = False
        for file in os.listdir(data_sources[uid]["root"]):
            if "DATA_VERSION" in file:
                existing_version = file[13:]
                if existing_version == data_sources[uid]["version"]:
                    print(
                        "Existing data source ("
                        + uid
                        + ") version ("
                        + existing_version
                        + ") is current, aborting download."
                        + " Root directory: "
                        + data_sources[uid]["root"]
                    )
                    return
                else:
                    # found root directory with different version
                    found_version_message = (
                        "("
                        + uid
                        + ") Found previous data version ("
                        + existing_version
                        + "). Replace with requested version ("
                        + data_sources[uid]["version"]
                        + ")?"
                    )
                    replace_existing = (
                        replace if replace else prompt_yes_no(found_version_message)
                    )
                    resolved_existing = True
        if not resolved_existing:
            # found root directory with unspecified version
            found_version_message = (
                "("
                + uid
                + ") Found unknown data version. Replace with requested version ("
                + data_sources[uid]["version"]
                + ")?"
            )
            replace_existing = (
                replace if replace else prompt_yes_no(found_version_message)
            )
        if replace_existing:
            clean_data(uid)
        else:
            print("Not replacing data, aborting download.")
            return

    # download
    download_pre_args = (
        ""
        if "download_pre_args" not in data_sources[uid]
        else data_sources[uid]["download_pre_args"]
    )
    download_post_args = (
        ""
        if "download_post_args" not in data_sources[uid]
        else data_sources[uid]["download_post_args"]
    )
    download_command = (
        "wget --continue "
        + download_pre_args
        + data_sources[uid]["source"]
        + " -P "
        + data_dir
        + download_post_args
    )
    # print(download_command)
    os.system(download_command)
    assert os.path.exists(
        os.path.join(data_dir, data_sources[uid]["package_name"])
    ), "Download failed, no package found."

    # unpack
    package_name = data_sources[uid]["package_name"]
    if package_name.endswith(".zip"):
        os.system(
            "unzip -n "
            + data_dir
            + package_name
            + " -d "
            + data_sources[uid]["unpack_to"]
        )
    else:
        # TODO: support more compression types as necessary
        print(
            "Data unpack failed for " + uid + " unsupported filetype: " + package_name
        )
        return
    assert os.path.exists(
        os.path.join(data_dir, data_sources[uid]["root"])
    ), "Unpacking failed, no root directory."

    # write version
    with open(version_filepath, "w"):
        print("Wrote version file: " + version_filepath)

    # clean-up
    os.system("rm " + data_dir + package_name)

    print(
        "Dataset ("
        + uid
        + ") successfully downloaded. Root directory: "
        + data_sources[uid]["root"]
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--uid",
        type=str,
        help="Unique ID of the data to download.",
    )
    parser.add_argument(
        "--clean",
        action="store_true",
        help="Remove nested child directories for the datasource.",
    )
    parser.add_argument(
        "--replace",
        action="store_true",
        help="If set, alternative versions of any dataset found during download will be deleted automatically. Otherwise user will be prompted before overriding existing data.",
    )

    args = parser.parse_args()
    replace = args.replace

    uids = [args.uid]
    if args.uid in data_groups:
        "Downloading a data group..."
        uids = data_groups[args.uid]

    for uid in uids:
        if args.clean:
            clean_data(uid)
        else:
            download_and_place(uid, replace)
