#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import argparse
import os
import shutil
import zipfile

import git

repo = git.Repo(".", search_parent_directories=True)
dir_path = repo.working_tree_dir
# Root data directory. Optionaly overridden by input argument "--data_path".
data_path = os.path.join(dir_path, "data")

data_sources = {}
data_groups = {}


def initialize_test_data_sources(data_path):
    global data_sources
    global data_groups
    r"""Initializes data_sources and data_groups dicts with a variable data_path path.
    """
    # dict keyed by uids with download specs for various individual test assets and datasets
    data_sources = {
        # "example-uid": {
        #   "source": the URL download link
        #   "package_name": the filename of the downloaded compressed package
        #   "download_pre_args": (optional)(wget) commands preceding filename
        #   "download_post_args": (optional)(wget) commands follow filename
        #   "unpack_to": destination path for uncompression of the download package
        #   "root": root of the data directory structure once unpacked
        #   "version": data version tag
        # }
        "habitat-test-scenes": {
            "source": "http://dl.fbaipublicfiles.com/habitat/habitat-test-scenes.zip",
            "package_name": "habitat-test-scenes.zip",
            "unpack_to": data_path + "../",
            "root": data_path + "scene_datasets/habitat-test-scenes/",
            "version": "1.0",
        },
        "habitat-example-objects": {
            "source": "http://dl.fbaipublicfiles.com/habitat/objects_v0.2.zip",
            "package_name": "objects_v0.2.zip",
            "unpack_to": data_path + "objects/example_objects/",
            "root": data_path + "objects/example_objects/",
            "version": "0.2",
        },
        "locobot-merged": {
            "source": "http://dl.fbaipublicfiles.com/habitat/locobot_merged_v0.2.zip",
            "package_name": "locobot_merged_v0.2.zip",
            "unpack_to": data_path + "objects/locobot_merged/",
            "root": data_path + "objects/locobot_merged/",
            "version": "0.2",
        },
        "mp3d-test-scene": {
            "source": "http://dl.fbaipublicfiles.com/habitat/mp3d_example.zip",
            "package_name": "mp3d_example.zip",
            "unpack_to": data_path + "scene_datasets/mp3d_test/",
            "root": data_path + "scene_datasets/mp3d_test/",
            "version": "1.0",
        },
        "coda-scene": {
            "source": "'https://docs.google.com/uc?export=download&id=1Pc-J6pZzXEd8RSeLM94t3iwO8q_RQ853'",
            "package_name": "coda.zip",
            "download_pre_args": "--no-check-certificate ",
            "download_post_args": " -O " + data_path + "coda.zip",
            "unpack_to": data_path + "scene_datasets/",
            "root": data_path + "scene_datasets/coda/",
            "version": "1.0",
        },
    }

    # data sources can be grouped for batch commands with a new uid
    data_groups = {
        "ci-test-assets": [
            "habitat-test-scenes",
            "habitat-example-objects",
            "locobot-merged",
            "mp3d-test-scene",
            "coda-scene",
        ]
    }


def prompt_yes_no(message):
    r"""Prints a message and prompts the user for "y" or "n" returning True or False."""
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
    r"""Deletes the "root" directory for the named data-source."""
    if not data_sources.get(uid):
        print(f"Data clean failed, no datasource named {uid}")
        return
    root_path = data_sources[uid]["root"]
    print(f"Cleaning datasource ({uid}) directory: {root_path}")
    try:
        shutil.rmtree(root_path)
    except OSError as e:
        print(f"Error: {dir_path} : {e.strerror}")


def download_and_place(uid, replace=False):
    r"""Data-source download function. Validates uid, handles existing data version, downloads data, unpacks, writes version, cleans up."""
    if not data_sources.get(uid):
        print(f"Data download failed, no datasource named {uid}")
        return

    root_path = data_sources[uid]["root"]
    version_tag = data_sources[uid]["version"]
    # check for current version
    version_filepath = os.path.join(
        data_path,
        data_sources[uid]["root"] + "DATA_VERSION_" + version_tag,
    )
    if os.path.exists(data_sources[uid]["root"]):
        resolved_existing = False
        replace_existing = False
        for file in os.listdir(root_path):
            if "DATA_VERSION" in file:
                existing_version = file[13:]
                if existing_version == version_tag:
                    print(
                        f"Existing data source ({uid}) version ({existing_version}) is current, aborting download. Root directory: {root_path}"
                    )
                    return
                else:
                    # found root directory with different version
                    found_version_message = f"({uid}) Found previous data version ({existing_version}). Replace with requested version ({version_tag})?"
                    replace_existing = (
                        replace if replace else prompt_yes_no(found_version_message)
                    )
                    resolved_existing = True
        if not resolved_existing:
            # found root directory with unspecified version
            found_version_message = f"({uid}) Found unknown data version. Replace with requested version ({version_tag})?"
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
        if not data_sources[uid].get("download_pre_args")
        else data_sources[uid]["download_pre_args"]
    )
    download_post_args = (
        ""
        if not data_sources[uid].get("download_post_args")
        else data_sources[uid]["download_post_args"]
    )
    download_command = (
        "wget --continue "
        + download_pre_args
        + data_sources[uid]["source"]
        + " -P "
        + data_path
        + download_post_args
    )
    # print(download_command)
    os.system(download_command)
    assert os.path.exists(
        os.path.join(data_path, data_sources[uid]["package_name"])
    ), "Download failed, no package found."

    # unpack
    package_name = data_sources[uid]["package_name"]
    if package_name.endswith(".zip"):
        with zipfile.ZipFile(data_path + package_name, "r") as zip_ref:
            zip_ref.extractall(data_sources[uid]["unpack_to"])

    else:
        # TODO: support more compression types as necessary
        print(f"Data unpack failed for {uid}. Unsupported filetype: {package_name}")
        return
    assert os.path.exists(
        os.path.join(data_path, data_sources[uid]["root"])
    ), "Unpacking failed, no root directory."

    # write version
    with open(version_filepath, "w"):
        print("Wrote version file: " + version_filepath)

    # clean-up
    os.remove(data_path + package_name)

    print(f"Dataset ({uid}) successfully downloaded. Root directory: {root_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--uid",
        type=str,
        help="Unique ID of the data to download.",
    )
    parser.add_argument(
        "--data_path",
        default=data_path,
        type=str,
        help='Optionally provide a path to the desired root data/ directory. Default is "habitat-sim/data/".',
    )
    arg_group = parser.add_mutually_exclusive_group()
    arg_group.add_argument(
        "--clean",
        action="store_true",
        help="Remove nested child directories for the datasource.",
    )
    arg_group.add_argument(
        "--replace",
        action="store_true",
        help="If set, alternative versions of any dataset found during download will be deleted automatically. Otherwise user will be prompted before overriding existing data.",
    )

    args = parser.parse_args()
    replace = args.replace

    # initialize data_sources and data_groups with test and example assets
    if not os.path.exists(args.data_path):
        os.mkdir(args.data_path)
    data_path = os.path.abspath(args.data_path) + "/"
    initialize_test_data_sources(data_path=data_path)

    # validatation: ids are unique between groups and sources
    for key in data_groups:
        assert key not in data_sources, "Duplicate key: " + key

    uids = [args.uid]
    if args.uid in data_groups:
        "Downloading a data group..."
        uids = data_groups[args.uid]

    for uid in uids:
        if args.clean:
            clean_data(uid)
        else:
            download_and_place(uid, replace)
