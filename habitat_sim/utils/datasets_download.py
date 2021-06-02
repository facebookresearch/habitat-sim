#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import argparse
import os
import pathlib
import shutil
import sys
import traceback
import zipfile

data_sources = {}
data_groups = {}


def initialize_test_data_sources(data_path):
    global data_sources
    global data_groups
    r"""Initializes data_sources and data_groups dicts with a variable data_path path.
    """
    # dict keyed by uids with download specs for various individual test assets and datasets
    data_sources = {
        # "example_uid": {
        #   "source": the URL download link
        #   "package_name": the filename of the downloaded compressed package
        #   "download_pre_args": (optional)(wget) commands preceding filename
        #   "download_post_args": (optional)(wget) commands follow filename
        #   "link": symlink to the data directory pointing to the active version directory
        #   "version": data version tag
        # }
        "habitat_test_scenes": {
            "source": "http://dl.fbaipublicfiles.com/habitat/habitat-test-scenes_v1.0.zip",
            "package_name": "habitat-test-scenes_v1.0.zip",
            "link": data_path + "scene_datasets/habitat-test-scenes",
            "version": "1.0",
        },
        "habitat_test_pointnav_dataset": {
            "source": "http://dl.fbaipublicfiles.com/habitat/habitat-test-pointnav-dataset_v1.0.zip",
            "package_name": "habitat-test-pointnav-dataset_v1.0.zip",
            "link": data_path + "datasets/pointnav/habitat-test-scenes",
            "version": "1.0",
        },
        "habitat_example_objects": {
            "source": "http://dl.fbaipublicfiles.com/habitat/objects_v0.2.zip",
            "package_name": "objects_v0.2.zip",
            "link": data_path + "objects/example_objects",
            "version": "0.2",
        },
        "locobot_merged": {
            "source": "http://dl.fbaipublicfiles.com/habitat/locobot_merged_v0.2.zip",
            "package_name": "locobot_merged_v0.2.zip",
            "link": data_path + "objects/locobot_merged",
            "version": "0.2",
        },
        "mp3d_example_scene": {
            "source": "http://dl.fbaipublicfiles.com/habitat/mp3d_example.zip",
            "package_name": "mp3d_example.zip",
            "link": data_path + "scene_datasets/mp3d_example",
            "version": "1.0",
        },
        "coda_scene": {
            "source": "https://dl.fbaipublicfiles.com/habitat/coda_v1.0.zip",
            "package_name": "coda_v1.0.zip",
            "download_pre_args": "--no-check-certificate ",
            "download_post_args": " -O " + data_path + "coda_v1.0.zip",
            "link": data_path + "scene_datasets/coda",
            "version": "1.0",
        },
    }

    # data sources can be grouped for batch commands with a new uid
    data_groups = {
        "ci_test_assets": [
            "habitat_test_scenes",
            "habitat_test_pointnav_dataset",
            "habitat_example_objects",
            "locobot_merged",
            "mp3d_example_scene",
            "coda_scene",
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


def clean_data(uid, data_path):
    r"""Deletes the "root" directory for the named data-source."""
    if not data_sources.get(uid):
        print(f"Data clean failed, no datasource named {uid}")
        return
    link_path = os.path.join(data_path, data_sources[uid]["link"])
    version_tag = data_sources[uid]["version"]
    version_dir = os.path.join(data_path, "versioned_data/" + uid + "_" + version_tag)
    print(
        f"Cleaning datasource ({uid}). Directory: '{version_dir}'. Symlink: '{link_path}'."
    )
    try:
        shutil.rmtree(version_dir)
        os.unlink(link_path)
    except OSError:
        print("Removal error:")
        traceback.print_exc(file=sys.stdout)
        print("--------------------")


def download_and_place(uid, data_path, replace=False):
    r"""Data-source download function. Validates uid, handles existing data version, downloads data, unpacks, writes version, cleans up."""
    if not data_sources.get(uid):
        print(f"Data download failed, no datasource named {uid}")
        return

    # link_path = os.path.join(data_path, data_sources[uid]["link"])
    link_path = pathlib.Path(data_sources[uid]["link"])
    version_tag = data_sources[uid]["version"]
    version_dir = os.path.join(data_path, "versioned_data/" + uid + "_" + version_tag)

    # check for current version
    if os.path.exists(version_dir):
        print(
            f"Existing data source ({uid}) version ({version_tag}) is current. Data located: '{version_dir}'. Symblink: '{link_path}'."
        )
        replace_existing = (
            replace if replace else prompt_yes_no("Replace versioned data?")
        )

        if replace_existing:
            clean_data(uid, data_path)
        else:
            print("=======================================================")
            print(
                f"Not replacing data, generating symlink ({link_path}) and aborting download."
            )
            print("=======================================================")
            # create a symlink to the versioned data
            if link_path.exists():
                os.unlink(link_path)
            elif not link_path.parent.exists():
                link_path.parent.mkdir(parents=True, exist_ok=True)
            os.symlink(src=version_dir, dst=link_path, target_is_directory=True)
            assert link_path.exists(), "Failed, no symlink generated."
            return

    # download new version
    download_pre_args = data_sources[uid].get("download_pre_args", "")
    download_post_args = data_sources[uid].get("download_post_args", "")

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
            zip_ref.extractall(version_dir)
    else:
        # TODO: support more compression types as necessary
        print(f"Data unpack failed for {uid}. Unsupported filetype: {package_name}")
        return

    assert os.path.exists(version_dir), "Unpacking failed, no version directory."

    # create a symlink to the new versioned data
    if link_path.exists():
        os.unlink(link_path)
    elif not link_path.parent.exists():
        link_path.parent.mkdir(parents=True, exist_ok=True)
    os.symlink(src=version_dir, dst=link_path, target_is_directory=True)

    assert link_path.exists(), "Unpacking failed, no symlink generated."

    # clean-up
    os.remove(data_path + package_name)

    print("=======================================================")
    print(f"Dataset ({uid}) successfully downloaded.")
    print(f"Source: '{version_dir}'")
    print(f"Symlink: '{link_path}'")
    print("=======================================================")


class AutoHelpParser(argparse.ArgumentParser):
    def error(self, message):
        sys.stderr.write("error: %s\n" % message)
        self.print_help()
        sys.exit(2)


def main(args):
    parser = AutoHelpParser()
    arg_group1 = parser.add_mutually_exclusive_group()
    arg_group1.add_argument(
        "--uids",
        nargs="*",
        default=None,
        type=str,
        help="Unique ID of the data to download.",
    )
    arg_group1.add_argument(
        "--list",
        action="store_true",
        help="List available datasource uid options and exit.",
    )

    parser.add_argument(
        "--data-path",
        type=str,
        help='Optionally provide a path to the desired root data/ directory. Default is "habitat-sim/data/".',
    )
    arg_group2 = parser.add_mutually_exclusive_group()
    arg_group2.add_argument(
        "--clean",
        action="store_true",
        help="Remove nested child directories for the datasource.",
    )
    arg_group2.add_argument(
        "--replace",
        action="store_true",
        help="If set, existing equivalent versions of any dataset found during download will be deleted automatically. Otherwise user will be prompted before overriding existing data.",
    )

    args = parser.parse_args(args)
    replace = args.replace

    # get a default data_path from git
    data_path = args.data_path
    if not data_path:
        try:
            import git

            repo = git.Repo(".", search_parent_directories=True)
            dir_path = repo.working_tree_dir
            # Root data directory. Optionaly overridden by input argument "--data-path".
            data_path = os.path.join(dir_path, "data")
        except Exception:
            traceback.print_exc(file=sys.stdout)
            print("----------------------------------------------------------------")
            print(
                "Aborting download, failed to get default data_path from git repo and none provided."
            )
            print("Try providing --data-path (e.g. '/path/to/habitat-sim/data/')")
            print("----------------------------------------------------------------")
            parser.print_help()
            exit(2)

    # initialize data_sources and data_groups with test and example assets
    if not os.path.exists(data_path):
        os.mkdir(data_path)
    data_path = os.path.abspath(data_path) + "/"
    initialize_test_data_sources(data_path=data_path)

    # validatation: ids are unique between groups and sources
    for key in data_groups:
        assert key not in data_sources, "Duplicate key: " + key

    if args.list:
        print("====================================")
        print("Currently available datasources are:")
        print("------------------------------------")
        for source_uid in data_sources:
            print(source_uid)
        print("====================================")
        print("Currently available datagroups are:")
        print("------------------------------------")
        for group in data_groups.items():
            print(group)
        print("====================================")
        exit()

    if not args.uids:
        print("No datasource uid(s) provided.")
        parser.print_help()
        exit(2)
    for uid in args.uids:
        uids = [uid]
        if uid in data_groups:
            "Downloading a data group..."
            uids = data_groups[uid]

        for uid in uids:
            if args.clean:
                clean_data(uid, data_path)
            else:
                download_and_place(uid, data_path, replace)


if __name__ == "__main__":
    main(sys.argv[1:])
