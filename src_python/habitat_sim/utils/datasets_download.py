#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import argparse
import gzip
import itertools
import json
import os
import pathlib
import shlex
import shutil
import subprocess
import sys
import tarfile
import traceback
import zipfile
from typing import List, Optional

data_sources = {}
data_groups = {}


def hm3d_train_configs_post(extract_dir: str) -> List[str]:
    all_scene_dataset_cfg = os.path.join(
        extract_dir, "hm3d_basis.scene_dataset_config.json"
    )
    assert os.path.exists(all_scene_dataset_cfg)

    link_name = os.path.join(extract_dir, "..", "hm3d_basis.scene_dataset_config.json")
    os.symlink(all_scene_dataset_cfg, link_name)

    return [link_name]


def hm3d_semantic_configs_post(extract_dir: str) -> List[str]:
    all_scene_dataset_cfg = os.path.join(
        extract_dir, "hm3d_annotated_basis.scene_dataset_config.json"
    )
    assert os.path.exists(all_scene_dataset_cfg)

    dst_name = os.path.join(
        extract_dir, "..", "hm3d_annotated_basis.scene_dataset_config.json"
    )
    os.replace(all_scene_dataset_cfg, dst_name)

    return [dst_name]


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
            "source": "http://dl.fbaipublicfiles.com/habitat/mp3d/mp3d_example_v1.1.zip",
            "package_name": "mp3d_example_v1.1.zip",
            "link": data_path + "scene_datasets/mp3d_example",
            "version": "1.1",
        },
        "coda_scene": {
            "source": "https://dl.fbaipublicfiles.com/habitat/coda_v1.0.zip",
            "package_name": "coda_v1.0.zip",
            "download_pre_args": "--no-check-certificate ",
            "download_post_args": " -O " + data_path + "coda_v1.0.zip",
            "link": data_path + "scene_datasets/coda",
            "version": "1.0",
        },
        "webxr_hand_demo": {
            "source": "https://dl.fbaipublicfiles.com/habitat/data/scene_datasets/webxr_hand_demo_data.zip",
            "package_name": "webxr_hand_demo_data.zip",
            "link": data_path + "webxr_hand_demo_dataset",
            "version": "1.0",
        },
        "replica_cad_dataset": {
            "source": "https://dl.fbaipublicfiles.com/habitat/ReplicaCAD/ReplicaCAD_dataset_v1.5.zip",
            "package_name": "ReplicaCAD_dataset_v1.5.zip",
            "link": data_path + "replica_cad",
            "version": "1.5",
        },
        "replica_cad_baked_lighting": {
            "source": "https://dl.fbaipublicfiles.com/habitat/ReplicaCAD/ReplicaCAD_baked_lighting_v1.5.zip",
            "package_name": "ReplicaCAD_baked_lighting_v1.5.zip",
            "link": data_path + "replica_cad_baked_lighting",
            "version": "1.5",
        },
        "ycb": {
            "source": "https://dl.fbaipublicfiles.com/habitat/ycb/hab_ycb_v1.2.zip",
            "package_name": "hab_ycb_v1.2.zip",
            "link": data_path + "objects/ycb",
            "version": "1.2",
        },
        "hab_fetch": {
            "source": "http://dl.fbaipublicfiles.com/habitat/hab_fetch_v1.0.zip",
            "package_name": "hab_fetch_v1.0.zip",
            "link": data_path + "robots/hab_fetch",
            "version": "1.0",
        },
        "rearrange_pick_dataset_v0": {
            "source": "https://dl.fbaipublicfiles.com/habitat/data/datasets/rearrange_pick/replica_cad/v0/rearrange_pick_replica_cad_v0.zip",
            "package_name": "rearrange_pick_replica_cad_v0.zip",
            "link": data_path + "datasets/rearrange_pick/replica_cad/v0",
            "version": "1.0",
        },
        "rearrange_dataset_v1": {
            "source": "https://dl.fbaipublicfiles.com/habitat/data/datasets/replica_cad/v1.zip",
            "package_name": "v1.zip",
            "link": data_path + "datasets/replica_cad/rearrange",
            "version": "1.0",
        },
        "hab2_bench_assets": {
            "source": "https://dl.fbaipublicfiles.com/habitat/ReplicaCAD/hab2_bench_assets.zip",
            "package_name": "hab2_bench_assets.zip",
            "link": data_path + "hab2_bench_assets",
            "version": "1.0",
        },
    }

    data_sources.update(
        {
            f"hm3d_{split}_{data_format}": {
                "source": "https://api.matterport.com/resources/habitat/hm3d-{split}-{data_format}.tar{ext}".format(
                    ext=".gz" if data_format == "obj+mtl" else "",
                    split=split,
                    data_format=data_format,
                ),
                "download_pre_args": "--location",
                "package_name": "hm3d-{split}-{data_format}.tar{ext}".format(
                    ext=".gz" if data_format == "obj+mtl" else "",
                    split=split,
                    data_format=data_format,
                ),
                "link": data_path + "scene_datasets/hm3d",
                "version": "1.0",
                "version_dir": "hm3d-{version}/hm3d",
                "extract_postfix": f"{split}",
                "downloaded_file_list": f"hm3d-{{version}}/{split}-{data_format}-files.json.gz",
                "requires_auth": True,
                "use_curl": True,
                "post_extract_fn": hm3d_train_configs_post
                if split == "train" and data_format == "configs"
                else None,
            }
            for split, data_format in itertools.product(
                ["minival", "train", "val"],
                ["glb", "obj+mtl", "habitat", "configs"],
            )
        }
    )

    data_sources.update(
        {
            f"hm3d_example_{data_format}": {
                "source": "https://github.com/matterport/habitat-matterport-3dresearch/raw/main/example/hm3d-example-{data_format}.tar{ext}".format(
                    ext=".gz" if data_format == "obj+mtl" else "",
                    data_format=data_format,
                ),
                "package_name": "hm3d-example-{data_format}.tar{ext}".format(
                    ext=".gz" if data_format == "obj+mtl" else "",
                    data_format=data_format,
                ),
                "link": data_path + "scene_datasets/hm3d",
                "version": "1.0",
                "version_dir": "hm3d-{version}/hm3d",
                "extract_postfix": "example",
                "downloaded_file_list": f"hm3d-{{version}}/example-{data_format}-files.json.gz",
            }
            for data_format in ["glb", "obj+mtl", "habitat", "configs"]
        }
    )

    data_sources.update(
        {
            f"hm3d_{split}_semantic_{data_format}_v0.1": {
                "source": "https://api.matterport.com/resources/habitat/hm3d-{split}-semantic-{data_format}-v0.1.tar{ext}".format(
                    ext=".gz" if data_format == "annots" else "",
                    split=split,
                    data_format=data_format,
                ),
                "download_pre_args": "--location",
                "package_name": "hm3d-{split}-semantic-{data_format}-v0.1.tar{ext}".format(
                    ext=".gz" if data_format == "annots" else "",
                    split=split,
                    data_format=data_format,
                ),
                "link": data_path + "scene_datasets/hm3d",
                "version": "1.0",
                "version_dir": "hm3d-{version}/hm3d",
                "extract_postfix": f"{split}",
                "downloaded_file_list": f"hm3d-{{version}}/{split}-semantic-{data_format}-files.json.gz",
                "requires_auth": True,
                "use_curl": True,
                "post_extract_fn": hm3d_semantic_configs_post
                if data_format == "configs"
                else None,
            }
            for split, data_format in itertools.product(
                ["minival", "train", "val"],
                ["annots", "configs"],
            )
        }
    )

    data_sources.update(
        {
            f"hm3d_example_semantic_{data_format}_v0.1": {
                "source": "https://github.com/matterport/habitat-matterport-3dresearch/raw/main/example/hm3d-example-semantic-{data_format}-v0.1.tar{ext}".format(
                    ext=".gz" if data_format == "annots" else "",
                    data_format=data_format,
                ),
                "package_name": "hm3d-example-semantic-{data_format}-v0.1.tar{ext}".format(
                    ext=".gz" if data_format == "annots" else "",
                    data_format=data_format,
                ),
                "link": data_path + "scene_datasets/hm3d",
                "version": "1.0",
                "version_dir": "hm3d-{version}/hm3d",
                "extract_postfix": "example",
                "downloaded_file_list": f"hm3d-{{version}}/example-semantic-{data_format}-files.json.gz",
            }
            for data_format in ["annots", "configs"]
        }
    )

    # data sources can be grouped for batch commands with a new uid
    data_groups = {
        "ci_test_assets": [
            "habitat_test_scenes",
            "habitat_test_pointnav_dataset",
            "habitat_example_objects",
            "locobot_merged",
            "mp3d_example_scene",
            "coda_scene",
            "replica_cad_dataset",
            "hab_fetch",
        ],
        "rearrange_task_assets": [
            "replica_cad_dataset",
            "hab_fetch",
            "ycb",
            "rearrange_pick_dataset_v0",
            "rearrange_dataset_v1",
        ],
        "hm3d_example": [
            "hm3d_example_habitat",
            "hm3d_example_configs",
            "hm3d_example_semantic_annots_v0.1",
            "hm3d_example_semantic_configs_v0.1",
        ],
        "hm3d_val": [
            "hm3d_val_habitat",
            "hm3d_val_configs",
            "hm3d_val_semantic_annots_v0.1",
            "hm3d_val_semantic_configs_v0.1",
        ],
        "hm3d_train": [
            "hm3d_train_habitat",
            "hm3d_train_configs",
            "hm3d_train_semantic_annots_v0.1",
            "hm3d_train_semantic_configs_v0.1",
        ],
        "hm3d_minival": [
            "hm3d_minival_habitat",
            "hm3d_minival_configs",
            "hm3d_minival_semantic_annots_v0.1",
            "hm3d_minival_semantic_configs_v0.1",
        ],
        "hm3d_semantics": [
            "hm3d_example_semantic_annots_v0.1",
            "hm3d_example_semantic_configs_v0.1",
            "hm3d_val_semantic_annots_v0.1",
            "hm3d_val_semantic_configs_v0.1",
            "hm3d_train_semantic_annots_v0.1",
            "hm3d_train_semantic_configs_v0.1",
            "hm3d_minival_semantic_annots_v0.1",
            "hm3d_minival_semantic_configs_v0.1",
        ],
        "hm3d_full": list(filter(lambda k: k.startswith("hm3d_"), data_sources.keys())),
    }

    data_groups.update(
        {
            f"hm3d_{split}_full": list(
                filter(lambda k: k.startswith(f"hm3d_{split}"), data_sources.keys())
            )
            for split in ["train", "val", "minival", "example"]
        }
    )

    data_groups["hm3d"] = (
        data_groups["hm3d_val"]
        + data_groups["hm3d_train"]
        + data_groups["hm3d_minival"]
    )

    data_groups["ci_test_assets"].extend(data_groups["hm3d_example"])


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


def get_version_dir(uid, data_path):
    version_tag = data_sources[uid]["version"]
    if "version_dir" in data_sources[uid]:
        version_dir = os.path.join(
            data_path,
            "versioned_data",
            data_sources[uid]["version_dir"].format(version=version_tag),
        )
    else:
        version_dir = os.path.join(
            data_path, "versioned_data/" + uid + "_" + version_tag
        )
    return version_dir


def get_downloaded_file_list(uid, data_path):
    version_tag = data_sources[uid]["version"]
    downloaded_file_list = None
    if "downloaded_file_list" in data_sources[uid]:
        downloaded_file_list = os.path.join(
            data_path,
            "versioned_data",
            data_sources[uid]["downloaded_file_list"].format(version=version_tag),
        )
    return downloaded_file_list


def clean_data(uid, data_path):
    r"""Deletes the "root" directory for the named data-source."""
    if not data_sources.get(uid):
        print(f"Data clean failed, no datasource named {uid}")
        return
    link_path = os.path.join(data_path, data_sources[uid]["link"])
    version_dir = get_version_dir(uid, data_path)
    downloaded_file_list = get_downloaded_file_list(uid, data_path)
    print(
        f"Cleaning datasource ({uid}). Directory: '{version_dir}'. Symlink: '{link_path}'."
    )
    if downloaded_file_list is None:
        try:
            shutil.rmtree(version_dir)
            os.unlink(link_path)
        except OSError:
            print("Removal error:")
            traceback.print_exc(file=sys.stdout)
            print("--------------------")
    elif os.path.exists(downloaded_file_list):
        with gzip.open(downloaded_file_list, "rt") as f:
            package_files = json.load(f)

        for package_file in reversed(package_files):
            if os.path.isdir(package_file) and len(os.listdir(package_file)) == 0:
                os.rmdir(package_file)
            elif not os.path.isdir(package_file) and os.path.exists(package_file):
                os.remove(package_file)
            elif os.path.islink(package_file):
                os.unlink(package_file)

        os.remove(downloaded_file_list)

        if os.path.exists(version_dir) and len(os.listdir(version_dir)) == 0:
            os.rmdir(version_dir)

        if not os.path.exists(version_dir):
            os.unlink(link_path)

        meta_dir = os.path.dirname(downloaded_file_list)
        if os.path.exists(meta_dir) and len(os.listdir(meta_dir)) == 0:
            os.rmdir(meta_dir)


def download_and_place(
    uid,
    data_path,
    username: Optional[str] = None,
    password: Optional[str] = None,
    replace: Optional[bool] = None,
):
    r"""Data-source download function. Validates uid, handles existing data version, downloads data, unpacks, writes version, cleans up."""
    if not data_sources.get(uid):
        print(f"Data download failed, no datasource named {uid}")
        return

    # link_path = os.path.join(data_path, data_sources[uid]["link"])
    link_path = pathlib.Path(data_sources[uid]["link"])
    version_tag = data_sources[uid]["version"]
    version_dir = get_version_dir(uid, data_path)
    downloaded_file_list = get_downloaded_file_list(uid, data_path)

    # check for current version
    if os.path.exists(version_dir) and (
        downloaded_file_list is None or os.path.exists(downloaded_file_list)
    ):
        print(
            f"Existing data source ({uid}) version ({version_tag}) is current. Data located: '{version_dir}'. Symblink: '{link_path}'."
        )
        replace_existing = (
            replace if replace is not None else prompt_yes_no("Replace versioned data?")
        )

        if replace_existing:
            clean_data(uid, data_path)
        else:
            print("=======================================================")
            print(
                f"Not replacing data, generating symlink ({link_path}) and aborting download."
            )
            print("=======================================================")

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
    requires_auth = data_sources[uid].get("requires_auth", False)
    if requires_auth:
        assert username is not None, "Usename required, please enter with --username"
        assert (
            password is not None
        ), "Password is required, please enter with --password"

    use_curl = data_sources[uid].get("use_curl", False)
    if use_curl:
        if requires_auth:
            download_pre_args = f"{download_pre_args} --user {username}:{password}"

        download_command = (
            "curl --continue-at - "
            + download_pre_args
            + " "
            + data_sources[uid]["source"]
            + " -o "
            + os.path.join(data_path, data_sources[uid]["package_name"])
            + download_post_args
        )
    else:
        if requires_auth:
            download_pre_args = (
                f"{download_pre_args} --user {username} --password {password}"
            )

        download_command = (
            "wget --continue "
            + download_pre_args
            + data_sources[uid]["source"]
            + " -P "
            + data_path
            + download_post_args
        )
    #  print(download_command)
    subprocess.check_call(shlex.split(download_command))
    assert os.path.exists(
        os.path.join(data_path, data_sources[uid]["package_name"])
    ), "Download failed, no package found."

    # unpack
    package_name = data_sources[uid]["package_name"]
    extract_postfix = data_sources[uid].get("extract_postfix", "")
    extract_dir = os.path.join(version_dir, extract_postfix)
    if package_name.endswith(".zip"):
        with zipfile.ZipFile(data_path + package_name, "r") as zip_ref:
            zip_ref.extractall(extract_dir)
            package_files = zip_ref.namelist()
    elif package_name.count(".tar") == 1:
        with tarfile.open(data_path + package_name, "r:*") as tar_ref:
            tar_ref.extractall(extract_dir)
            package_files = tar_ref.getnames()
    else:
        # TODO: support more compression types as necessary
        print(f"Data unpack failed for {uid}. Unsupported filetype: {package_name}")
        return

    assert os.path.exists(version_dir), "Unpacking failed, no version directory."
    package_files = [os.path.join(extract_dir, fname) for fname in package_files]

    post_extract_fn = data_sources[uid].get("post_extract_fn", None)
    if post_extract_fn is not None:
        result = post_extract_fn(extract_dir)
        if result is not None:
            package_files = result + package_files

    if downloaded_file_list is not None:
        with gzip.open(downloaded_file_list, "wt") as f:
            json.dump([extract_dir] + package_files, f)

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
        dest="replace",
        default=None,
        action="store_true",
        help="If set, existing equivalent versions of any dataset found during download will be deleted automatically. Otherwise user will be prompted before overriding existing data.",
    )
    arg_group2.add_argument(
        "--no-replace",
        dest="replace",
        action="store_false",
        help="If set, existing equivalent versions of any dataset found during download will be skipped automatically. Otherwise user will be prompted before overriding existing data.",
    )

    parser.add_argument(
        "--username",
        type=str,
        default=None,
        help="Username to use for downloads that require authentication",
    )
    parser.add_argument(
        "--password",
        type=str,
        default=None,
        help="Password to use for downloads that require authentication",
    )

    args = parser.parse_args(args)
    replace = args.replace

    # get a default data_path "./data/"
    data_path = args.data_path
    if not data_path:
        try:
            data_path = os.path.abspath("./data/")
            print(
                f"No data-path provided, default to: {data_path}. Use '--data-path' to specify another location."
            )
            if not os.path.exists(data_path):
                os.makedirs(data_path)
        except Exception:
            traceback.print_exc(file=sys.stdout)
            print("----------------------------------------------------------------")
            print(
                "Aborting download, failed to create default data_path and none provided."
            )
            print("Try providing --data-path (e.g. '/path/to/habitat-sim/data/')")
            print("----------------------------------------------------------------")
            parser.print_help()
            exit(2)

    # initialize data_sources and data_groups with test and example assets
    os.makedirs(data_path, exist_ok=True)
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
                download_and_place(
                    uid, data_path, args.username, args.password, replace
                )


if __name__ == "__main__":
    main(sys.argv[1:])
