#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
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
from shutil import which
from typing import List, Optional

from git import Repo

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
        "hssd-hab": {
            "source": "https://huggingface.co/datasets/hssd/hssd-hab.git",
            "link": data_path + "scene_datasets/hssd-hab",
            "version": "v0.2.4",
            "requires_auth": True,
        },
        "hssd-raw": {
            "source": "https://huggingface.co/datasets/hssd/hssd-scenes.git",
            "link": data_path + "scene_datasets/hssd-scenes",
            "version": "main",
            "requires_auth": True,
        },
        "hssd-hab_internal": {
            "source": "https://huggingface.co/datasets/fpss/fphab.git",
            "link": data_path + "scene_datasets/fphab",
            "version": "main",
            "requires_auth": True,
        },
        "hssd-hab_objectnav_dataset": {
            "source": "https://www.dropbox.com/s/26ribfiup5249b8/objectnav_hssd-hab_v0.2.3.zip",
            "package_name": "objectnav_hssd-hab_v0.2.3.zip",
            "link": data_path + "datasets/objectnav/hssd-hab",
            "version": "v0.2.3",
        },
        "ai2thor-hab": {
            "source": "https://huggingface.co/datasets/hssd/ai2thor-hab.git",
            "link": data_path + "scene_datasets/ai2thor-hab",
            "version": "main",
            "requires_auth": True,
        },
        "procthor-hab_objectnav_dataset": {
            "source": "https://www.dropbox.com/s/mdfpevn1srr37cr/objectnav_procthor-hab.zip",
            "package_name": "objectnav_procthor-hab.zip",
            "link": data_path + "datasets/objectnav/procthor-hab",
            "version": "v1",
        },
        "habitat_test_scenes": {
            "source": "https://huggingface.co/datasets/ai-habitat/habitat_test_scenes.git",
            "link": data_path + "scene_datasets/habitat-test-scenes",
            "version": "v1.0",
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
            "source": "https://huggingface.co/datasets/ai-habitat/ReplicaCAD_dataset.git",
            "link": data_path + "replica_cad",
            "version": "v1.6",
        },
        "replica_cad_baked_lighting": {
            "source": "https://huggingface.co/datasets/ai-habitat/ReplicaCAD_baked_lighting.git",
            "link": data_path + "replica_cad_baked_lighting",
            "version": "v1.6",
        },
        "ycb": {
            "source": "https://huggingface.co/datasets/ai-habitat/ycb.git",
            "link": data_path + "objects/ycb",
            "version": "v1.2",
        },
        "franka_panda": {
            "source": "https://dl.fbaipublicfiles.com/polymetis/franka_panda.zip",
            "package_name": "franka_panda.zip",
            "link": data_path + "robots/franka_panda",
            "version": "1.0",
        },
        "hab_spot_arm": {
            "source": "https://huggingface.co/datasets/ai-habitat/hab_spot_arm.git",
            "link": data_path + "robots/hab_spot_arm",
            "version": "v2.0",
        },
        "hab_stretch": {
            "source": "https://huggingface.co/datasets/ai-habitat/hab_stretch.git",
            "link": data_path + "robots/hab_stretch",
            "version": "v1.0",
        },
        "hab_fetch": {
            "source": "https://huggingface.co/datasets/ai-habitat/hab_fetch.git",
            "link": data_path + "robots/hab_fetch",
            "version": "v2.0",
        },
        "humanoid_data": {
            "source": "http://dl.fbaipublicfiles.com/habitat/humanoids/humanoid_data_v0.1.zip",
            "package_name": "humanoid_data_v0.1.zip",
            "link": data_path + "humanoids/humanoid_data",
            "version": "0.1",
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

    # add individual hm3d splits, format components, and versions
    data_sources.update(
        {
            f"hm3d_{split}_{data_format}_v{version}": {
                "source": "https://api.matterport.com/resources/habitat/hm3d-{split}-{data_format}{version_string}.tar{ext}".format(
                    ext="",
                    split=split,
                    data_format=data_format,
                    version_string="-v0.2"
                    if (version == "0.2" and data_format != "configs")
                    else "",
                ),
                "download_pre_args": "--location",
                "package_name": "hm3d-{split}-{data_format}.tar{ext}".format(
                    ext="",
                    split=split,
                    data_format=data_format,
                ),
                "link": data_path + "scene_datasets/hm3d",
                "version": version,
                "version_dir": "hm3d-{version}/hm3d",
                "extract_postfix": f"{split}",
                "downloaded_file_list": f"hm3d-{{version}}/{split}-{data_format}-files.json.gz",
                "requires_auth": True,
                "use_curl": True,
                "post_extract_fn": hm3d_train_configs_post
                if split == "train" and data_format == "configs"
                else None,
            }
            for split, data_format, version in itertools.product(
                ["minival", "train", "val"],
                ["glb", "habitat", "configs"],
                ["0.1", "0.2"],
            )
        }
    )

    # all all hm3d examples (v0.2 only)
    data_sources.update(
        {
            f"hm3d_example_{data_format}": {
                "source": "https://github.com/matterport/habitat-matterport-3dresearch/raw/main/example/hm3d-example-{data_format}{version_string}.tar{ext}".format(
                    ext="",
                    data_format=data_format,
                    version_string="-v0.2" if data_format != "configs" else "",
                ),
                "package_name": "hm3d-example-{data_format}{version_string}.tar{ext}".format(
                    ext="",
                    data_format=data_format,
                    version_string="-v0.2" if data_format != "configs" else "",
                ),
                "link": data_path + "scene_datasets/hm3d",
                "version": "0.2",
                "version_dir": "hm3d-{version}/hm3d",
                "extract_postfix": "example",
                "downloaded_file_list": f"hm3d-{{version}}/example-{data_format}-files.json.gz",
            }
            for data_format in ["glb", "habitat", "configs"]
        }
    )

    # add all hm3d semantic splits, format components, and versions
    data_sources.update(
        {
            f"hm3d_{split}_semantic_{data_format}_v{version}": {
                "source": "https://api.matterport.com/resources/habitat/hm3d-{split}-semantic-{data_format}-v{version_string}.tar{ext}".format(
                    ext=".gz" if (version == "0.1" and data_format == "annots") else "",
                    split=split,
                    data_format=data_format,
                    version_string=version,
                ),
                "download_pre_args": "--location",
                "package_name": "hm3d-{split}-semantic-{data_format}-v{version_string}.tar{ext}".format(
                    ext=".gz" if (version == "0.1" and data_format == "annots") else "",
                    split=split,
                    data_format=data_format,
                    version_string=version,
                ),
                "link": data_path + "scene_datasets/hm3d",
                "version": version,
                "version_dir": "hm3d-{version}/hm3d",
                "extract_postfix": f"{split}",
                "downloaded_file_list": f"hm3d-{{version}}/{split}-semantic-{data_format}-files.json.gz",
                "requires_auth": True,
                "use_curl": True,
                "post_extract_fn": hm3d_semantic_configs_post
                if data_format == "configs"
                else None,
            }
            for split, data_format, version in itertools.product(
                ["minival", "train", "val"], ["annots", "configs"], ["0.1", "0.2"]
            )
        }
    )

    # all all hm3d semantic examples (v0.2 only)
    data_sources.update(
        {
            f"hm3d_example_semantic_{data_format}": {
                "source": "https://github.com/matterport/habitat-matterport-3dresearch/raw/main/example/hm3d-example-semantic-{data_format}-v0.2.tar{ext}".format(
                    ext="",
                    data_format=data_format,
                ),
                "package_name": "hm3d-example-semantic-{data_format}-v0.2.tar{ext}".format(
                    ext="",
                    data_format=data_format,
                ),
                "link": data_path + "scene_datasets/hm3d",
                "version": "0.2",
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
            "hab_stretch",
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
            "hm3d_example_semantic_annots",
            "hm3d_example_semantic_configs",
        ],
    }

    # add all hm3d variations splits + versions
    for version in ["v0.1", "v0.2"]:
        data_groups.update(
            {
                f"hm3d_val_{version}": [
                    f"hm3d_val_habitat_{version}",
                    f"hm3d_val_configs_{version}",
                    f"hm3d_val_semantic_annots_{version}",
                    f"hm3d_val_semantic_configs_{version}",
                ],
            }
        )
        data_groups.update(
            {
                f"hm3d_train_{version}": [
                    f"hm3d_train_habitat_{version}",
                    f"hm3d_train_configs_{version}",
                    f"hm3d_train_semantic_annots_{version}",
                    f"hm3d_train_semantic_configs_{version}",
                ],
            }
        )
        data_groups.update(
            {
                f"hm3d_minival_{version}": [
                    f"hm3d_minival_habitat_{version}",
                    f"hm3d_minival_configs_{version}",
                    f"hm3d_minival_semantic_annots_{version}",
                    f"hm3d_minival_semantic_configs_{version}",
                ]
            }
        )
        data_groups.update(
            {
                f"hm3d_semantics_{version}": [
                    f"hm3d_example_semantic_annots_{version}",
                    f"hm3d_example_semantic_configs_{version}",
                    f"hm3d_val_semantic_annots_{version}",
                    f"hm3d_val_semantic_configs_{version}",
                    f"hm3d_train_semantic_annots_{version}",
                    f"hm3d_train_semantic_configs_{version}",
                    f"hm3d_minival_semantic_annots_{version}",
                    f"hm3d_minival_semantic_configs_{version}",
                ]
            }
        )

    # to reproduce old experiments with hm3d v1.0 and hm3d semantics v0.1
    data_groups["hm3d_v0.1"] = (
        data_groups["hm3d_val_v0.1"]
        + data_groups["hm3d_train_v0.1"]
        + data_groups["hm3d_minival_v0.1"]
    )

    # this download is all of hm3d v0.2 + all examples in both original glb and BASIS compressed formats
    data_groups.update(
        {
            "hm3d_full": list(
                filter(
                    lambda k: (
                        k.startswith("hm3d_") and ("v0.2" in k or "example" in k)
                    ),
                    data_sources.keys(),
                )
            )
        }
    )

    # add full (glb + BASIS) downloads for v0.2 grouped by split
    data_groups.update(
        {
            f"hm3d_{split}_full": list(
                filter(
                    lambda k: (
                        k.startswith(f"hm3d_{split}")
                        and ("v0.2" in k or "example" in k)
                    ),
                    data_sources.keys(),
                )
            )
            for split in ["train", "val", "minival", "example"]
        }
    )

    # this is the primary hm3d download with v0.2 splits for use with Habitat (BASIS compressed only)
    data_groups["hm3d"] = (
        data_groups["hm3d_val_v0.2"]
        + data_groups["hm3d_train_v0.2"]
        + data_groups["hm3d_minival_v0.2"]
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
        if answer.lower() == "n":
            return False
        print("Invalid answer...")


def get_version_dir(uid, data_path, is_repo=False):
    """
    Constructs to the versioned_data directory path for the data source.
    """
    version_tag = data_sources[uid]["version"]
    if is_repo:
        # this is a git repo, so don't include version in the directory name
        version_dir = os.path.join(data_path, "versioned_data/" + uid)
    elif "version_dir" in data_sources[uid]:
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


def get_downloaded_file_list(uid: str, data_path: str) -> Optional[str]:
    """
    Get the downloaded file list path configured for the data source.
    """
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
    is_repo = data_sources[uid]["source"].endswith(".git")
    version_dir = get_version_dir(uid, data_path, is_repo)
    if not os.path.exists(version_dir) and not os.path.islink(link_path):
        print(f"Found nothing to clean for datasource ({uid}).")
        return
    downloaded_file_list = get_downloaded_file_list(uid, data_path)
    print(
        f"Cleaning datasource ({uid}). Directory: '{version_dir}'. Symlink: '{link_path}'."
    )
    if downloaded_file_list is None:
        try:
            if os.path.exists(version_dir):
                shutil.rmtree(version_dir)
            if os.path.islink(link_path):
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


def clone_repo_source(
    uid: str,
    version_dir: str,
    requires_auth: bool,
    username: Optional[str] = None,
    password: Optional[str] = None,
    prune_lfs: bool = True,
):
    """
    Clones and processes a datasource hosted on a git repo (e.g. HuggingFace Dataset).
    Handles authentication for gated sources.
    Automatically prunes the resulting repo to reduce memory overhead.
    """
    version_tag = data_sources[uid]["version"]
    clone_command = f" git clone --depth 1 --branch {version_tag} "
    if requires_auth:
        adjusted_password = password.replace(" ", "%20")
        url_split = data_sources[uid]["source"].split("https://")[-1]
        # NOTE: The username and password are stored in .git/config. Should we post-process this out?
        clone_command += f'"https://{username}:{adjusted_password}@{url_split}"'
    else:
        clone_command += f"\"{data_sources[uid]['source']}\""

    # place the output in the specified directory
    split_command = shlex.split(clone_command)
    split_command.append(f"{version_dir}")

    print(" ".join(split_command))

    subprocess.check_call(split_command)

    if prune_lfs:
        # NOTE: we make this optional because older git versions don't support "-f --recent"
        assert (
            which("git-lfs") is not None
        ), "`git-lfs` is not installed, cannot prune and won't get lfs files, only links. Install and try again or re-run with `--no-prune`."

        # prune the repo to reduce wasted memory consumption
        prune_command = "git lfs prune -f --recent"
        subprocess.check_call(shlex.split(prune_command), cwd=version_dir)


def checkout_repo_tag(repo: Repo, version_dir: str, tag: str):
    """
    Checkout the specified tag for an existing repo with git python API.
    """
    print(f" checking out {tag} and pulling changes from repo.")
    repo.remote().fetch(f"{tag}")
    # using git commandline wrapper, so installed lfs should be used to pull
    repo.git.checkout(f"{tag}")
    if which("git-lfs") is None:
        print(" WARNING: git-lfs is not installed, cannot pull lfs files from links.")
    else:
        # NOTE: repo.remote().pull() was not correctly using lfs, so calling it directly in case lfs was installed between runs.
        subprocess.check_call(shlex.split("git lfs pull"), cwd=version_dir)


def get_and_place_compressed_package(
    uid: str,
    data_path: str,
    version_dir: str,
    downloaded_file_list: Optional[str],
    requires_auth: bool,
    username: Optional[str] = None,
    password: Optional[str] = None,
):
    """
    Downloads and unpacks a datasource hosted as a compressed package at a URL.
    Handles authentication for gated sources.
    """
    download_pre_args = data_sources[uid].get("download_pre_args", "")
    download_post_args = data_sources[uid].get("download_post_args", "")
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

    # clean-up
    os.remove(data_path + package_name)


def download_and_place(
    uid: str,
    data_path: str,
    username: Optional[str] = None,
    password: Optional[str] = None,
    replace: Optional[bool] = None,
    prune_lfs: bool = True,
):
    r"""Data-source download function. Validates uid, handles existing data version, downloads data, unpacks, writes version, cleans up."""
    if not data_sources.get(uid):
        print(f"Data download failed, no datasource named {uid}")
        return

    is_repo = data_sources[uid]["source"].endswith(".git")
    if is_repo and which("git-lfs") is None:
        print(
            "-\nWARNING: repo datasource detected and git-lfs is not installed. Download will be limited to lfs link files instead of full assets. \nTo address this, abort and clean datasources. Then install git lfs:\nWith Linux:\n `sudo apt install git-lfs`\n 'git lfs install'\n-"
        )
    link_path = pathlib.Path(data_sources[uid]["link"])
    version_tag = data_sources[uid]["version"]
    version_dir = get_version_dir(uid, data_path, is_repo)
    downloaded_file_list = get_downloaded_file_list(uid, data_path)

    # check for current version
    if os.path.exists(version_dir) and (
        downloaded_file_list is None or os.path.exists(downloaded_file_list)
    ):
        # for existing repos, checkout the specified version
        if is_repo:
            print(f"Found the existing repo for ({uid}): {version_dir}")
            repo = Repo(version_dir)
            assert not repo.bare
            checkout_repo_tag(repo, version_dir, version_tag)
        else:
            print(
                f"Existing data source ({uid}) version ({version_tag}) is current. Data located: '{version_dir}'. Symblink: '{link_path}'."
            )

        replace_existing = (
            replace if replace is not None else prompt_yes_no("Replace versioned data?")
        )

        if replace_existing and not is_repo:
            clean_data(uid, data_path)
        else:
            print("=======================================================")
            if not replace_existing:
                print(f"Not replacing data, generating symlink ({link_path}).")
            print(f"Generating symlink ({link_path}).")
            print("=======================================================")

            if link_path.exists():
                os.unlink(link_path)
            elif not link_path.parent.exists():
                link_path.parent.mkdir(parents=True, exist_ok=True)
            os.symlink(src=version_dir, dst=link_path, target_is_directory=True)
            assert link_path.exists(), "Failed, no symlink generated."

            return

    # download new version
    requires_auth = data_sources[uid].get("requires_auth", False)
    if requires_auth:
        assert username is not None, "Username required, please enter with --username"
        assert (
            password is not None
        ), "Password is required, please enter with --password"

    if data_sources[uid]["source"].endswith(".git"):
        # git dataset, clone it
        clone_repo_source(
            uid, version_dir, requires_auth, username, password, prune_lfs
        )
    else:
        # compressed package dataset (e.g. .zip or .tar), download and unpack it
        get_and_place_compressed_package(
            uid,
            data_path,
            version_dir,
            downloaded_file_list,
            requires_auth,
            username,
            password,
        )

    # create a symlink to the new versioned data
    if link_path.exists():
        os.unlink(link_path)
    elif not link_path.parent.exists():
        link_path.parent.mkdir(parents=True, exist_ok=True)
    os.symlink(src=version_dir, dst=link_path, target_is_directory=True)

    assert link_path.exists(), "Unpacking failed, no symlink generated."

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
    parser.add_argument(
        "--no-prune",
        action="store_true",
        help="Optionally disable pruning for git-lfs repo datasources. Use this if your system git version does not support forced pruning (e.g. Ubuntu 20.x).",
    )

    args = parser.parse_args(args)
    replace = args.replace

    default_data_path = "./data"
    data_path = os.path.realpath(
        args.data_path if args.data_path else default_data_path
    )
    if not args.data_path:
        print(
            f"No data-path provided, defaults to: {default_data_path}. Use '--data-path' to specify another location."
        )
        if os.path.islink(default_data_path):
            print(
                f"Note, {default_data_path} is a symbolic link that points to {data_path}."
            )

    try:
        os.makedirs(data_path, exist_ok=True)
    except OSError:
        traceback.print_exc(file=sys.stdout)
        print("----------------------------------------------------------------")
        print("Aborting download, failed to create data_path.")
        print("Try providing --data-path (e.g. '/path/to/habitat-sim/data/')")
        print("----------------------------------------------------------------")
        parser.print_help()
        exit(2)

    # initialize data_sources and data_groups with test and example assets
    data_path = os.path.abspath(data_path) + "/"
    initialize_test_data_sources(data_path=data_path)

    # validation: ids are unique between groups and sources
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
                    uid,
                    data_path,
                    args.username,
                    args.password,
                    replace,
                    prune_lfs=(not args.no_prune),
                )


if __name__ == "__main__":
    main(sys.argv[1:])
