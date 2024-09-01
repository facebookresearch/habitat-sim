# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import argparse
import json
import os
from typing import Callable, List


def file_is_scene_config(filepath: str) -> bool:
    """
    Return whether or not the file is an scene_instance.json
    """
    return filepath.endswith(".scene_instance.json")


def find_files(root_dir: str, discriminator: Callable[[str], bool]) -> List[str]:
    """
    Recursively find all filepaths under a root directory satisfying a particular constraint as defined by a discriminator function.

    :param root_dir: The roor directory for the recursive search.
    :param discriminator: The discriminator function which takes a filepath and returns a bool.

    :return: The list of all absolute filepaths found satisfying the discriminator.
    """
    filepaths: List[str] = []

    if not os.path.exists(root_dir):
        print(" Directory does not exist: " + str(dir))
        return filepaths

    for entry in os.listdir(root_dir):
        entry_path = os.path.join(root_dir, entry)
        if os.path.isdir(entry_path):
            sub_dir_filepaths = find_files(entry_path, discriminator)
            filepaths.extend(sub_dir_filepaths)
        # apply a user-provided discriminator function to cull filepaths
        elif discriminator(entry_path):
            filepaths.append(entry_path)
    return filepaths


def remove_ssd_from_scene_instance_json(filepath: str):
    """
    Strips any 'semantic_scene_instance' field from a scene_instance.json files and re-exports it.
    """
    assert filepath.endswith(".scene_instance.json"), "Must be a scene instance JSON."

    file_is_modified = False
    scene_conf = None
    with open(filepath, "r") as f:
        scene_conf = json.load(f)
        if "semantic_scene_instance" in scene_conf:
            scene_conf.pop("semantic_scene_instance")
            file_is_modified = True

    # write the data as necessary
    if file_is_modified and scene_conf is not None:
        with open(filepath, "w") as f:
            json.dump(scene_conf, f)


def main():
    parser = argparse.ArgumentParser(
        description="Remove all 'semantic_scene_instance' fields from scene_instnace files in the dataset."
    )
    parser.add_argument(
        "--dataset-root-dir",
        type=str,
        help="path to HSSD SceneDataset root directory containing 'fphab-uncluttered.scene_dataset_config.json'.",
    )
    args = parser.parse_args()
    fp_root_dir = args.dataset_root_dir
    config_root_dir = os.path.join(fp_root_dir, "scenes-uncluttered")
    configs = find_files(config_root_dir, file_is_scene_config)

    for _ix, filepath in enumerate(configs):
        remove_ssd_from_scene_instance_json(filepath)


if __name__ == "__main__":
    main()
