# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import argparse
import json
import os
from typing import Any, Callable, Dict, List


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


def get_scene_instance_json(filepath: str) -> List[str]:
    """
    Load a scene instance JSON as a dict and return it.
    """
    assert filepath.endswith(".scene_instance.json"), "Must be a scene instance JSON."

    with open(filepath, "r") as f:
        scene_conf = json.load(f)
        return scene_conf


def save_scene_instance_json(filepath: str, scene_dict: Dict[str, Any]) -> None:
    with open(filepath, "w") as f:
        json.dump(scene_dict, f, indent=4)


def get_scene_id_from_filepath(filepath: str) -> str:
    return filepath.split("/")[-1].split(".")[0]


def scene_has_semantic_map(scene_dict: Dict[str, Any]) -> bool:
    if "semantic_scene_instance" in scene_dict:
        if scene_dict["semantic_scene_instance"] == "hssd_ssd_map":
            return True
        return None
    return None


def write_self_semantic(filepath: str) -> None:
    scene_dict = get_scene_instance_json(filepath)
    scene_id = get_scene_id_from_filepath(filepath)
    scene_dict["semantic_scene_instance"] = f"{scene_id}.semantic_config.json"
    save_scene_instance_json(filepath, scene_dict)


def clear_articulated_object_states(filepath: str) -> None:
    scene_dict = get_scene_instance_json(filepath)
    if "articulated_object_instances" in scene_dict:
        for ao_instance in scene_dict["articulated_object_instances"]:
            if "initial_joint_pose" in ao_instance:
                del ao_instance["initial_joint_pose"]
            if "initial_joint_velocities" in ao_instance:
                del ao_instance["initial_joint_velocities"]
        save_scene_instance_json(filepath, scene_dict)


def set_articulated_objects_motion_type(
    filepath: str, motion_type: str = "static"
) -> None:
    """
    Set all AOs in the scene instance to the specified motion type.
    Choices are "static", "dynamic", or "kinematic"
    """
    assert motion_type in ["static", "dynamic", "kinematic"]
    scene_dict = get_scene_instance_json(filepath)
    if "articulated_object_instances" in scene_dict:
        for ao_instance in scene_dict["articulated_object_instances"]:
            ao_instance["motion_type"] = motion_type
    save_scene_instance_json(filepath, scene_dict)


def set_path_to_scene_filter_file(filepath: str, rel_filter_path: str) -> None:
    """
    Sets the scene's receptacle filter path.
    """

    assert rel_filter_path.endswith(".json")

    scene_dict = get_scene_instance_json(filepath)
    if "user_defined" not in scene_dict:
        scene_dict["user_defined"] = {}
    scene_dict["user_defined"]["scene_filter_file"] = rel_filter_path
    save_scene_instance_json(filepath, scene_dict)


def clear_user_defined(filepath: str):
    """
    Clear any user_defined subconfigs from a scene instance file leaving behind only the filter file.
    """
    scene_dict = get_scene_instance_json(filepath)
    if (
        "user_defined" in scene_dict
        and "scene_filter_file" in scene_dict["user_defined"]
    ):
        # allow the filter filepath
        scene_dict["user_defined"] = {
            "scene_filter_file": scene_dict["user_defined"]["scene_filter_file"]
        }
    else:
        del scene_dict["user_defined"]
    save_scene_instance_json(filepath, scene_dict)


def main():
    parser = argparse.ArgumentParser(
        description="Remove all 'semantic_scene_instance' fields from scene_instnace files in the dataset."
    )
    parser.add_argument(
        "--scenes-dir",
        type=str,
        help="Path to a directory containing the relevant '*.scene_dataset_config.json'. These will be overwritten.",
    )
    parser.add_argument(
        "--scenes",
        nargs="+",
        type=str,
        help="One or more scene ids to limit modifications to a subset of the instance.",
    )

    args = parser.parse_args()
    scenes_dir = args.scenes_dir
    configs = find_files(scenes_dir, file_is_scene_config)

    if args.scenes is not None:
        configs_subset = []
        for config in configs:
            for scene_id in args.scenes:
                if scene_id in config:
                    configs_subset.append(config)
                    break
        configs = configs_subset

    # set path to scene_filter_files
    if True:
        for config in configs:
            scene_name = config.split("/")[-1].split(".scene_instance.json")[0]
            filter_path = f"scene_filter_files/siro_scene_filter_files/{scene_name}.rec_filter.json"
            set_path_to_scene_filter_file(config, filter_path)

    # set all AOs to STATIC
    if True:
        for config in configs:
            print(f"Setting ao motion_type for {config}")
            set_articulated_objects_motion_type(config, motion_type="static")

    # remove articulated object states:
    if True:
        for config in configs:
            print(f"Removing ao states from {config}")
            clear_articulated_object_states(config)

    # remove everything except the filter file from the user_defined
    if True:
        for config in configs:
            print(f"Clearing user_defined from {config}")
            clear_user_defined(config)

    # semantic scene id modifier
    if False:
        num_w_semantic = 0
        for _ix, filepath in enumerate(configs):
            write_self_semantic(filepath)
            scene_dict = get_scene_instance_json(filepath)
            if scene_has_semantic_map(scene_dict):
                print(f"yes = {filepath}")
                num_w_semantic += 1
            else:
                print(f"no = {filepath}")

        print(
            f"Total scenes with semantic specifier = {num_w_semantic} / {len(configs)}"
        )


if __name__ == "__main__":
    main()
