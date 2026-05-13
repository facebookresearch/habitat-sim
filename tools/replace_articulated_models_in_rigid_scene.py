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


def file_is_urdf(filepath: str) -> bool:
    """
    Return whether or not the file is a .urdf
    """
    return filepath.endswith(".urdf")


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


scenes_without_filters = []


def find_and_replace_articulated_models_for_config(
    filepath: str,
    top_dir: str,
    urdf_names: str,
    src_dir: str = "scenes-uncluttered",
    dest_dir: str = "scenes-articulated-uncluttered",
) -> None:
    """
    For a given scene config, try to find a matching articulated objects for each rigid object. If found, add them to the config, replacing the rigid objects.

    :param top_dir: The top directory of the dataset from which the rec filter file path should be relative.
    """
    assert filepath.endswith(".scene_instance.json"), "Must be a scene instance JSON."
    scene_name = filepath.split(".scene_instance.json")[0].split("/")[-1]

    print(f"scene_name = {scene_name}")

    file_is_modified = False
    with open(filepath, "r") as f:
        scene_conf = json.load(f)

        ao_instance_data = []
        if "articulated_object_instances" in scene_conf:
            ao_instance_data = scene_conf["articulated_object_instances"]

        modified_object_instance_data = []
        for object_instance_data in scene_conf["object_instances"]:
            object_name = object_instance_data["template_name"]

            # look for a matching articulated object entry
            urdf_name_match = None
            for urdf_name in urdf_names:
                if object_name in urdf_name:
                    urdf_name_match = urdf_name
                    break

            # create the modified JSON data
            if urdf_name_match is None:
                # add the object to the modified list
                modified_object_instance_data.append(object_instance_data)
            else:
                file_is_modified = True
                # TODO: all objects are non-uniformly scaled and won't fit exactly in the scenes...
                # assert "non_uniform_scale" not in object_instance_data, "Rigid object is non-uniformaly scaled. Cannot replace with equivalent articulated object."
                this_ao_instance_data = {
                    "template_name": urdf_name_match,
                    "translation_origin": "COM",
                    "fixed_base": True,
                    "motion_type": "DYNAMIC",
                }
                if "translation" in object_instance_data:
                    this_ao_instance_data["translation"] = object_instance_data[
                        "translation"
                    ]
                if "rotation" in object_instance_data:
                    this_ao_instance_data["rotation"] = object_instance_data["rotation"]
                ao_instance_data.append(this_ao_instance_data)

        scene_conf["object_instances"] = modified_object_instance_data
        scene_conf["articulated_object_instances"] = ao_instance_data

    if file_is_modified:
        filepath = filepath.split(src_dir)[0] + dest_dir + filepath.split(src_dir)[-1]
        with open(filepath, "w") as f:
            json.dump(scene_conf, f, indent=4)


def main():
    parser = argparse.ArgumentParser(
        description="Modify the scene_instance.json files, replacing rigid objects with articulated coutnerparts in a urdf/ directory."
    )
    parser.add_argument(
        "--dataset-root-dir",
        type=str,
        help="path to HSSD SceneDataset root directory containing 'fphab-uncluttered.scene_dataset_config.json'.",
    )
    parser.add_argument(
        "--src-dir",
        type=str,
        default="scenes-uncluttered",
        help="Name of the source scene config directory within root-dir.",
    )
    parser.add_argument(
        "--dest-dir",
        type=str,
        default="scenes-articulated-uncluttered",
        help="Name of the destination scene config directory within root-dir. Will be created if doesn't exist.",
    )
    parser.add_argument(
        "--scenes",
        nargs="+",
        type=str,
        help="Substrings which indicate scenes which should be converted. When provided, only these scenes are converted.",
        default=None,
    )
    args = parser.parse_args()
    fp_root_dir = args.dataset_root_dir
    src_dir = args.src_dir
    dest_dir = args.dest_dir
    config_root_dir = os.path.join(fp_root_dir, src_dir)
    configs = find_files(config_root_dir, file_is_scene_config)
    urdf_dir = os.path.join(fp_root_dir, "urdf/")
    urdf_files = find_files(urdf_dir, file_is_urdf)

    # create scene output directory
    os.makedirs(os.path.join(fp_root_dir, dest_dir), exist_ok=True)

    invalid_urdf_files = []

    # only consider urdf files with reasonable accompanying contents
    def urdf_has_meshes_and_config(urdf_filepath: str) -> bool:
        """
        Return whether or not there are render meshes and a config accompanying the urdf.
        """
        if not os.path.exists(urdf_filepath.split(".urdf")[0] + ".ao_config.json"):
            return False
        has_render_mesh = False
        for file_name in os.listdir(os.path.dirname(urdf_filepath)):
            if file_name.endswith(".glb") and "receptacle" not in file_name:
                has_render_mesh = True
                break
        return has_render_mesh

    valid_urdf_files = [
        urdf_file for urdf_file in urdf_files if urdf_has_meshes_and_config(urdf_file)
    ]

    invalid_urdf_files = [
        urdf_file for urdf_file in urdf_files if urdf_file not in valid_urdf_files
    ]

    urdf_names = [
        urdf_filename.split("/")[-1].split(".urdf")[0]
        for urdf_filename in valid_urdf_files
    ]

    # limit the scenes which are converted
    if args.scenes is not None:
        scene_limited_configs = []
        for scene in args.scenes:
            for config in configs:
                if scene + "." in config:
                    scene_limited_configs.append(config)
        configs = list(set(scene_limited_configs))

    for _ix, filepath in enumerate(configs):
        find_and_replace_articulated_models_for_config(
            filepath,
            urdf_names=urdf_names,
            top_dir=fp_root_dir,
            src_dir=src_dir,
            dest_dir=dest_dir,
        )

    print(
        f"Migrated {len(valid_urdf_files)} urdfs into {len(configs)} scene configs. Invalid urdfs found and skipped ({len(invalid_urdf_files)}) = {invalid_urdf_files}"
    )


if __name__ == "__main__":
    main()
