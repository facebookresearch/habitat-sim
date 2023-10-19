# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import argparse
import os
from typing import Callable, List


def file_endswith(filepath: str, end_str: str) -> bool:
    """
    Return whether or not the file ends with a string.
    """
    return filepath.endswith(end_str)


def find_files(
    root_dir: str, discriminator: Callable[[str, str], bool], disc_str: str
) -> List[str]:
    """
    Recursively find all filepaths under a root directory satisfying a particular constraint as defined by a discriminator function.

    :param root_dir: The roor directory for the recursive search.
    :param discriminator: The discriminator function which takes a filepath and discriminator string and returns a bool.

    :return: The list of all absolute filepaths found satisfying the discriminator.
    """
    filepaths: List[str] = []

    if not os.path.exists(root_dir):
        print(" Directory does not exist: " + str(dir))
        return filepaths

    for entry in os.listdir(root_dir):
        entry_path = os.path.join(root_dir, entry)
        if os.path.isdir(entry_path):
            sub_dir_filepaths = find_files(entry_path, discriminator, disc_str)
            filepaths.extend(sub_dir_filepaths)
        # apply a user-provided discriminator function to cull filepaths
        elif discriminator(entry_path, disc_str):
            filepaths.append(entry_path)
    return filepaths


def run_armature_urdf_conversion(blend_file: str, export_path: str, script_path: str):
    assert os.path.exists(blend_file), f"'{blend_file}' does not exist."
    os.makedirs(export_path, exist_ok=True)
    base_command = f"blender {blend_file} --background --python {script_path} -- --export-path {export_path}"
    # first export the meshes
    os.system(base_command + " --export-meshes")
    # then export the URDF
    os.system(base_command)


# -----------------------------------------
# Batches blender converter calls over a directory of blend files
# e.g. python tools/batched_armature_to_urdf.py --root-dir ~/Downloads/OneDrive_1_9-27-2023/ --out-dir tools/armature_out_test/ --converter-script-path tools/blender_armature_to_urdf.py
# e.g. add " --skip-strings wardrobe" to skip all objects with "wardrobe" in the filepath
# -----------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Run Blender Armature to URDF converter for all .blend files in a directory."
    )
    parser.add_argument(
        "--root-dir",
        type=str,
        help="Path to a directory containing .blend files for conversion.",
    )
    parser.add_argument(
        "--out-dir",
        type=str,
        help="Path to a directory for exporting URDF and assets.",
    )
    parser.add_argument(
        "--converter-script-path",
        type=str,
        help="Path to blender_armature_to_urdf.py.",
        default="tools/blender_armature_to_urdf.py",
    )
    parser.add_argument(
        "--skip-strings",
        nargs="+",
        type=str,
        help="Substrings which indicate a path which should be skippped. E.g. an object hash '6f57e5076e491f54896631bfe4e9cfcaa08899e2' to skip that object's blend file.",
        default=None,
    )
    args = parser.parse_args()
    root_dir = args.root_dir
    assert os.path.isdir(root_dir), "directory must exist."
    assert os.path.exists(
        args.converter_script_path
    ), f"provided script path '{args.converter_script_path}' does not exist."

    # get blend files
    blend_paths = find_files(root_dir, file_endswith, ".blend")
    if args.skip_strings is not None:
        skipped_strings = [
            path
            for skip_str in args.skip_strings
            for path in blend_paths
            if skip_str in path
        ]
        blend_paths = list(set(blend_paths) - set(skipped_strings))

    for blend_path in blend_paths:
        run_armature_urdf_conversion(
            blend_file=blend_path,
            export_path=args.out_dir,
            script_path=args.converter_script_path,
        )


if __name__ == "__main__":
    main()
