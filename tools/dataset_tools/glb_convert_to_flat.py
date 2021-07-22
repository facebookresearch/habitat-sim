#!/usr/bin/env python

# glb_convert_to_flat : will take all glb files in the given source
# directory, convert them to be flat shaded, and save them to the
# given destination directory.

import os

import config_utils as ut
import glb_mesh_tools as gut

# Source directory to walk for" glb files we wish to convert to flat.
GLB_SOURCE_DIR = "/home/john/Facebook/habitat-sim/data/scene_datasets/replicaCAD_working/baking/baked_urdf/"

# Destination directory to place modified glbs
GLB_UNLIT_DEST_DIR = "/home/john/Facebook/habitat-sim/data/scene_datasets/replicaCAD_working/baking/unlit_urdf/"


def main():

    # get all files in specified directory whose names match passed regex string
    # returns list of tuples of path,dirnames, file names
    src_file_list = ut.get_files_matching_regex(GLB_SOURCE_DIR, ".*\\.glb")
    dest_path_dict = {
        path: path.replace(GLB_SOURCE_DIR, GLB_UNLIT_DEST_DIR)
        for path, _, _ in src_file_list
    }
    for path, _, filename in src_file_list:
        src_filename = os.path.join(path, filename)
        src_file_exists = os.path.exists(src_filename)
        if not src_file_exists:
            print("!!!!!! {} does not exist, skipping.".format(src_filename))
            continue

        dest_path = dest_path_dict[path]
        dest_path_exists = os.path.exists(dest_path)
        if not dest_path_exists:
            # create dest path
            os.makedirs(dest_path, exist_ok=True)
            dest_path_exists = os.path.exists(dest_path)

        dest_filename = os.path.join(dest_path, filename)
        print(
            "Src File exists : \t{} | Src Filename : {}\n\tDest Path exists : {} | Dest Filename : {}".format(
                src_file_exists,
                src_filename,
                dest_path_exists,
                dest_filename,
            )
        )

        gut.convert_file_to_unlit(src_filename, dest_filename)

        print("\n")
        # remove temporary


if __name__ == "__main__":
    main()
