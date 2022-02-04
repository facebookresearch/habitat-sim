#!/usr/bin/env python

# mosify_scene_instances : utility to modify a set of existing scene instance files.
# In this case, we are modifying the transformations specified in the scene instance
# to account for objects being repivoted

import os
from os.path import join

import config_utils as ut
import glb_mesh_tools as gut

# directory where datasets are found
DATASET_DIR = os.path.join(
    os.path.expanduser("~"), "Facebook/habitat-sim/data/scene_datasets/"
)

# specify directory where original scene instance configs can be found. Will crawl all
# directories under specified directory looking for json configs whose names
# match given regex search paramet-ers.

SCENE_INSTANCE_SRC_DIR = DATASET_DIR + "replicaCAD_working/configs_v1.0/scenes/"

# specify directory where to place modified scene instances
MOD_OBJS_OUTPUT = DATASET_DIR + "replicaCAD/configs/scenes_mod/"
os.makedirs(MOD_OBJS_OUTPUT, exist_ok=True)

# file holding object modifications - name of object, transform, and magnitude of transform
OBJECT_MOD_FILENAME = os.path.join(
    os.path.expanduser("~"),
    "Facebook/habitat-sim/data/scene_datasets/replicaCAD_working/object_tests/origin_transforms_origin_center_of_mass_median.txt",
)

# desired  scene instance config filename regex, not including '.json'
SCENE_INSTANCE_FILE_RE = "^apt_[0-9].scene_instance"


def main():

    # get all files in specified directory whose names match passed regex string
    # returns list of tuples of path,dirnames, file names
    file_list = ut.get_files_matching_regex(
        SCENE_INSTANCE_SRC_DIR, f"{SCENE_INSTANCE_FILE_RE}.json"
    )

    with open(OBJECT_MOD_FILENAME, "r") as f:
        file_lines = f.readlines()
    # build a dictionary keyed by object name that
    mod_dict = {
        line.split(":")[0].strip(): [
            float(v.strip()) for v in line.split("[")[-1].split("]")[0].split(",")
        ]
        for line in file_lines
    }

    # load and process each scene instance
    for path, _, filename in file_list:
        src_file = join(path, filename)
        # this is the original scene instance file
        orig_scene_instance = ut.load_json_into_dict(src_file)

        # modify all elements in object instance dictionary
        for obj_instance in orig_scene_instance["object_instances"]:
            obj_name = obj_instance["template_name"].split("/")[-1].strip()
            if obj_name not in mod_dict:
                print(f"Unable to find {obj_name} in modification dictionary")
                continue
            orig_trans = list(obj_instance["translation"])
            orig_rotation = list(obj_instance["rotation"])
            # modification needs to be mod vector rotated by object's rotation to account for change in origin
            pivot_displacement = mod_dict[obj_name]

            mod_val = gut.rotate_vector_by_quat_list(pivot_displacement, orig_rotation)
            for i in range(len(orig_trans)):
                obj_instance["translation"][i] += mod_val[i]

        # save to new file location
        new_file = join(MOD_OBJS_OUTPUT, filename)
        print(f"Scene Instance : {src_file} modified and saved to {new_file}\n")

        ut.mod_json_val_and_save(("", new_file, orig_scene_instance, False))


if __name__ == "__main__":
    main()
