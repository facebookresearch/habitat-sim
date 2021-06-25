#!/usr/bin/env python

# set_semantic_ids - tool for extracting semantic ids and class names
# for objects from a json file and adding the semantic id to object
# configuration files corresponding to the class name

import json
import os
from os.path import join

import config_utils as ut

# Whether or not to build ReplicaCAD Semantic Lexicon from replica src
BUILD_REPLICACAD_LEXICON = True
# Whether or not to modify existing json files by adding semantic ID
ADD_SEMANTIC_ID_TO_JSON = False

# whether or not to update paths to assets in config to be absolute so that saving
# this file in an alternate location does not break the config load
ABSOLUTE_ASSET_PATHS = True


# Where ReplicaCAD data is located, relative to this file
REPLICACAD_DIR = "../../data/scene_datasets/replicaCAD"
# where modified json configs should be written
CONFIG_OUTPUT_DIR = join(REPLICACAD_DIR, "configs/objects")
# CONFIG_OUTPUT_DIR = REPLICACAD_DIR + "_sid_configs/"
os.makedirs(CONFIG_OUTPUT_DIR, exist_ok=True)

# where to put/search for the ssd lexicon
SSD_OUTPUT_DIR = join(REPLICACAD_DIR, "ssd")

# Where original Replica house file is located, relative to this
# file, to use for class and id derivation
SRC_SEMANTIC_FILENAME = (
    "../../data/scene_datasets/replica_dataset/apartment_0/habitat/info_semantic.json"
)
# where replicaCAD semantic lexicon should reside (either to be written or read)
REPLICACAD_SEMANTIC_FILENAME = join(SSD_OUTPUT_DIR, "replicaCAD_semantic_lexicon.json")


# where scene stats will be written
STATS_OUTPUT_DIR = join(REPLICACAD_DIR, "scene_stats")
# os.makedirs(STATS_OUTPUT_DIR, exist_ok=True)

# where the object configs live
OBJECT_CONFIG_DIR = join(REPLICACAD_DIR, "objects/convex/")

# source of scene instance files
SCENE_CONFIG_DIR = join(REPLICACAD_DIR, "scenes")


# Read the semantic file describing classes and their class IDs,
# and assign each object's config a value for "semantic_id"
# corresponding to the class that is present in the config file name.
def set_object_semantic_ids(config_json_dict, lex_dict):
    # build list of semantic names sorted by length, so that more
    # precise matches will be appropriately mapped by overwriting
    # mappings using smaller (i.e. more general) class names
    # (wall_cabinet vs wall)
    sorted_key_list = sorted(lex_dict.keys(), key=len)

    # info dict is keyed by filename, and holds tuples containing
    # (src filename, dest filename, json tag, json val)
    info_dict = {}
    num_classes = 0
    # go through sorted_key_list in order of increasing length,
    # find entries that match, put in dictionary.
    for class_key in sorted_key_list:
        semantic_id = int(lex_dict[class_key])
        json_val_dict = {"semantic_id": semantic_id}
        for config_filename, dirs in config_json_dict.items():
            # construct key name proposals to filter internal
            # substrings but also allow for numbered instances
            check_key1 = "_" + class_key + "_"
            check_key2 = "_" + class_key + "."

            if (check_key1 in config_filename) or (check_key2 in config_filename):
                info_dict[config_filename] = (
                    dirs[0],
                    dirs[1],
                    json_val_dict,
                    ABSOLUTE_ASSET_PATHS,
                )
        num_classes += 1

    # find files that have not been mapped
    print("\nUNMAPPED OBJECT CONFIGS - these will be hand-mapped. : \n")
    unmapped_configs = {}
    for k, v in config_json_dict.items():
        # if file name not found in mapped dictionary
        if k not in info_dict:
            unmapped_configs[k] = v

    for k, _ in unmapped_configs.items():
        print('"{}"'.format(k))
    print("\n")

    # the objects below were not named appropriately and so matches were not found
    # with existing Replica classes.  Class values are being hardcoded for these,
    # based on inspecting the object and assigning a label
    replaced_IDs = {
        "frl_apartment_tv_object.object_config.json": int(lex_dict["tablet"]),
        "frl_apartment_shoebox_01.object_config.json": int(lex_dict["box"]),
        "frl_apartment_setupbox.object_config.json": int(lex_dict["tablet"]),
        "frl_apartment_choppingboard_02.object_config.json": int(
            lex_dict["chopping_board"]
        ),
        "frl_apartment_remote-control_01.object_config.json": int(
            lex_dict["remote_control"]
        ),
        "frl_apartment_knifeblock.object_config.json": int(lex_dict["knife_block"]),
        "frl_apartment_sponge_dish.object_config.json": int(
            lex_dict["kitchen_utensil"]
        ),
        "frl_apartment_tvstand.object_config.json": int(lex_dict["tv_stand"]),
    }

    #  add hand-mapped entries to result dict
    for k, v in unmapped_configs.items():
        semantic_id = replaced_IDs[k]
        json_val_dict = {"semantic_id": semantic_id}
        info_dict[k] = (v[0], v[1], json_val_dict, ABSOLUTE_ASSET_PATHS)
    # return results
    return info_dict


def semantic_id_mapping_report(info_dict, lex_dict):
    # Provides an accounting of each semantic class, and how many objects belong to it
    mapped_classes = {}
    # mapping of id to name
    lex_inverse_dict = {}
    for k, v in lex_dict.items():
        mapped_classes[k] = 0
        lex_inverse_dict[v] = k

    # modify object configs with deduced semantic IDs
    print("Object mappings : \n")
    itr = 0
    for k, v in info_dict.items():
        semantic_id = v[2]["semantic_id"]
        semantic_class = lex_inverse_dict[semantic_id]
        print(
            "{} | k: {} : Semantic ID: {} : class: {}".format(
                itr, k, semantic_id, semantic_class
            )
        )
        mapped_classes[semantic_class] += 1
        itr += 1

    print("\n{} Total Object configuration files processed\n".format(itr))

    print("\nMapped and Unmapped classes to existing object configs:\n")
    for k, v in mapped_classes.items():
        if v != 0:
            print("mapped class : {} : ID: {} count : {}".format(k, lex_dict[k], v))
    print("\n")
    for k, v in mapped_classes.items():
        if v == 0:
            print("unmapped class : {} : ID: {} ".format(k, lex_dict[k]))


def save_modified_json_files(info_dict):
    # save results to file
    for _, v in info_dict.items():
        if len(v) > 2:
            ut.mod_json_val_and_save(v)
        else:
            ut.mod_json_val_and_save((v[0], v[1], {}, ABSOLUTE_ASSET_PATHS))


# This will load the scene instance configs and analyze the objects
# placed within each, including their semantic ID, counts, etc.
# The results are returned in a dictionary
def load_replicaCAD_scene_instances(stats_output_dir):
    # key is scene file name, value is tuple with input dir and output dir
    scene_json_dict = ut.build_config_src_dest_dict(
        SCENE_CONFIG_DIR,
        stats_output_dir,
        predicate=lambda x: "static_furniture" not in x,
        debug=True,
    )

    for k, v in scene_json_dict.items():
        print("k : {} | v : {}".format(k, v))

    return scene_json_dict


# either build a new ReplicaCAD semantic dictionary of class->ID mappings,
# or load the existing one
def replicaCAD_get_semantic_lexicon(
    build_replicaCAD_lexicon, SRC_SEMANTIC_FILENAME, REPLICACAD_SEMANTIC_FILENAME
):
    # Load semantic mappings between classes and ids into a dictionary from
    # Replica "house" file, and save these mappings in a new file
    def build_semantic_lexicon(semantic_filename):
        # build dictionary of class names and semantic IDs
        lex_dict = {}
        with open(semantic_filename, "r") as src:
            data = json.load(src)
            for elem in data["classes"]:
                # replace hyphens to match config file names
                lex_dict[elem["name"].replace("-", "_")] = elem["id"]
        return lex_dict

    if build_replicaCAD_lexicon:
        # load semantic lexicon and save results
        lex_dict = build_semantic_lexicon(SRC_SEMANTIC_FILENAME)
        # save json list object holding name and id entries
        # for all potential classes in ReplicaCAD
        lex_json = [{"name": k, "id": v} for k, v in lex_dict.items()]
        out_dict = {"classes": lex_json}
        with open(REPLICACAD_SEMANTIC_FILENAME, "w") as dest:
            json.dump(out_dict, dest, indent=2)
    else:
        # load semantic lexicon and save results
        lex_dict = build_semantic_lexicon(REPLICACAD_SEMANTIC_FILENAME)

    return lex_dict


def main():

    # Build a dictionary keyed by config file name where value is a tuple holding the fully
    # qualified source path to the file and the fully qualified dest path for the file.
    config_json_dict = ut.build_config_src_dest_dict(
        OBJECT_CONFIG_DIR, CONFIG_OUTPUT_DIR, debug=True
    )

    if ADD_SEMANTIC_ID_TO_JSON:
        # Either build and save, or load an existing, ReplicaCAD semantic lexicon mapping classes and IDs
        lex_dict = replicaCAD_get_semantic_lexicon(
            BUILD_REPLICACAD_LEXICON,
            SRC_SEMANTIC_FILENAME,
            REPLICACAD_SEMANTIC_FILENAME,
        )
        # create dictionary of mappings from object config files
        # to semantic ids, with input and output file locations
        obj_map_dict = set_object_semantic_ids(config_json_dict, lex_dict)
        # print out a report of results
        semantic_id_mapping_report(obj_map_dict, lex_dict)

        save_modified_json_files(obj_map_dict)
        # just moving config files
    else:
        save_modified_json_files(config_json_dict)

    # build dictionary of results from analyzing all scene instance jsons
    # scene_obj_res_dict = load_replicaCAD_scene_instances(STATS_OUTPUT_DIR)


if __name__ == "__main__":
    main()
