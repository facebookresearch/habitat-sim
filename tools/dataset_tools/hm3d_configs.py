#!/usr/bin/env python

# hm3d_configs.py : This program will synthesize all the stage
# configs required for HM3D dataset.  Currently we do not use
# this to synthesize configs, but rather have hand tooled Scene
# Dataset Configs that use wildecards to support all the scenes
# within a particular partition.

# NOTE : due to the addition of glob/wildcard support to the
# metadata subsystem in Habitat, these configs are not necessary for HM3D.

import os
import re
from os.path import join

import config_utils as ut

# specify directory where HM3D dataset directories can be found.
# This directory is where the Scene Dataset files will be saved
HM3D_BASE_DIR = "/Users/jmturner/Documents/HM3D/habitat"

# data partition names/subdirs
HM3D_DATA_PARTITIONS = ["minival", "test", "train", "val"]

# numeric spans of each data partition.  The directoriues
# holding the scene data are prefixed by numbers in these ranges
HM3D_DATA_PARTITIONS = {
    "minival": (800, 809),
    "test": (900, 999),
    "train": (0, 799),
    "val": (800, 899),
}

# desired scene directory regex
# HM3D scene directories have 5 digits followed by a dash followed
# by the scene hash, 11 alpha-numeric values
HM3D_SCENE_SUBDIR_RE = "[0-9]{5}-[a-z0-9A-Z]{11}"

# All HM3D stages share the same frame and origin value
HM3D_JSON_MOD_VALS = {
    "up": [0, 1, 0],
    "front": [0, 0, -1],
    "origin": [0, 0, 0],
}

# The extension to use to reference stage render asests
HM3D_STAGE_RENDER_ASSET_EXT = ".basis.glb"
# The extension used to reference existing navmesh assets.
# Make empty string if none exists
HM3D_NAVMESH_EXT = ".navmesh"


# process the files within a particular scene directory
# and construct stage configs for each scene
# Returns a tuple holding the scene hash, stage config full filepath and
# navmesh file location, all of which are used by Scene Dataset
# configuration.
def hm3d_build_stage_config(partition, path_and_dir):
    dir_path = path_and_dir[0]
    dir_name = path_and_dir[1]
    scene_dir = join(dir_path, dir_name)
    # construct the scene name
    scene_name = dir_name.split("-")[1]
    # build json dict to write
    json_dict = {}
    # names of render asset and navmesh
    # use relative names in stage config,
    # but return absolute path name to navmesh
    stage_render_asset = scene_name + HM3D_STAGE_RENDER_ASSET_EXT
    if os.path.isfile(join(scene_dir, stage_render_asset)):
        json_dict["render_asset"] = stage_render_asset

    if len(HM3D_NAVMESH_EXT.strip()) > 0:
        navemesh_asset = scene_name + HM3D_NAVMESH_EXT
        abs_navmesh_asset = join(scene_dir, navemesh_asset)
        if os.path.isfile(abs_navmesh_asset):
            # print(f"Navmesh Found : {navemesh_asset}")
            json_dict["navmesh"] = navemesh_asset
        else:
            print(f"Navmesh Not Found : {navemesh_asset}")
            abs_navmesh_asset = ""
    else:
        abs_navmesh_asset = ""

    # set standard values for all HM3D stage configs
    for k, v in HM3D_JSON_MOD_VALS.items():
        json_dict[k] = v

    # print(f"{scene_dir} - {scene_name} | {stage_render_asset} | {navemesh_asset} ")
    stage_config_filename = scene_name + ut.CONFIG_EXTENSIONS["stage"] + ".json"
    abs_stage_config_filename = join(scene_dir, stage_config_filename)
    # compose and save JSON file
    ut.mod_json_val_and_save(("", abs_stage_config_filename, json_dict))
    return (scene_name, abs_stage_config_filename, abs_navmesh_asset)


def build_scene_config():
    pass


def main():
    # file name pattern to find for modification
    config_file_pattern = re.compile(f"{HM3D_SCENE_SUBDIR_RE}")
    # file listings for each partition
    file_listings = {}
    for partition in HM3D_DATA_PARTITIONS:
        sub_dir = join(HM3D_BASE_DIR, partition)
        # set dictionary entry for partition name to be directories list
        # of tuples containing path and directory name of directories
        # matching regex
        file_listings[partition] = ut.get_directories_matching_regex(
            sub_dir, config_file_pattern
        )

    for partition, path_and_dir_list in file_listings.items():
        print(f"Size of partition : {len(path_and_dir_list)}")
        # for each partition, we want a dictionary of the scene hash (which will
        # be used as an access tag in scene dataset config) as well as stage config location and navmesh location
        scene_dataset_vals = {}
        scene_dataset_config_name = join(
            HM3D_BASE_DIR,
            f'HM3D_{partition}_data{ut.CONFIG_EXTENSIONS["dataset"]}.json',
        )
        print(f"Scene dataset config name : {scene_dataset_config_name}")
        # go through every scene
        for path_and_dir in path_and_dir_list:
            # build an individual stage config and return scene hash,
            # absolute stage config location, absolute navmesh location in tuple
            stage_vals = hm3d_build_stage_config(partition, path_and_dir)
            rel_stage_config_path = ut.transform_path_relative(
                stage_vals[1], HM3D_BASE_DIR
            )
            rel_navmesh_path = ut.transform_path_relative(stage_vals[2], HM3D_BASE_DIR)
            # set scene dataset values for each individual scene instance
            scene_dataset_vals[stage_vals[0]] = {
                "stage_config_path": rel_stage_config_path,
                "navmesh_path": rel_navmesh_path,
            }
            # print(
            #    f"Stage config vals returned : Scene : {stage_vals[0]}\n"
            #    f"Base scene dataset dir:{HM3D_BASE_DIR}\n"
            #    f"Converted config rel path : {rel_stage_config_path}\n"
            #    f"Converted navmesh rel path : {rel_navmesh_path}"
            # )
        print(
            f"{partition} partition complete : scene dataset size {len(scene_dataset_vals)}"
        )
        for k, v in scene_dataset_vals.items():
            print(f"Scene : {k} | Data : {v}")
        # TODO create scene dataset for each partition

    # # build dictionary of 4-element tuples to modify json values
    # res_dict = {}

    # sorted_key_list = sorted(res_dict.keys())

    # for k in sorted_key_list:
    #     v = res_dict[k]
    #     print(f"file:{k} | src loc:{v[0]} | dest file:{v[1]} | json tag:{v[2]} | json val:{v[3]}")
    #     mod_json_val_and_save(v)


if __name__ == "__main__":
    main()
