#!/usr/bin/env python


import os

import config_utils as ut
import glb_mesh_tools as gut

# import numpy as np
# import pyglet
import trimesh

HOME = os.path.expanduser("~")
DATASET_SRC_DIR = os.path.join(HOME, "Documents/AI2Thor/")

SCENES_SRC_DIR = DATASET_SRC_DIR + "scenes/"
print(SCENES_SRC_DIR)
# Output for deconned objects

DEST_DIR = DATASET_SRC_DIR + "decon_output/"
print(DEST_DIR)
os.makedirs(DEST_DIR, exist_ok=True)

# Directory to save stage files and stage configs
STAGES_OUTPUT_DIR = os.path.join(DEST_DIR, "stages/")
os.makedirs(STAGES_OUTPUT_DIR, exist_ok=True)

# Directory to save object files and configs
OBJECTS_OUTPUT_DIR = os.path.join(DEST_DIR, "objects/")
os.makedirs(OBJECTS_OUTPUT_DIR, exist_ok=True)

SCENE_INSTANCE_OUTPUT_DIR = os.path.join(DEST_DIR, "scenes/")
os.makedirs(SCENE_INSTANCE_OUTPUT_DIR, exist_ok=True)

# Tags in scene glbs representing the stage and the sub-tree of objects
stage_tag = "Structure"
objects_tag = "Objects"


def deconstruct_scene_glb(
    src_scene_filename,
    scene_name_base,
    stage_dest_dir,
    object_dest_dir,
    stage_tag,
    objects_tag,
):

    try:
        scene_graph = trimesh.load(src_scene_filename)
    except BaseException:
        print(
            "Unable to load file {} - not recognized as valid mesh file. Aborting".format(
                src_scene_filename
            )
        )
        return {}
    stage_graph, stage_transform = gut.extract_obj_mesh_from_scenegraph(
        scene_graph, stage_tag
    )
    # stage_graph.show(viewer="gl")
    # print("Stage global transform \n{}".format(stage_transform))

    # base stage name - fully qualified directories + stub
    stage_name_base = scene_name_base + "_stage"
    stage_dest_filename_base = os.path.join(stage_dest_dir, stage_name_base)

    # export result to glb file
    stage_dest_filename = stage_dest_filename_base + ".glb"
    # save the extracted stage.glb
    stage_graph.export(stage_dest_filename)

    # set up stage configuration file
    stage_config_filename = (
        stage_dest_filename_base + ut.CONFIG_EXTENSIONS["stage"] + ".json"
    )
    # get relative path
    rel_stage_asset_filename = ut.transform_path_relative(
        stage_dest_filename, stage_dest_dir
    )
    # print("Relative stage path :{}".format(rel_stage_asset_filename))
    stage_config_json_dict = {"render_asset": rel_stage_asset_filename}
    # save config
    ut.mod_json_val_and_save(("", stage_config_filename, stage_config_json_dict))
    # stage component of scene instance config dict
    stage_instance_dict = gut.build_instance_config_json(
        stage_name_base, stage_transform
    )

    scene_instance_dict = {
        "stage_instance": stage_instance_dict,
        "object_instances": [],
    }

    objects = scene_graph.graph.transforms[objects_tag]
    for obj_name in objects:
        object_scene, obj_transform = gut.extract_obj_mesh_from_scenegraph(
            scene_graph, obj_name
        )
        # object_scene.show(viewer="gl")
        obj_name_base = obj_name + "_" + str(object_scene.md5())
        obj_dest_filename_base = os.path.join(object_dest_dir, obj_name_base)
        # build file names for output
        obj_dest_filename = obj_dest_filename_base + ".glb"
        obj_config_filename = (
            obj_dest_filename_base + ut.CONFIG_EXTENSIONS["object"] + ".json"
        )
        # print("Object dest filename : {}".format(obj_dest_filename))
        # save extracted object mesh
        object_scene.export(obj_dest_filename)
        rel_obj_dest_filename = ut.transform_path_relative(
            obj_dest_filename, object_dest_dir
        )
        obj_config_json_dict = {"render_asset": rel_obj_dest_filename}
        # save object config
        ut.mod_json_val_and_save(("", obj_config_filename, obj_config_json_dict))
        # build objectect instance config
        obj_instance_dict = gut.build_instance_config_json(obj_name_base, obj_transform)
        scene_instance_dict["object_instances"].append(obj_instance_dict)

    return scene_instance_dict


def main():
    # get listing of all scene glbs
    file_list = ut.get_files_matching_regex(SCENES_SRC_DIR)
    # dictionary keyed by scene of all scene instance dictionaries returned from deconstruct_scene_glb
    scene_instance_dicts = {}

    for path_file_tuple in file_list:
        scene_name = path_file_tuple[-1]
        scene_name_base = scene_name.split(".glb")[0]

        src_scene_filename = os.path.join(SCENES_SRC_DIR, scene_name)
        scene_instance_config_name = (
            scene_name_base + ut.CONFIG_EXTENSIONS["scene"] + ".json"
        )
        abs_scene_instance_filename = os.path.join(
            SCENE_INSTANCE_OUTPUT_DIR, scene_instance_config_name
        )
        print(
            "Scene {} Scene Instance name {}".format(
                scene_name, abs_scene_instance_filename
            )
        )

        scene_instance_dicts[scene_name_base] = deconstruct_scene_glb(
            src_scene_filename,
            scene_name_base,
            STAGES_OUTPUT_DIR,
            OBJECTS_OUTPUT_DIR,
            stage_tag,
            objects_tag,
        )
        # save scene instance configuration

        ut.mod_json_val_and_save(
            ("", abs_scene_instance_filename, scene_instance_dicts[scene_name_base])
        )


if __name__ == "__main__":
    main()
