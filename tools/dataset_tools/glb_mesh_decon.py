#!/usr/bin/env python

# glb_mesh_decon : utlitities to deconstruct an aggregate glb scene
# mesh into constituent stage and object meshes and lighting configurations
# usable by habitat.

import os
from typing import Dict, Set

import config_utils as ut
import glb_mesh_tools as gut

####
# Source directories
HOME = os.path.expanduser("~")
DATASET_SRC_DIR = os.path.join(HOME, "Documents/AI2Thor/")
# Scene Source directory
SCENES_SRC_DIR = DATASET_SRC_DIR + "scenes/"

GLTF_EXPORT_DIR = DATASET_SRC_DIR + "tmpGLTFScenes/"
os.makedirs(GLTF_EXPORT_DIR, exist_ok=True)


####
# Output directories for deconned objects
# Base destination directory
DEST_DIR = DATASET_SRC_DIR + "scene_dataset/"
os.makedirs(DEST_DIR, exist_ok=True)
# Whether or not to build the JSON config files for stages
BUILD_STAGE_CONFIGS = True
# Directory to save stage files and stage configs (if used)
STAGES_OUTPUT_DIR = os.path.join(DEST_DIR, "stages/")
os.makedirs(STAGES_OUTPUT_DIR, exist_ok=True)

# Whether or not to build the JSON config files for objects
BUILD_OBJECT_CONFIGS = True
# Directory to save object files and configs
OBJECTS_OUTPUT_DIR = os.path.join(DEST_DIR, "objects/")
os.makedirs(OBJECTS_OUTPUT_DIR, exist_ok=True)

# Directory to save lighting configuration JSONs
LIGHTING_INSTANCE_OUTPUT_DIR = os.path.join(DEST_DIR, "lighting/")
os.makedirs(LIGHTING_INSTANCE_OUTPUT_DIR, exist_ok=True)

# Directory to save synthesized scene instance configuration JSONs
SCENE_INSTANCE_OUTPUT_DIR = os.path.join(DEST_DIR, "scenes/")
os.makedirs(SCENE_INSTANCE_OUTPUT_DIR, exist_ok=True)


# wether or not to save the scene graph structure as a json file
BUILD_SG_DIAGNOSTIC = True
# Directory to save scene graph json structure - for diagnostics
# Don't put in dataset directory since this isn't used by habitat
SG_OUTPUT_DIR = os.path.join(DATASET_SRC_DIR, "scene_graph_diagnostics/")
if BUILD_SG_DIAGNOSTIC:
    os.makedirs(SG_OUTPUT_DIR, exist_ok=True)

# Tags in scene glbs representing the stage, object and lighting subtrees
stage_tag = "Structure"
objects_tag = "Objects"
lighting_tag = "Lighting"

# This is a dict of sets to be used to include nodes in the stage scene
# graph that are not directly linked to the stage scene_graph. The key of
# the dict is the name of the node that should be searched, and value is
# a set of substrings (case-insensitive) of successor node names that
# should be included in the stage construction. The nodes whose names
# contain these substrings (And their subtrees) will be added to the
# stage and not spun off as individual objects.
STAGE_INCLUDE_OBJ_DICT = {objects_tag: {"floor_", "window"}}

# This is a set of substrings (case-insensitive) of node names for
# nodes that should be excluded from object synthesis
# Make OBJECT_EXCLUDE_SUBNODES an empty set to not ignore anything
OBJECT_EXCLUDE_SUBNODES = {"boundingbox", "colliders", "visibilitypoints"}
STAGE_EXCLUDE_SUBNODES = {stage_tag: OBJECT_EXCLUDE_SUBNODES}


def extract_stage_from_scene(
    scene_graph,
    scene_name_base: str,
    stage_dest_dir: str,
    stage_tag: str,
    include_obj_dict: Dict[str, Set[str]],
    build_configs: bool,
):
    # Extract the stage mesh and its transform in the world
    stage_graph, stage_transform = gut.extract_obj_mesh_from_scenegraph(
        scene_graph, stage_tag, "world", include_obj_dict, STAGE_EXCLUDE_SUBNODES
    )
    # display stage
    # stage_graph.show(viewer="gl")
    # print("Stage global transform \n{}".format(stage_transform))

    # base stage name - fully qualified directories + stub
    stage_name_base = scene_name_base + "_stage"
    stage_dest_filename_base = os.path.join(stage_dest_dir, stage_name_base)

    # export stage mesh result to glb file
    stage_dest_filename = stage_dest_filename_base + ".glb"
    stage_graph.export(stage_dest_filename)

    if build_configs:
        # get relative path
        rel_stage_asset_filename = ut.transform_path_relative(
            stage_dest_filename, stage_dest_dir
        )
        # set up stage configuration file
        stage_config_filename = (
            stage_dest_filename_base + ut.CONFIG_EXTENSIONS["stage"] + ".json"
        )
        # print("Relative stage path :{}".format(rel_stage_asset_filename))
        stage_config_json_dict = {
            "render_asset": rel_stage_asset_filename,
            "collision_asset": rel_stage_asset_filename,
        }
        # save config
        ut.mod_json_val_and_save(("", stage_config_filename, stage_config_json_dict))
    # stage component of scene instance config dict
    stage_instance_dict = gut.build_instance_config_json(
        stage_name_base, stage_transform
    )

    return stage_instance_dict


def extract_objects_from_scene(
    scene_graph,
    scene_name_base: str,
    object_dest_dir: str,
    objects_tag: str,
    exclude_obj_dict,
    build_configs: bool,
):
    objects_raw = scene_graph.graph.transforms[objects_tag]
    object_instance_configs = []
    objects = []

    # exclude object names that have been added to stage already
    # get set of object names that are ref'ed by objects tag
    str_set = exclude_obj_dict[objects_tag]
    for obj_name in objects_raw:
        obj_is_valid = True
        for substr in str_set:
            if substr.lower() in obj_name.lower():
                obj_is_valid = False
                break
        if obj_is_valid:
            objects.append(obj_name)
    #
    for obj_name in objects:
        obj_exclude_dict = dict(exclude_obj_dict)
        # exclude elements in objects that do not correspond to geometry
        obj_exclude_dict[obj_name] = OBJECT_EXCLUDE_SUBNODES

        object_scene, obj_transform = gut.extract_obj_mesh_from_scenegraph(
            scene_graph, obj_name, objects_tag, {}, obj_exclude_dict
        )
        if object_scene is None:
            continue
        # object_scene.show(viewer="gl")
        obj_name_base = obj_name  # + "_" + str(object_scene.md5())
        obj_dest_filename_base = os.path.join(object_dest_dir, obj_name_base)
        # build file names for output
        obj_dest_filename = obj_dest_filename_base + ".glb"
        # print("Object dest filename : {}".format(obj_dest_filename))
        # save extracted object mesh
        object_scene.export(obj_dest_filename)

        if build_configs:
            rel_obj_dest_filename = ut.transform_path_relative(
                obj_dest_filename, object_dest_dir
            )
            obj_config_json_dict = {
                "render_asset": rel_obj_dest_filename,
                "collision_asset": rel_obj_dest_filename,
            }

            obj_config_filename = (
                obj_dest_filename_base + ut.CONFIG_EXTENSIONS["object"] + ".json"
            )
            # save object config
            ut.mod_json_val_and_save(("", obj_config_filename, obj_config_json_dict))
        # build objectect instance config
        obj_instance_dict = gut.build_instance_config_json(obj_name_base, obj_transform)
        object_instance_configs.append(obj_instance_dict)

    return object_instance_configs


def extract_lighting_from_scene(
    scene_filename_glb,
    scene_graph,
    scene_name_base: str,
    lights_dest_dir: str,
    lights_tag: str,
):

    # Get lighting hierarchy from scene_graph
    # build lighting config for scene_graph
    lighting_config_json_dict = gut.extract_ligthing_from_scenegraph(
        scene_graph, lights_tag, "world"
    )

    # Get json
    base_json = gut.extract_json_from_glb(scene_filename_glb)
    nodes_dict = base_json["nodes"]
    # list of lights - accessed in scene_graph by idx
    light_list = base_json["extensions"]["KHR_lights_punctual"]["lights"]
    print("Light list : {}".format(light_list))
    # dict of nodes referencing lights, keyed by original nodes idx
    light_nodes_list = [
        n
        for n in nodes_dict
        if "extensions" in n and "KHR_lights_punctual" in n["extensions"]
    ]
    # find nodes that represent light instances
    # new_lights_list = []
    for n in light_nodes_list:
        lights_idx = n["extensions"]["KHR_lights_punctual"]["light"]
        print(
            "idx {} name : {} : {}".format(
                lights_idx, n["name"], light_list[lights_idx]
            )
        )

    # base stage name - fully qualified directories + stub
    lighting_name_base = scene_name_base + "_lighting"
    lighting_dest_filename_base = os.path.join(lights_dest_dir, lighting_name_base)
    # set up lighting configuration file
    lighting_config_filename = (
        lighting_dest_filename_base + ut.CONFIG_EXTENSIONS["lighting"] + ".json"
    )
    # temp config save
    # ut.mod_json_val_and_save(("", lighting_config_filename, base_json))
    # save config
    # TODO not currently saving JSON properly
    ut.mod_json_val_and_save(("", lighting_config_filename, lighting_config_json_dict))

    # return the string name of the lighting config
    # TODO change from empty string
    return ""  # lighting_name_base


def build_scene_dataset_config(scene_dataset_filename, default_attrs):
    scene_dataset_config = {
        "stages": {"paths": {".json": ["stages/*"]}},
        "objects": {"paths": {".json": ["objects/*"]}},
        "light_setups": {"paths": {".json": ["lighting/*"]}},
        "scene_instances": {"paths": {".json": ["scenes/*"]}},
    }
    # add default attributes specs if any exist
    for k, v in default_attrs.items():
        if len(v) > 0:
            scene_dataset_config[k]["default_attributes"] = v
    # Save scene dataset config
    ut.mod_json_val_and_save(
        (
            "",
            os.path.join(
                DEST_DIR,
                scene_dataset_filename + ut.CONFIG_EXTENSIONS["dataset"] + ".json",
            ),
            scene_dataset_config,
        )
    )


# build dict rep of scene graph and save to json
def build_scene_graph_diagnostic(scene_graph, scene_name_base):
    # get node graph dictionary
    sg_node_dict = gut.build_glb_scene_graph_dict(scene_graph, "world")
    # print("{}".format(sg_node_dict))
    # build file name
    abs_sg_diagnostic_filename = os.path.join(
        SG_OUTPUT_DIR, scene_name_base + "_sg_layout.json"
    )
    # save node graph dictionary as json
    ut.mod_json_val_and_save(("", abs_sg_diagnostic_filename, sg_node_dict))


def main():
    # get listing of all scene glbs
    file_list = ut.get_files_matching_regex(SCENES_SRC_DIR)
    # build the scene dataset configuration
    default_attributes = {
        "stages": {
            "requires_lighting": "true",
            "up": [0, 1, 0],
            "front": [0, 0, -1],
            "origin": [0, 0, 0],
        },
        "objects": {
            "requires_lighting": "true",
            "up": [0, 1, 0],
            "front": [0, 0, -1],
            "origin": [0, 0, 0],
        },
        "scene_instances": {
            "translation_origin": "asset_local",
            "default_lighting": "",
        },
        "light_setups": {},
    }
    build_scene_dataset_config("AI2Thor", default_attributes)

    # for testing
    file_list = [
        ("", "", "FloorPlan_Train1_1.glb"),
        ("", "", "FloorPlan430_physics.glb"),
        ("", "", "FloorPlan_Val2_4.glb"),
    ]

    # Go through every scene glb file
    for path_file_tuple in file_list:
        scene_name = path_file_tuple[-1]
        # test this scene
        # scene_name = "FloorPlan320_physics.glb"
        scene_name_base = scene_name.split(".glb")[0]

        src_scene_filename = os.path.join(SCENES_SRC_DIR, scene_name)
        # build scene instance config file
        abs_scene_instance_filename = os.path.join(
            SCENE_INSTANCE_OUTPUT_DIR,
            scene_name_base + ut.CONFIG_EXTENSIONS["scene"] + ".json",
        )

        # Save individual stage and objects extracted from scene and
        # construct scene instance json based on layout
        scene_graph = gut.load_glb_as_scene(src_scene_filename)
        if scene_graph is None:
            continue

        # gltf_scene_name = os.path.join(GLTF_EXPORT_DIR, scene_name_base + ".gltf")
        # convert_success = gut.convert_glb_to_gltf(
        #     src_scene_filename, gltf_scene_name, True
        # )
        # print("Conversion worked! :{}".format(convert_success))

        print(
            "Scene {} Scene Instance name {}".format(
                scene_name, abs_scene_instance_filename
            )
        )

        if BUILD_SG_DIAGNOSTIC:
            build_scene_graph_diagnostic(scene_graph, scene_name_base)

        # extract all lighting configs in the scene
        lighting_setup_config_name = extract_lighting_from_scene(
            src_scene_filename,
            scene_graph,
            scene_name_base,
            LIGHTING_INSTANCE_OUTPUT_DIR,
            lighting_tag,
        )
        continue
        # extract the stage from the scene glb
        stage_instance_config = extract_stage_from_scene(
            scene_graph,
            scene_name_base,
            STAGES_OUTPUT_DIR,
            stage_tag,
            STAGE_INCLUDE_OBJ_DICT,
            BUILD_STAGE_CONFIGS,
        )

        # extract all the object instances within the scene
        obj_instance_config_list = extract_objects_from_scene(
            scene_graph,
            scene_name_base,
            OBJECTS_OUTPUT_DIR,
            objects_tag,
            STAGE_INCLUDE_OBJ_DICT,
            BUILD_OBJECT_CONFIGS,
        )

        # compose the scene instance configuration JSON
        scene_instance_dict = {
            "stage_instance": stage_instance_config,
            "object_instances": obj_instance_config_list,
            "default_lighting": lighting_setup_config_name,
        }

        # save scene instance configuration
        ut.mod_json_val_and_save(("", abs_scene_instance_filename, scene_instance_dict))


if __name__ == "__main__":
    main()
