#!/usr/bin/env python

# glb_mesh_decon : utlitities to deconstruct an aggregate glb scene
# mesh into constituent stage and object meshes and lighting configurations
# usable by habitat.

import os
from collections import OrderedDict, defaultdict
from typing import Dict, Optional, Set

import config_utils as ut
import glb_mesh_tools as gut

###
# JSON Configuration file for running this application.
MESH_DECON_CONFIG_JSON = "mesh_decon_AI2Thor.json"


####
# Dataset Source directories
DATASET_SRC_DIR = os.path.join(os.path.expanduser("~"), "Documents/AI2Thor/")
# Scene Source directory
SCENES_SRC_DIR = DATASET_SRC_DIR + "scenes/"

####
# Dataset specs
# Name/handle for scene dataset - used to name Scene Dataset config output
SCENE_DATASET_NAME = "AI2Thor"
# Tags in scene glbs representing the stage, object and lighting subtrees
STAGE_TAG = "Structure"
OBJECTS_TAG = "Objects"
LIGHTING_TAG = "Lighting"

# This is a dict of sets to be used to include nodes in the stage scene
# graph that are not directly linked to the stage scene_graph. The key of
# the dict is the name of the node that should be searched, and value is
# a set of lower-case substrings (for case-insensitive matching) of
# successor node names that should be included in the stage construction.
# The nodes whose names contain these substrings (And their subtrees) will
# be added to the stage and not spun off as individual objects.
STAGE_INCLUDE_OBJ_DICT = {
    OBJECTS_TAG: {
        "floor_",
        "window",
        "a-frameshelf",
        "blinds",
        "burner",
        "cabinet",
        "counter",
        "curtains",
        "faucet",
        "lightfixture",
        "lightswitch",
        "shelf",
        "shelfve",
        "showercurtain",
        "showerdoor",
        "showerglass",
        "showerhead",
        "sink",
        "stoveknob",
        "toilet_",
        "toiletpaperhanger",
        "towelholder",
    }
}

# Objects specified in the STAGE_INCLUDE_OBJ_DICT are, by default, ignored
# when objects are extracted from the scene graph.  This dict will provide
# lowercase substrs of object node names (for case insensitive matching)
# that should be explicitly included that might otherwise be ignored on
# object extraction.  These objects will also not be included in the stage.
OBJECT_OVERRIDE_SUBNODES = {"floor_lamp"}

# This is a set of substrings (case-insensitive) of node names for
# nodes that should be excluded from object synthesis
# Make OBJECT_EXCLUDE_SUBNODES an empty set to not ignore anything
OBJECT_EXCLUDE_SUBNODES = {
    "boundingbox",
    "colliders",
    "visibilitypoints",
    "hideandseek",
    "trigger",
}
STAGE_EXCLUDE_SUBNODES = {STAGE_TAG: OBJECT_EXCLUDE_SUBNODES}
STAGE_EXCLUDE_SUBNODES[OBJECTS_TAG] = OBJECT_OVERRIDE_SUBNODES

# This is a set of lowercase substrings of names of objects that
# should be specified as static in the scene instance upon creation
# when an otherwise dynamic scene is being instantiated.
# This should include large furnishings and other obstacles that we
# do not wish to simulate dynamically
OBJECTS_CREATED_STATIC = {
    "bed",
    "bench",
    "chair",
    "cloth",
    "decor",
    "decorative",
    "desk",
    "dresser",
    "fridge",
    "lamp",
    "microwave",
    "mirror",
    "nightstand",
    "ottoman",
    "painting",
    "poster",
    "safe",
    "sofa",
    "tvstand",
    "television",
    "table",
}


####
# Output directories for deconned objects
# Base destination directory
DEST_DIR = DATASET_SRC_DIR + "scene_dataset/"

###
# Scene instance
# Whether or not to build the scene instance configuration files
BUILD_SCENE_CONFIGS = True
# Directory to save synthesized scene instance configuration JSONs
SCENE_INSTANCE_OUTPUT_DIR = os.path.join(DEST_DIR, "scenes/")

###
# STAGE
# Whether or not to build the JSON config files for stages
BUILD_STAGE_CONFIGS = True
# Whether or not to save the stage GLB file
BUILD_STAGE_GLBS = True
# Directory to save stage GLB files and stage configs (if used)
STAGES_OUTPUT_DIR = os.path.join(DEST_DIR, "stages/")

###
# OBJECTS
# This value denotes that all object instances should be set to static in
# scene instance implementation.
OBJECTS_ALL_STATIC = False
# Whether or not to build the JSON config files for objects
BUILD_OBJECT_CONFIGS = True
# Whether or not to build the object GLB files
BUILD_OBJECT_GLBS = True
# Directory to save object files and configs
OBJECTS_OUTPUT_DIR = os.path.join(DEST_DIR, "objects/")

###
# Lighting
# Whether or not to build the lighting configurations
# Lighting requires pygltflib to be installed, so if not present
# will not succeed
BUILD_LIGHTING_CONFIGS = False
# Directory to save lighting configuration JSONs
LIGHTING_CONFIG_OUTPUT_DIR = os.path.join(DEST_DIR, "lighting/")


# Export source glb scenes as gltf scenes.  This
# requires pygltflib to be installed, so if not present,
# will not work.
SAVE_SRC_SCENES_AS_GLTF = False
# Where to save the synthesized gltf files
GLTF_EXPORT_DIR = DATASET_SRC_DIR + "tmpGLTFScenes/"

BUILD_SG_DIAGNOSTIC = True
# Directory to save scene graph json structure - for diagnostics
# Don't put in dataset directory since this isn't used by habitat
SG_OUTPUT_DIR = os.path.join(DATASET_SRC_DIR, "scene_graph_diagnostics/")

# These are values to use for default attributes in the Scene Dataset configuration
# file.
DEFAULT_ATTRIBUTES_TEMPLATE = {
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


def load_decon_global_config_values(mesh_decon_config_json):
    """This function will load values for the configuration globals used by this application to
    deconstruct the scene glbs into constituent stages and objects and synthesize the appropriate
    JSON configuration files for the results.
    """
    # Load configuration describing the scene meshes we wish to deconstruct into stages and objects
    decon_configs = ut.load_json_into_dict(mesh_decon_config_json)
    if len(decon_configs) == 0:
        return decon_configs
    # If nothing loaded, proceed through decon with already specified global values
    if len(decon_configs) == 0:
        return {}
    if "dataset_name" in decon_configs:
        global SCENE_DATASET_NAME
        SCENE_DATASET_NAME = decon_configs["dataset_name"]
    if "dataset_src_subdir" in decon_configs:
        global DATASET_SRC_DIR
        if (
            "dataset_rel_to_home" in decon_configs
            and decon_configs["dataset_rel_to_home"]
        ):
            home = os.path.expanduser("~")
        else:
            home = ""
        DATASET_SRC_DIR = os.path.join(home, "Documents/AI2Thor/")

    if "scenes_src_subdir" in decon_configs:
        scene_subdir = decon_configs["scenes_src_subdir"]
    else:
        scene_subdir = "scenes/"

    # Aggregate Scene GLBS source directory
    global SCENES_SRC_DIR
    SCENES_SRC_DIR = os.path.join(DATASET_SRC_DIR, scene_subdir)

    # Load the tags referencing the node names in the scene graph corresponding to the branches containing
    # stage, object, and lighting information
    if "stage_tag" in decon_configs:
        global STAGE_TAG
        STAGE_TAG = decon_configs["stage_tag"].strip()
    if "objects_tag" in decon_configs:
        global OBJECTS_TAG
        OBJECTS_TAG = decon_configs["objects_tag"].strip()
    if "lighting_tag" in decon_configs:
        global LIGHTING_TAG
        LIGHTING_TAG = decon_configs["lighting_tag"].strip()

    # This references nodes from the objects subtree that should be merged into the
    # stage and not treated as individual objects
    if "stage_include_obj_substr" in decon_configs:
        global STAGE_INCLUDE_OBJ_DICT
        # remove any default values
        STAGE_INCLUDE_OBJ_DICT.clear()
        # set these values specific for objects subbranch
        STAGE_INCLUDE_OBJ_DICT[OBJECTS_TAG] = decon_configs["stage_include_obj_substr"]

    global STAGE_EXCLUDE_SUBNODES
    # specify the substring names of objects that we wish to retain as
    # objects but may otherwise be swept up into the stage from STAGE_INCLUDE_OBJ_DICT.
    if "obj_override_names" in decon_configs:
        global OBJECT_OVERRIDE_SUBNODES
        OBJECT_OVERRIDE_SUBNODES = decon_configs["obj_override_names"]
        STAGE_EXCLUDE_SUBNODES[OBJECTS_TAG] = OBJECT_OVERRIDE_SUBNODES

    # Substrings that denote nodes in the objects subtree that we do not want to include
    # in the resultant objects we create, either objects or components of objects
    if "obj_exclude_names" in decon_configs:
        global OBJECT_EXCLUDE_SUBNODES
        OBJECT_EXCLUDE_SUBNODES = decon_configs["obj_exclude_names"]
        STAGE_EXCLUDE_SUBNODES = {STAGE_TAG: OBJECT_EXCLUDE_SUBNODES}

    # Substrings denoting the object nodes that should be instanced as static within scene
    # instance config
    if "obj_created_static" in decon_configs:
        global OBJECTS_CREATED_STATIC
        OBJECTS_CREATED_STATIC = decon_configs["obj_created_static"]

    # Default attribute values for the loaded dataset.  These will be used to intialize all
    # configs as they are created, before any stage or object-specific config values are read in
    # from disk
    if "default_attributes" in decon_configs:
        dflt_vals_dict = decon_configs["default_attributes"]
        global DEFAULT_ATTRIBUTES_TEMPLATE
        # all sub-values may not be present
        for k, v in dflt_vals_dict.items():
            DEFAULT_ATTRIBUTES_TEMPLATE[k] = v

    return decon_configs


def build_required_directories():
    """This function will re-make, and create if appropriate, the destination directories
    used by the application to save the various results.
    """
    ####
    # Output directories for deconned objects
    # Base destination directory
    global DEST_DIR
    DEST_DIR = DATASET_SRC_DIR + "scene_dataset/"
    os.makedirs(DEST_DIR, exist_ok=True)

    global STAGES_OUTPUT_DIR
    # Directory to save stage GLB files and stage configs (if used)
    STAGES_OUTPUT_DIR = os.path.join(DEST_DIR, "stages/")
    if BUILD_STAGE_CONFIGS or BUILD_STAGE_GLBS:
        os.makedirs(STAGES_OUTPUT_DIR, exist_ok=True)

    global OBJECTS_OUTPUT_DIR
    # Directory to save object files and configs
    OBJECTS_OUTPUT_DIR = os.path.join(DEST_DIR, "objects/")
    if BUILD_OBJECT_CONFIGS or BUILD_OBJECT_GLBS:
        os.makedirs(OBJECTS_OUTPUT_DIR, exist_ok=True)

    # Directory to save lighting configuration JSONs
    global LIGHTING_CONFIG_OUTPUT_DIR
    LIGHTING_CONFIG_OUTPUT_DIR = os.path.join(DEST_DIR, "lighting/")
    if BUILD_LIGHTING_CONFIGS:
        os.makedirs(LIGHTING_CONFIG_OUTPUT_DIR, exist_ok=True)

    # Directory to save synthesized scene instance configuration JSONs
    global SCENE_INSTANCE_OUTPUT_DIR
    SCENE_INSTANCE_OUTPUT_DIR = os.path.join(DEST_DIR, "scenes/")
    if BUILD_SCENE_CONFIGS:
        os.makedirs(SCENE_INSTANCE_OUTPUT_DIR, exist_ok=True)

    # Export glb scenes as gltf scenes
    global GLTF_EXPORT_DIR
    GLTF_EXPORT_DIR = DATASET_SRC_DIR + "tmpGLTFScenes/"
    if SAVE_SRC_SCENES_AS_GLTF:
        os.makedirs(GLTF_EXPORT_DIR, exist_ok=True)

    # Directory to save scene graph json structure - for diagnostics
    # Don't put in dataset directory since this isn't used by habitat
    global SG_OUTPUT_DIR
    SG_OUTPUT_DIR = os.path.join(DATASET_SRC_DIR, "scene_graph_diagnostics/")
    if BUILD_SG_DIAGNOSTIC:
        os.makedirs(SG_OUTPUT_DIR, exist_ok=True)


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
        scene_graph, stage_tag, "world", include_obj_dict, STAGE_EXCLUDE_SUBNODES, True
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
    objects_raw = scene_graph.graph.transforms.children_dict[objects_tag]
    object_instance_configs = []
    objects = []

    # exclude object names that have been added to stage already
    # get set of object names that are ref'ed by objects tag
    excl_str_set = exclude_obj_dict[objects_tag]
    for obj_name in objects_raw:
        obj_is_valid = True
        for substr in excl_str_set:
            if substr.lower() in obj_name.lower():
                obj_is_valid = False
                break
        # check if invalid object should be overridden
        if not obj_is_valid:
            for substr in OBJECT_OVERRIDE_SUBNODES:
                if substr.lower() in obj_name.lower():
                    obj_is_valid = True
                    break
        if obj_is_valid:
            objects.append(obj_name)
    #
    for obj_name in objects:
        obj_exclude_dict = {}  # dict(exclude_obj_dict)
        # exclude elements in objects that do not correspond to geometry
        # obj_exclude_dict[obj_name] = OBJECT_EXCLUDE_SUBNODES
        # extract the object "scene" and its global transform
        # (scene is the mesh + other assets to save for object glb)
        object_scene, obj_transform = gut.extract_obj_mesh_from_scenegraph(
            scene_graph, obj_name, objects_tag, {}, obj_exclude_dict, False
        )
        if object_scene is None:
            continue

        # TODO  isolate and extract collision mesh if present?
        obj_name_base = obj_name
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
        # build object instance info to be used in scene instance config to place object
        obj_instance_dict = gut.build_instance_config_json(obj_name_base, obj_transform)
        object_instance_configs.append(obj_instance_dict)

    return object_instance_configs


def extract_lighting_from_scene(
    scene_filename_glb,
    scene_graph,
    scene_name_base: str,
    lights_dest_dir: str,
    lights_tag: str,
    debug: Optional[bool] = False,
):

    lighting_res_dict = gut.extract_lighting_from_gltf(scene_filename_glb, lights_tag)
    if len(lighting_res_dict) == 0:
        return ""

    if debug:
        print("Lighting res :")
        for k, v in lighting_res_dict.items():
            if isinstance(v, list):
                i = 0
                for elem in v:
                    print("\t{}[{}] : {}".format(k, i, elem))
                    i += 1
            else:
                print("\t{} : {}".format(k, v))

    # base stage name - fully qualified directories + stub
    lighting_name_base = scene_name_base + "_lighting"
    lighting_dest_filename_base = os.path.join(lights_dest_dir, lighting_name_base)
    # set up lighting configuration file
    lighting_config_filename = (
        lighting_dest_filename_base + ut.CONFIG_EXTENSIONS["lighting"] + ".json"
    )
    # temp config save
    ut.mod_json_val_and_save(("", lighting_config_filename, lighting_res_dict))
    # save lighting_res_dict
    # TODO not currently saving JSON properly
    # ut.mod_json_val_and_save(("", lighting_config_filename, lighting_config_json_dict), dry_run=True)

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


def test_objects():
    bad_object_filename = os.path.join(DATASET_SRC_DIR, "test/Alarm_Clock_1_bad.glb")
    good_object_filename = os.path.join(DATASET_SRC_DIR, "test/Alarm_Clock_1_good.glb")
    bad_object_out_filename = os.path.join(
        DATASET_SRC_DIR, "test/Alarm_Clock_1_bad.gltf"
    )
    good_object_out_filename = os.path.join(
        DATASET_SRC_DIR, "test/Alarm_Clock_1_good.gltf"
    )
    gut.convert_glb_to_gltf(bad_object_filename, bad_object_out_filename, True)
    gut.convert_glb_to_gltf(good_object_filename, good_object_out_filename, True)


def main():

    # get listing of all scene glbs
    file_list = ut.get_files_matching_regex(SCENES_SRC_DIR)

    build_scene_dataset_config(SCENE_DATASET_NAME, DEFAULT_ATTRIBUTES_TEMPLATE)

    ###for testing - 17_physics has stove w/burner and knobs
    file_list = [
        ("", "", "FloorPlan320_physics.glb"),
        ("", "", "FloorPlan_Train8_3.glb"),
        ("", "", "FloorPlan430_physics.glb"),
        ("", "", "FloorPlan17_physics.glb"),
        ("", "", "FloorPlan_Val2_4.glb"),
    ]

    # count the instances of objects by name
    object_instance_count_dict = defaultdict(int)
    # file name to save object counts
    object_instance_count_filename = os.path.join(
        DEST_DIR, SCENE_DATASET_NAME + "_object_counts.json"
    )

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
        print(
            "Scene {} Scene Instance name {}".format(
                scene_name, abs_scene_instance_filename
            )
        )

        # Save individual stage and objects extracted from scene and
        # construct scene instance json based on layout
        scene_graph = gut.load_glb_as_scene(src_scene_filename)
        if scene_graph is None:
            continue

        if SAVE_SRC_SCENES_AS_GLTF:
            gltf_scene_name = os.path.join(GLTF_EXPORT_DIR, scene_name_base + ".gltf")
            convert_success = gut.convert_glb_to_gltf(
                src_scene_filename, gltf_scene_name, True
            )
            print(
                "\t{} {}successfully converted.".format(
                    src_scene_filename, ("" if convert_success else "not ")
                )
            )

        # If requested build diagnostic info about scene graph and save
        if BUILD_SG_DIAGNOSTIC:
            build_scene_graph_diagnostic(scene_graph, scene_name_base)

        lighting_setup_config_name = ""
        if BUILD_LIGHTING_CONFIGS:
            # extract all lighting configs in the scene
            lighting_setup_config_name = extract_lighting_from_scene(
                src_scene_filename,
                scene_graph,
                scene_name_base,
                LIGHTING_CONFIG_OUTPUT_DIR,
                LIGHTING_TAG,
            )

        # extract the stage from the scene glb
        stage_instance_config = extract_stage_from_scene(
            scene_graph,
            scene_name_base,
            STAGES_OUTPUT_DIR,
            STAGE_TAG,
            STAGE_INCLUDE_OBJ_DICT,
            BUILD_STAGE_CONFIGS,
        )

        # extract all the object instances within the scene
        obj_instance_config_list = extract_objects_from_scene(
            scene_graph,
            scene_name_base,
            OBJECTS_OUTPUT_DIR,
            OBJECTS_TAG,
            STAGE_INCLUDE_OBJ_DICT,
            BUILD_OBJECT_CONFIGS,
        )
        # get counts of object instances
        for elem in obj_instance_config_list:
            object_instance_count_dict[elem["template_name"]] += 1

        # compose the scene instance configuration JSON
        scene_instance_dict = {
            "stage_instance": stage_instance_config,
            "object_instances": obj_instance_config_list,
            "default_lighting": lighting_setup_config_name,
        }

        # save scene instance configuration
        ut.mod_json_val_and_save(("", abs_scene_instance_filename, scene_instance_dict))

        # resave updated object counts for every scene, in case some scene fails parsing
        # order results in result json
        object_instance_ordered_dict = OrderedDict(
            sorted(object_instance_count_dict.items(), key=lambda x: x[0])
        )
        # save all object config counts
        ut.mod_json_val_and_save(
            ("", object_instance_count_filename, object_instance_ordered_dict)
        )


if __name__ == "__main__":
    # Load configuration describing the scene meshes we wish to deconstruct into stages and objects
    load_decon_global_config_values(MESH_DECON_CONFIG_JSON)
    # whether a successful load occurred or not, we need to make the destination directories
    build_required_directories()

    main()
