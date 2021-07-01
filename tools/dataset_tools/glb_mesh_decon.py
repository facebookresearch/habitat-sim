#!/usr/bin/env python

# glb_mesh_decon : utlitities to deconstruct an aggregate glb scene
# mesh into constituent stage and object meshes and lighting configurations
# usable by habitat.

import os
from collections import OrderedDict, defaultdict
from typing import Any, Dict, Optional, Set, Tuple

import config_utils as ut
import glb_mesh_tools as gut

###
# JSON Configuration file for running this application.
# MESH_DECON_CONFIG_JSON_FILENAME = "mesh_decon_AI2Thor.json"
MESH_DECON_CONFIG_JSON_FILENAME = "mesh_decon_ReplicaCAD.json"
# MESH_DECON_CONFIG_JSON_FILENAME = "mesh_decon_ReplicaCAD_baked.json"


####
# Dataset Source directories
DATASET_SRC_DIR = os.path.join(os.path.expanduser("~"), "Documents/AI2Thor/")
# Scene Source directory
SCENES_SRC_DIR = DATASET_SRC_DIR + "scenes/"
# Source of existing objects, to be used for obj instance name matching
# in scene instance config.  Only used if we wish to match instance names
# to existing object names
OBJECTS_SRC_DIR = DATASET_SRC_DIR + "objects/"

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
# This is a set of lowercase substrings of names of objects that
# should be specified as dynamic in the scene instance upon creation
# when a dynamic scene is being instantiated.  This is intended to provide
# an easy override for when all objects are specified to be static
OBJECTS_CREATED_DYNAMIC = {}


####
# Output directories for deconned objects
# Base destination directories for glbs and configs, relative to DATASET_SRC_DIR
DEST_SUBDIR = "scene_dataset/"
DEST_GLB_SUBDIR = "."
DEST_CONFIG_SUBDIR = "scene_dataset/"

DEST_CONFIG_DIR = os.path.join(DATASET_SRC_DIR, DEST_SUBDIR, DEST_CONFIG_SUBDIR)

DEST_GLB_DIR = os.path.join(DATASET_SRC_DIR, DEST_SUBDIR, DEST_GLB_SUBDIR)


###
# Scene instance
# Whether or not to build the scene instance configuration files
BUILD_SCENE_CONFIGS = True
SCENE_INSTANCE_OUTPUT_SUBDIR = "scenes/"

# Directory to save synthesized scene instance configuration JSONs
SCENE_INSTANCE_OUTPUT_DIR = os.path.join(DEST_CONFIG_DIR, SCENE_INSTANCE_OUTPUT_SUBDIR)

###
# STAGE
# Whether or not to build the JSON config files for stages
BUILD_STAGE_CONFIGS = True
STAGE_CONFIG_OUTPUT_SUBDIR = "stages/"

# Whether or not to save the stage GLB file
BUILD_STAGE_GLBS = True
STAGE_GLB_OUTPUT_SUBDIR = "stages/"

# Directory to save stage GLB files and stage configs (if used)
STAGE_GLB_OUTPUT_DIR = os.path.join(DEST_GLB_DIR, STAGE_GLB_OUTPUT_SUBDIR)
# Directory to save stage GLB files and stage configs (if used)
STAGE_CONFIG_OUTPUT_DIR = os.path.join(DEST_CONFIG_DIR, STAGE_CONFIG_OUTPUT_SUBDIR)

###
# OBJECTS
# This value denotes that all object instances should be set to static in
# scene instance implementation.
OBJECTS_ALL_STATIC = False
# Whether or not to build the JSON config files for objects
BUILD_OBJECT_CONFIGS = True
OBJECT_CONFIG_OUTPUT_SUBDIR = "objects/"
# Whether or not to build the object GLB files
BUILD_OBJECT_GLBS = True
OBJECT_GLB_OUTPUT_SUBDIR = "objects/"

# Directory to save object files
OBJ_GLB_OUTPUT_DIR = os.path.join(DEST_GLB_DIR, OBJECT_GLB_OUTPUT_SUBDIR)

# Directory to save object configs
OBJ_CONFIG_OUTPUT_DIR = os.path.join(DEST_CONFIG_DIR, OBJECT_CONFIG_OUTPUT_SUBDIR)

###
# Lighting
# Whether or not to build the lighting configurations
# Lighting requires pygltflib to be installed, so if not present
# will not succeed
BUILD_LIGHTING_CONFIGS = False
LIGHT_CONFIG_OUTPUT_SUBDIR = "lighting/"
# Directory to save lighting configuration JSONs
LIGHTING_CONFIG_OUTPUT_DIR = os.path.join(DEST_CONFIG_DIR, LIGHT_CONFIG_OUTPUT_SUBDIR)


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


def load_decon_global_config_values(decon_config_json: str):
    """This function will load values for the configuration globals used by this application to
    deconstruct the scene glbs into constituent stages and objects and synthesize the appropriate
    JSON configuration files for the results.
    """
    # Load configuration describing the scene meshes we wish to deconstruct into stages and objects
    decon_configs = ut.load_json_into_dict(decon_config_json)
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
        if "dataset_rel_to_home" in decon_configs:
            home = os.path.expanduser("~")
        else:
            home = ""
        DATASET_SRC_DIR = os.path.join(home, decon_configs["dataset_src_subdir"])

    if "scenes_src_subdir" in decon_configs:
        scene_subdir = decon_configs["scenes_src_subdir"]
    else:
        scene_subdir = "scenes/"

    # Aggregate Scene GLBS source directory
    global SCENES_SRC_DIR
    SCENES_SRC_DIR = os.path.join(DATASET_SRC_DIR, scene_subdir)

    if "objects_src_subdir" in decon_configs:
        obj_src_subdir = decon_configs["objects_src_subdir"]
    else:
        obj_src_subdir = "objects/"
    global OBJECTS_SRC_DIR
    OBJECTS_SRC_DIR = os.path.join(DATASET_SRC_DIR, obj_src_subdir)

    if "dataset_dest_subdir" in decon_configs:
        global DEST_SUBDIR
        DEST_SUBDIR = decon_configs["dataset_dest_subdir"].strip()

    # build various glb and config output directories
    if "dataset_glb_dest_subdir" in decon_configs:
        global DEST_GLB_SUBDIR
        DEST_GLB_SUBDIR = decon_configs["dataset_glb_dest_subdir"].strip()

    if "stage_glb_dest_subdir" in decon_configs:
        global STAGE_GLB_OUTPUT_SUBDIR
        STAGE_GLB_OUTPUT_SUBDIR = decon_configs["stage_glb_dest_subdir"].strip()

    if "obj_glb_dest_subdir" in decon_configs:
        global OBJECT_GLB_OUTPUT_SUBDIR
        OBJECT_GLB_OUTPUT_SUBDIR = decon_configs["obj_glb_dest_subdir"].strip()

    if "dataset_config_dest_subdir" in decon_configs:
        global DEST_CONFIG_SUBDIR
        DEST_CONFIG_SUBDIR = decon_configs["dataset_config_dest_subdir"].strip()

    if "scene_instance_dest_subdir" in decon_configs:
        global SCENE_INSTANCE_OUTPUT_SUBDIR
        SCENE_INSTANCE_OUTPUT_SUBDIR = decon_configs[
            "scene_instance_dest_subdir"
        ].strip()

    if "stage_config_dest_subdir" in decon_configs:
        global STAGE_CONFIG_OUTPUT_SUBDIR
        STAGE_CONFIG_OUTPUT_SUBDIR = decon_configs["stage_config_dest_subdir"].strip()

    if "obj_config_dest_subdir" in decon_configs:
        global OBJECT_CONFIG_OUTPUT_SUBDIR
        OBJECT_CONFIG_OUTPUT_SUBDIR = decon_configs["obj_config_dest_subdir"].strip()

    if "light_config_dest_subdir" in decon_configs:
        global LIGHT_CONFIG_OUTPUT_SUBDIR
        LIGHT_CONFIG_OUTPUT_SUBDIR = decon_configs["light_config_dest_subdir"].strip()

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

    # Load whether all objects should be static
    if "objects_all_static" in decon_configs:
        global OBJECTS_ALL_STATIC
        OBJECTS_ALL_STATIC = decon_configs["objects_all_static"]

    # if not specified in config file, set to be true
    if "save_object_instances" not in decon_configs:
        decon_configs["save_object_instances"] = True

        # if not specified in config file, set to be false
    if "save_articulated_object_instances" not in decon_configs:
        decon_configs["save_articulated_object_instances"] = False

    # if not specified in config, set default value to false
    if "match_object_names" not in decon_configs:
        decon_configs["match_object_names"] = False

    if "stage_instance_file_tag" not in decon_configs:
        decon_configs["stage_instance_file_tag"] = {}

    if "default_lighting_tag" not in decon_configs:
        decon_configs["default_lighting_tag"] = ""

    if "apply_object_transform" not in decon_configs:
        decon_configs["apply_object_transform"] = False

    # if no mappings present, make empty map of object node names to AO files
    if "ao_include_obj_names" not in decon_configs:
        decon_configs["ao_include_obj_names"] = {}

    # Load the settings for what configs and/or glbs to construct and save
    if "build_scene_configs" in decon_configs:
        global BUILD_SCENE_CONFIGS
        BUILD_SCENE_CONFIGS = decon_configs["build_scene_configs"]
    if "build_stage_configs" in decon_configs:
        global BUILD_STAGE_CONFIGS
        BUILD_STAGE_CONFIGS = decon_configs["build_stage_configs"]
    if "build_object_configs" in decon_configs:
        global BUILD_OBJECT_CONFIGS
        BUILD_OBJECT_CONFIGS = decon_configs["build_object_configs"]
    if "build_lighting_configs" in decon_configs:
        global BUILD_LIGHTING_CONFIGS
        BUILD_LIGHTING_CONFIGS = decon_configs["build_lighting_configs"]

    if "build_stage_glbs" in decon_configs:
        global BUILD_STAGE_GLBS
        BUILD_STAGE_GLBS = decon_configs["build_stage_glbs"]
    if "build_object_glbs" in decon_configs:
        global BUILD_OBJECT_GLBS
        BUILD_OBJECT_GLBS = decon_configs["build_object_glbs"]

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

    # if we specify all objects to be static, this will override it using the same process as "obj_created_static"
    if "obj_created_dynamic" in decon_configs:
        global OBJECTS_CREATED_DYNAMIC
        OBJECTS_CREATED_DYNAMIC = decon_configs["obj_created_dynamic"]

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


def build_required_directories(decon_configs):
    """This function will re-make, and create if appropriate, the destination directories
    used by the application to save the various results.
    """
    ####
    # Output directories for deconned objects
    # Base destination directory

    # Build output directories for glb creation, if needed

    dest_abs_dir = os.path.join(DATASET_SRC_DIR, DEST_SUBDIR)
    os.makedirs(dest_abs_dir, exist_ok=True)
    global DEST_CONFIG_DIR
    DEST_CONFIG_DIR = os.path.join(dest_abs_dir, DEST_CONFIG_SUBDIR)
    os.makedirs(DEST_CONFIG_DIR, exist_ok=True)
    global DEST_GLB_DIR
    DEST_GLB_DIR = os.path.join(dest_abs_dir, DEST_GLB_SUBDIR)
    os.makedirs(DEST_GLB_DIR, exist_ok=True)

    # Directory to save stage GLB files (if created)
    global STAGE_GLB_OUTPUT_DIR
    STAGE_GLB_OUTPUT_DIR = os.path.join(DEST_GLB_DIR, STAGE_GLB_OUTPUT_SUBDIR)
    if BUILD_STAGE_GLBS:
        os.makedirs(STAGE_GLB_OUTPUT_DIR, exist_ok=True)

    # Directory to save object GLB files
    global OBJ_GLB_OUTPUT_DIR
    OBJ_GLB_OUTPUT_DIR = os.path.join(DEST_GLB_DIR, OBJECT_GLB_OUTPUT_SUBDIR)
    if BUILD_OBJECT_GLBS:
        os.makedirs(OBJ_GLB_OUTPUT_DIR, exist_ok=True)

    # Directory to save synthesized scene instance configuration JSONs
    global SCENE_INSTANCE_OUTPUT_DIR
    SCENE_INSTANCE_OUTPUT_DIR = os.path.join(
        DEST_CONFIG_DIR, SCENE_INSTANCE_OUTPUT_SUBDIR
    )
    if BUILD_SCENE_CONFIGS:
        os.makedirs(SCENE_INSTANCE_OUTPUT_DIR, exist_ok=True)

    # Directory to save stage config files (if used)
    global STAGE_CONFIG_OUTPUT_DIR
    STAGE_CONFIG_OUTPUT_DIR = os.path.join(DEST_CONFIG_DIR, STAGE_CONFIG_OUTPUT_SUBDIR)
    if BUILD_STAGE_CONFIGS:
        os.makedirs(STAGE_CONFIG_OUTPUT_DIR, exist_ok=True)

    # Directory to save object config files
    global OBJ_CONFIG_OUTPUT_DIR
    OBJ_CONFIG_OUTPUT_DIR = os.path.join(DEST_CONFIG_DIR, OBJECT_CONFIG_OUTPUT_SUBDIR)
    if BUILD_OBJECT_CONFIGS:
        os.makedirs(OBJ_CONFIG_OUTPUT_DIR, exist_ok=True)

    # Directory to save lighting configuration JSONs
    global LIGHTING_CONFIG_OUTPUT_DIR
    LIGHTING_CONFIG_OUTPUT_DIR = os.path.join(
        DEST_CONFIG_DIR, LIGHT_CONFIG_OUTPUT_SUBDIR
    )
    if BUILD_LIGHTING_CONFIGS:
        os.makedirs(LIGHTING_CONFIG_OUTPUT_DIR, exist_ok=True)

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
    stage_tag: str,
    include_obj_dict: Dict[str, Set[str]],
    build_glbs: bool,
    build_configs: bool,
    decon_configs: Dict[str, Any],
):

    # world-space transformation of stage node
    stage_transform = scene_graph.graph.get(stage_tag)[0]

    # print("Stage global transform \n{}".format(stage_transform))
    # base stage name - fully qualified directories + stub
    stage_name_base = scene_name_base  # + "_stage"

    stage_glb_dest_filename_base = os.path.join(STAGE_GLB_OUTPUT_DIR, stage_name_base)

    # export stage mesh result to glb file
    stage_glb_dest_filename = stage_glb_dest_filename_base + ".glb"

    if build_glbs:
        # Extract the stage mesh and its transform in the world
        stage_graph = gut.extract_obj_mesh_from_scenegraph(
            scene_graph,
            stage_tag,
            "world",
            include_obj_dict,
            STAGE_EXCLUDE_SUBNODES,
            True,
        )
        # display stage
        # stage_graph.show(viewer="gl")

        stage_graph.export(stage_glb_dest_filename)

    if build_configs:
        # get relative path
        rel_stage_asset_filename = ut.transform_path_relative(
            stage_glb_dest_filename, STAGE_CONFIG_OUTPUT_DIR
        )
        stage_config_dest_filename_base = os.path.join(
            STAGE_CONFIG_OUTPUT_DIR, stage_name_base
        )
        # set up stage configuration file
        stage_config_filename = (
            stage_config_dest_filename_base + ut.CONFIG_EXTENSIONS["stage"] + ".json"
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
    stage_instance_dict["translation_origin"] = "COM"

    if len(decon_configs["stage_instance_file_tag"]) != 0:
        # mapping is provided to map scene name to prebuilt/predefined stage names
        replace_stage_dict = decon_configs["stage_instance_file_tag"]
        for k, v in replace_stage_dict.items():
            if k.lower() in stage_name_base.lower():
                stage_instance_dict["template_name"] = v
                break
    return stage_instance_dict


def extract_objects_from_scene(
    scene_graph,
    objects_tag: str,
    exclude_obj_dict,
    build_glbs: bool,
    build_configs: bool,
    decon_configs: Dict[str, Any],
    match_names_dict: Dict[str, Tuple[str, str, str]],
):
    objects_raw = scene_graph.graph.transforms.children_dict[objects_tag]
    object_instance_configs = []
    objects = []

    # exclude object names that have been added to stage already
    # get set of object names that are ref'ed by objects tag -
    # these were included in stage already

    excl_str_set = exclude_obj_dict[objects_tag]
    # get set of object names that we wish to exclude -
    # these are neither objects nor stage components
    obj_excl_str_set = OBJECT_EXCLUDE_SUBNODES
    for obj_name in objects_raw:
        obj_is_valid = True
        for substr in excl_str_set:
            if substr.lower() in obj_name.lower():
                obj_is_valid = False
                break
        # check if invalid object specification should be overridden
        if not obj_is_valid:
            for substr in OBJECT_OVERRIDE_SUBNODES:
                if substr.lower() in obj_name.lower():
                    obj_is_valid = True
                    break
        # if specified as valid by here, check if not explicitly excluded
        if obj_is_valid:
            for substr in obj_excl_str_set:
                if substr.lower() in obj_name.lower():
                    obj_is_valid = False
                    break
        if obj_is_valid:
            objects.append(obj_name)
    from collections import defaultdict

    # set objects motion type
    default_str = "STATIC" if OBJECTS_ALL_STATIC else "DYNAMIC"
    # override motion type
    obj_motion_type_dict = defaultdict(lambda: default_str)
    for obj_name in objects:
        for obj_substr in OBJECTS_CREATED_STATIC:
            if obj_substr.lower() in obj_name.lower():
                obj_motion_type_dict[obj_name] = "STATIC"
    # override now for objects in OBJECTS_CREATED_DYNAMIC if all static specified
    if OBJECTS_ALL_STATIC:
        for obj_name in objects:
            for obj_substr in OBJECTS_CREATED_DYNAMIC:
                if obj_substr.lower() in obj_name.lower():
                    obj_motion_type_dict[obj_name] = "DYNAMIC"
    #
    for obj_name in objects:
        obj_exclude_dict = {}  # dict(exclude_obj_dict)

        # make the file name base to reference the object from the mesh instance name
        obj_name_base = obj_name
        # ReplicaCAD only (so far) - match instance object mesh name with existing object names instead
        # so that scene_instance references actual object correctly
        if len(match_names_dict) > 0:
            for k, _ in match_names_dict.items():
                if k.lower() in obj_name_base.lower():
                    obj_name_base = k
                    break

        obj_glb_dest_filename_base = os.path.join(OBJ_GLB_OUTPUT_DIR, obj_name_base)
        # build file names for output
        obj_glb_dest_filename = obj_glb_dest_filename_base + ".glb"
        # print("Object dest filename : {}".format(obj_dest_filename))

        if build_glbs:
            # extract the object "scene" for obj_name object instance in mesh
            # (scene is the mesh + other assets to save for object glb)
            object_scene = gut.extract_obj_mesh_from_scenegraph(
                scene_graph, obj_name, objects_tag, {}, obj_exclude_dict, False
            )
            if object_scene is None:
                continue

            object_scene.export(obj_glb_dest_filename)

        # world-space transformation of stage node
        obj_transform = scene_graph.graph.get(obj_name)[0]

        # ReplicaCAD Only - improperly named object
        # build object instance info to be used in scene instance config to place object
        obj_instance_dict = gut.build_instance_config_json(
            obj_name_base,
            obj_transform,
            reframe_transform=decon_configs["apply_object_transform"],
            calc_scale=False,
        )
        obj_instance_dict["motion_type"] = obj_motion_type_dict[obj_name]
        obj_instance_dict["translation_origin"] = "COM"

        object_instance_configs.append(obj_instance_dict)

        # set object instance motion type

        if build_configs:
            # build a config stub for this object with derived render and collision asset names
            rel_obj_dest_filename = ut.transform_path_relative(
                obj_glb_dest_filename, OBJ_CONFIG_OUTPUT_DIR
            )
            obj_config_json_dict = {
                "render_asset": rel_obj_dest_filename,
                "collision_asset": rel_obj_dest_filename,
            }
            obj_config_filename_base = os.path.join(
                OBJ_CONFIG_OUTPUT_DIR, obj_name_base
            )
            obj_config_filename = (
                obj_config_filename_base + ut.CONFIG_EXTENSIONS["object"] + ".json"
            )
            # save object config
            ut.mod_json_val_and_save(("", obj_config_filename, obj_config_json_dict))

    return object_instance_configs


def extract_articulated_objects_from_scene(
    scene_graph, objects_tag: str, scene_name_base: str, decon_configs: Dict[str, Any]
):
    # Build articulated object instances
    # currently supported only for ReplicaCAD - use name tags to copy default settings from decon config
    ao_res_list = []
    if len(decon_configs["ao_instance_mappings"]) != 0:
        import numpy as np

        # mapping is provided to map scene name to prebuilt/predefined stage names
        ao_instance_dict = decon_configs["ao_instance_mappings"]
        # mapping of scene object names to AO model names, to use the locations in the scene to place the AOs
        ao_objects_dict = decon_configs["ao_include_obj_names"]
        # list of desired AOs to include
        art_obj_template_names = [
            "fridge",
            "kitchen_counter",
            "kitchenCupboard_01",
            "door2",
        ]
        art_src_object_names = {}
        if len(ao_objects_dict) != 0:
            # get all the objects in the scene so we can find the appropriate objects to use for AO placement, if any do this
            objects_raw = scene_graph.graph.transforms.children_dict[objects_tag]
            for ao_model_name, obj_name_substr in ao_objects_dict.items():
                for obj_name in objects_raw:
                    # get actual object name within scene graph
                    if obj_name_substr.lower() in obj_name.lower():
                        art_src_object_names[ao_model_name] = obj_name
                        art_obj_template_names.append(ao_model_name)

        for k, v in ao_instance_dict.items():
            if k.lower() in scene_name_base.lower():
                mapping_dict = v
                break

        for ao_name in art_obj_template_names:

            art_obj_instance_dict = {}
            art_obj_instance_dict["template_name"] = ao_name
            art_obj_instance_dict["fixed_base"] = True
            art_obj_instance_dict["auto_clamp_joint_limits"] = True
            art_obj_instance_dict["translation_origin"] = "COM"
            art_obj_instance_dict["motion_type"] = "DYNAMIC"

            if ao_name in mapping_dict:
                # print("Mapping for : {} is {} ".format(scene_name_base, mapping_dict))
                art_obj_instance_dict["translation"] = mapping_dict[ao_name][
                    "translation"
                ]
                art_obj_instance_dict["rotation"] = mapping_dict[ao_name]["rotation"]
            elif "kitchenCupboard_01" in ao_name:
                # kitchen cupboard is always at a specific offset from counter
                counter_trans = np.array(mapping_dict["kitchen_counter"]["translation"])
                counter_rotation = np.array(mapping_dict["kitchen_counter"]["rotation"])
                new_trans = counter_trans + np.array([-0.3, 1.5, 0])
                new_rot = gut.rotate_quat_by_quat(
                    counter_rotation, np.array([-0.707, 0.707, 0, 0])
                )
                art_obj_instance_dict["uniform_scale"] = 0.38
                art_obj_instance_dict["translation"] = list(new_trans)
                new_rot /= np.linalg.norm(new_rot)
                art_obj_instance_dict["rotation"] = list(new_rot)
            elif "door2" in ao_name:
                art_obj_instance_dict["translation"] = [-0.35, 2.40, -2.65]
                art_obj_instance_dict["rotation"] = [1, 0, 0, 0]
            elif ao_name in ao_objects_dict:
                # get transform from scene graph
                obj_name = art_src_object_names[ao_name]
                obj_transform = scene_graph.graph.get(obj_name)[0]
                # temporary instance, just to provide transformations in dictionary form
                tmp_instance_dict = gut.build_instance_config_json(
                    "not_to_be_used",
                    obj_transform,
                    True,
                    calc_scale=False,
                )
                new_trans = np.array(tmp_instance_dict["translation"])
                new_rot = np.array(tmp_instance_dict["rotation"])
                if "chestOfDrawers" in ao_name:
                    new_trans[1] = 0.0
                    art_obj_instance_dict["uniform_scale"] = 0.4
                    new_rot = gut.rotate_quat_by_quat(
                        new_rot, np.array([0.707, 0.707, 0, 0])
                    )
                    # global around y
                    new_rot = gut.rotate_quat_by_quat(
                        np.array([0.707, 0.0, 0.707, 0]), new_rot
                    )
                elif "cabinet" in ao_name:
                    new_trans[1] = 0.05
                    new_trans[2] += 0.1
                    new_rot = gut.rotate_quat_by_quat(
                        new_rot, np.array([0.0, 0.0, 1.0, 0.0])
                    )
                art_obj_instance_dict["translation"] = list(new_trans)
                art_obj_instance_dict["rotation"] = list(new_rot)
                print(
                    "\tArticulated Object {} made from existing object transformation.".format(
                        ao_name
                    )
                )
            else:
                print("Articulated Object {} not yet supported".format(ao_name))
                break

            ao_res_list.append(art_obj_instance_dict)

    return ao_res_list


def extract_lighting_from_scene(
    scene_filename_glb,
    scene_graph,
    scene_name_base: str,
    lights_tag: str,
    debug: Optional[bool] = False,
):
    # Still TODO
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
    lighting_dest_filename_base = os.path.join(
        LIGHTING_CONFIG_OUTPUT_DIR, lighting_name_base
    )
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
        "stages": {
            "paths": {
                ".json": [
                    os.path.join(DEST_CONFIG_SUBDIR, STAGE_CONFIG_OUTPUT_SUBDIR, "*")
                ]
            }
        },
        "objects": {
            "paths": {
                ".json": [
                    os.path.join(DEST_CONFIG_SUBDIR, OBJECT_CONFIG_OUTPUT_SUBDIR, "*")
                ]
            }
        },
        "light_setups": {
            "paths": {
                ".json": [
                    os.path.join(DEST_CONFIG_SUBDIR, LIGHT_CONFIG_OUTPUT_SUBDIR, "*")
                ]
            }
        },
        "scene_instances": {
            "paths": {
                ".json": [
                    os.path.join(DEST_CONFIG_SUBDIR, SCENE_INSTANCE_OUTPUT_SUBDIR, "*")
                ]
            }
        },
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
                DATASET_SRC_DIR,
                DEST_SUBDIR,
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
    # Load configuration describing the scene meshes we wish to deconstruct into stages and objects
    decon_configs = load_decon_global_config_values(MESH_DECON_CONFIG_JSON_FILENAME)
    # whether a successful load occurred or not, we need to make the destination directories
    build_required_directories(decon_configs)

    # get listing of all scene glbs
    file_list = ut.get_files_matching_regex(SCENES_SRC_DIR)
    # if we wish to match mesh object instance names with existing object files, get a
    # listing of all the existing object files from the specified object source dir
    existing_obj_dict = {}
    if decon_configs["match_object_names"]:
        existing_obj_list = ut.get_files_matching_regex(OBJECTS_SRC_DIR)
        for tup in existing_obj_list:
            obj_name = tup[2].split(".")[0]
            existing_obj_dict[obj_name] = tup

    # Don't build this for replicaCAD
    # build_scene_dataset_config(SCENE_DATASET_NAME, DEFAULT_ATTRIBUTES_TEMPLATE)

    ###for testing - 17_physics has stove w/burner and knobs
    # file_list = [
    #     ("", "", "FloorPlan320_physics.glb"),
    #     ("", "", "FloorPlan_Train8_3.glb"),
    #     ("", "", "FloorPlan430_physics.glb"),
    #     ("", "", "FloorPlan17_physics.glb"),
    #     ("", "", "FloorPlan_Val2_4.glb"),
    # ]

    # count the instances of objects by name
    object_instance_count_dict = defaultdict(int)
    # file name to save object counts
    object_instance_count_filename = os.path.join(
        DEST_CONFIG_DIR,
        SCENE_DATASET_NAME + "_object_counts.json",
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

        # Empty string corresponds to default lighting within habitat sim
        lighting_setup_config_name = decon_configs["default_lighting_tag"]
        if BUILD_LIGHTING_CONFIGS:
            # extract all lighting configs in the scene
            lighting_setup_config_name = extract_lighting_from_scene(
                src_scene_filename,
                scene_graph,
                scene_name_base,
                LIGHTING_TAG,
            )

        # extract the stage from the scene glb
        stage_instance_config = extract_stage_from_scene(
            scene_graph,
            scene_name_base,
            STAGE_TAG,
            STAGE_INCLUDE_OBJ_DICT,  # included in stage, excluded as objects
            BUILD_STAGE_GLBS,
            BUILD_STAGE_CONFIGS,
            decon_configs,
        )

        # extract all the object instances within the scene
        obj_instance_config_list = extract_objects_from_scene(
            scene_graph,
            OBJECTS_TAG,
            STAGE_INCLUDE_OBJ_DICT,  # included in stage, excluded as objects
            BUILD_OBJECT_GLBS,
            BUILD_OBJECT_CONFIGS,
            decon_configs,
            existing_obj_dict,
        )

        # get counts of object instances
        for elem in obj_instance_config_list:
            object_instance_count_dict[elem["template_name"]] += 1

        if BUILD_SCENE_CONFIGS:
            # compose the scene instance configuration JSON
            scene_instance_dict = {
                "stage_instance": stage_instance_config,
                "default_lighting": lighting_setup_config_name,
            }

            if decon_configs["save_object_instances"]:
                scene_instance_dict["object_instances"] = obj_instance_config_list

            if decon_configs["save_articulated_object_instances"]:
                art_obj_instance_config_list = extract_articulated_objects_from_scene(
                    scene_graph, OBJECTS_TAG, scene_name_base, decon_configs
                )
                scene_instance_dict[
                    "articulated_object_instances"
                ] = art_obj_instance_config_list

            # save scene instance configuration
            ut.mod_json_val_and_save(
                ("", abs_scene_instance_filename, scene_instance_dict)
            )

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
    main()
