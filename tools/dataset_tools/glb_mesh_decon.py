#!/usr/bin/env python

# glb_mesh_decon : utlitities to deconstruct an aggregate glb scene
# mesh into constituent stage and object meshes and lighting configurations
# usable by habitat.

import os
from collections import OrderedDict, defaultdict
from typing import Any, Dict, Optional, Tuple

import config_utils as ut
import glb_mesh_tools as gut

###
# JSON Configuration file for running this application.
# MESH_DECON_CONFIG_JSON_FILENAME = "mesh_decon_AI2Thor.json"
MESH_DECON_CONFIG_JSON_FILENAME = "mesh_decon_ReplicaCAD.json"
# MESH_DECON_CONFIG_JSON_FILENAME = "mesh_decon_ReplicaCAD_baked.json"
# MESH_DECON_CONFIG_JSON_FILENAME = "mesh_decon_floorplanner.json"


def build_default_configs():
    """This function builds a dictionary with default settings for the
    various functionalities this script supports.  These are overridden
    by any values specified in JSON file pointed to by
    MESH_DECON_CONFIG_JSON_FILENAME.

    See mesh_decon_example_json.txt file for more information.
    """

    default_configs = {}
    # Name of the dataset
    default_configs["dataset_name"] = "AI2Thor"
    # whether or not the source directory for the data used by this script is relative to home
    default_configs["dataset_src_rel_to_home"] = True
    # the full source directory for the data used by this dataset, either relative to home or relative to cwd
    default_configs["dataset_src_subdir"] = "Documents/AI2Thor/"
    # the subdirectory for the source scenes, relative to dataset_src_dir
    default_configs["scenes_src_subdir"] = "scenes/"
    # the subdirectory for the source of objects, relative to dataset_src_dir
    default_configs["objects_src_subdir"] = "objects/"

    # whether the output destination directory is relative to home or not
    default_configs["dataset_dest_rel_to_home"] = True
    # the full destination directory for the data written by this script, either relative to home or relative
    # to cwd.  All pathing in written configs will be relaive to this directory
    default_configs["dataset_dest_subdir"] = "Documents/AI2Thor/scene_dataset/"

    # subdirectory within dataset_dest_dir for output of glbs
    default_configs["dataset_glb_dest_subdir"] = "."
    # subdirectory within dataset_dest_dir for output of configs
    default_configs["dataset_config_dest_subdir"] = "configs/"

    # whether or not to build stage glbs from source scenes
    default_configs["build_stage_glbs"] = True
    # subdirectory within dataset_glb_dest_subdir for saving stage glbs
    default_configs["stage_glb_dest_subdir"] = "stages/"
    # whether or not to build object glbs from source scenes
    default_configs["build_object_glbs"] = True
    # subdirectory within dataset_glb_dest_subdir for object glbs
    default_configs["obj_glb_dest_subdir"] = "objects/"

    # whether or not to build the scene dataset config
    default_configs["build_dataset_config"] = True

    # whether or not to save stage configs for each scene
    default_configs["build_stage_configs"] = True
    # subdirectory within dataset_config_dest_subdir for stage configs to be written
    default_configs["stage_config_dest_subdir"] = "stages/"
    # whether or not to save object configs for objects found in scenes
    default_configs["build_object_configs"] = True
    # subdirectory within dataset_config_dest_subdir for object configs to be written
    default_configs["obj_config_dest_subdir"] = "objects/"
    # whether or not to build lighting configs based on lighting found in scenes
    default_configs["build_lighting_configs"] = True
    # subdirectory within dataset_config_dest_subdir for lighting configs to be written
    default_configs["light_config_dest_subdir"] = "lighting/"

    # whether or not to build scene instance configs
    default_configs["build_scene_configs"] = True
    # subdirectory within dataset_config_dest_subdir for scene instance configs to be written
    default_configs["scene_instance_dest_subdir"] = "scenes/"

    # whether or not to save object instances in scene instance configs
    default_configs["save_object_instances"] = True
    # whether or not we should save articulated object instances in scene config
    default_configs["save_articulated_object_instances"] = False

    # whether to match object name refs in object config synth to objects found in obj_glb_dest_subdir.
    # Used when synthesizing scene instances for scenes with existing object glb files,
    default_configs["match_object_names"] = False

    # dictionary keyed by some substring of file name, value is stage names to use for stage
    # instance refs in scene instance files. Search for the keys within scene name,
    # and use the values as the stage to ref
    default_configs["stage_instance_file_tag"] = {}

    # tag to insert in every scene instance attributes to reference the default lighting
    default_configs["default_lighting_tag"] = ""

    # whether to apply a transformation to the object's transformation before
    # calculating translation and rotation for saving in scene instance
    default_configs["apply_object_transform"] = False

    # Whether or not to calculate and save the object instance scale found in the transformation.
    default_configs["calc_object_uniform_scale"] = True

    # tag of root within scene graph
    default_configs["world_tag"] = "world"
    # tag within scene graph to find where components of the stage reside
    default_configs["stage_tag"] = "Structure"
    # tag within scene graph to find where individual objects reside
    default_configs["objects_tag"] = "Objects"
    # tag within scene graph to find where lighting definitions reside
    default_configs["lighting_tag"] = "Lighting"

    # whether or not to specify that all objects in the scene should be static in scene instance
    default_configs["objects_all_static"] = False

    # CURRENTLY ONLY REPLICA_CAD : specifies file name tag look-up and scene instance
    # articulated object mapping to use for each range of scenes
    default_configs["ao_instance_mappings"] = {}

    # This is a dict of sets to be used to include nodes in the stage scene
    # graph that are not directly linked to the stage scene_graph. The key of
    # the dict is the name of the node that should be searched (such as the objects_tag), and value is
    # a set of lower-case substrings (for case-insensitive matching) of
    # successor node names that should be included in the stage construction.
    # The nodes whose names contain these substrings (And their subtrees) will
    # be added to the stage and not spun off as individual objects.
    default_configs["stage_include_obj_substr"] = {}

    # What to consider the origin for the stage - either its COM or an origin local to the stage
    default_configs["stage_translation_origin"] = "COM"

    # Objects specified in the "stage_include_obj_substr" dict are, by default, ignored
    # when objects are extracted from the scene graph.  The "obj_override_names"
    # dict will provide lowercase substrs of object node names (for case insensitive
    # matching) that should be explicitly included that might otherwise be ignored on
    # object extraction.  These objects will also not be included in the stage.
    default_configs["obj_override_names"] = ["floor_lamp"]

    # This is a set of substrings (case-insensitive) of node names for
    # nodes that should be excluded from object synthesis
    # Make "obj_exclude_names" an empty set to not ignore anything
    default_configs["obj_exclude_names"] = []

    # map of URDF model names to object node names to map objects/locations to AOs.
    # These should be excluded from regular object extraction, to prevent double-dipping
    default_configs["ao_scene_mapping_obj_names"] = {}

    # This is a set of lowercase substrings of names of objects that
    # should be specified as static in the scene instance upon creation
    # when an otherwise dynamic scene is being instantiated.
    # This should include large furnishings and other obstacles that we
    # do not wish to simulate dynamically
    default_configs["obj_created_static"] = []

    # This is a set of lowercase substrings of names of objects that
    # should be specified as dynamic in the scene instance upon creation
    # when a dynamic scene is being instantiated.  This is intended to provide
    # an easy override for when all objects are specified to be static
    default_configs["obj_created_dynamic"] = []

    # What to consider the origin for an object - either its COM or an origin local to each object
    default_configs["obj_translation_origin"] = "COM"

    # Whether or not to traverse to leaf for object mesh building
    default_configs["obj_recurse_subnodes"] = False

    # whether or not to export and save the source scene glbs as gltfs, for diagnostic purposes
    default_configs["export_glbs_as_gltf"] = False
    # subdirectory relative to dataset_dest_dir, where gltf exports of scenes should be saved
    default_configs["gltf_export_subdir"] = "tmpGLTFScenes/"

    # whether to perform a diagnostic investigation of the source scene graphs
    # and save the hierarchy of node names as a json file
    default_configs["save_scenegraph_hierarchy"] = True

    # directory relative to dataset dest where to save the scene graph hieararchy output configs["scenegraph_diagnostics_dir"]
    default_configs["scenegraph_diagnostics_subdir"] = "scene_graph_diagnostics/"

    tmp_dflt_attrs = {}
    tmp_dflt_attrs["stages"] = {
        "requires_lighting": "true",
        "up": [0, 1, 0],
        "front": [0, 0, -1],
        "origin": [0, 0, 0],
    }
    tmp_dflt_attrs["objects"] = {
        "requires_lighting": "true",
        "up": [0, 1, 0],
        "front": [0, 0, -1],
        "origin": [0, 0, 0],
    }
    tmp_dflt_attrs["scene_instances"] = {
        "translation_origin": "asset_local",
        "default_lighting": "",
    }
    tmp_dflt_attrs["light_setups"] = {}

    # default attributes values to be used for types in the scene dataset config
    default_configs["default_attributes"] = tmp_dflt_attrs

    return default_configs


def load_decon_global_config_values(config_json: str):
    """This function will load values for the configuration values used by this application to
    deconstruct the scene glbs into constituent stages and objects and synthesize the appropriate
    JSON configuration files for the results.
    """
    # get default values
    configs = build_default_configs()

    # Load configuration describing the scene meshes we wish to deconstruct into stages and objects
    loaded_configs = ut.load_json_into_dict(config_json)

    # map values from file to existing default configs
    for k, v in loaded_configs.items():
        # move file-based configs into defaults
        if isinstance(v, dict):
            if k not in configs:
                configs[k] = v
            else:
                tmp_dict = {}
                for k1, v1 in configs[k].items():
                    tmp_dict[k1] = v1
                # overwrite defaults
                for k1, v1 in v.items():
                    tmp_dict[k1] = v1
                configs[k] = tmp_dict
        elif isinstance(v, str):
            if "true" in v:
                configs[k] = True
            elif "false" in v:
                configs[k] = False
            else:
                configs[k] = v.strip()
        else:
            configs[k] = v

    #######################
    # build appropriate source directory and subdirs
    if "dataset_src_rel_to_home" in configs:
        configs["dataset_src_dir"] = os.path.join(
            os.path.expanduser("~"), configs["dataset_src_subdir"]
        )
    else:
        configs["dataset_src_dir"] = configs["dataset_src_subdir"]

    # Aggregate Scene GLBS source directory
    configs["scenes_src_dir"] = os.path.join(
        configs["dataset_src_dir"], configs["scenes_src_subdir"]
    )
    # Objects source GLBS directory (potentially used for verification/matching from object nodes)
    configs["objects_src_dir"] = os.path.join(
        configs["dataset_src_dir"], configs["objects_src_subdir"]
    )

    ##########################
    # build appropriate destination directory and other subdirs
    # and create them on disk if appropriate
    if "dataset_dest_rel_to_home" in configs:
        configs["dataset_dest_dir"] = os.path.join(
            os.path.expanduser("~"), configs["dataset_dest_subdir"]
        )
    else:
        configs["dataset_dest_dir"] = configs["dataset_dest_subdir"]

    # Build destination directory, if dne
    os.makedirs(configs["dataset_dest_dir"], exist_ok=True)

    ##########################
    # output directories creation
    # glbs
    configs["dataset_glb_dest_dir"] = os.path.join(
        configs["dataset_dest_dir"], configs["dataset_glb_dest_subdir"]
    )
    if configs["build_stage_glbs"] or configs["build_object_glbs"]:
        # only attempt to make directory if we will be writing glb files
        os.makedirs(configs["dataset_glb_dest_dir"], exist_ok=True)

    # stage glbs dir
    configs["stage_glb_dest_dir"] = os.path.join(
        configs["dataset_glb_dest_dir"], configs["stage_glb_dest_subdir"]
    )
    if configs["build_stage_glbs"]:
        os.makedirs(configs["stage_glb_dest_dir"], exist_ok=True)
    # object glbs
    configs["obj_glb_dest_dir"] = os.path.join(
        configs["dataset_glb_dest_dir"], configs["obj_glb_dest_subdir"]
    )
    if configs["build_object_glbs"]:
        os.makedirs(configs["obj_glb_dest_dir"], exist_ok=True)

    # configs
    configs["dataset_config_dest_dir"] = os.path.join(
        configs["dataset_dest_dir"], configs["dataset_config_dest_subdir"]
    )
    if (
        configs["build_stage_configs"]
        or configs["build_object_configs"]
        or configs["build_lighting_configs"]
        or configs["build_scene_configs"]
    ):
        os.makedirs(configs["dataset_config_dest_dir"], exist_ok=True)
    # scene instance
    configs["scene_instance_dest_dir"] = os.path.join(
        configs["dataset_config_dest_dir"], configs["scene_instance_dest_subdir"]
    )
    if configs["build_scene_configs"]:
        os.makedirs(configs["scene_instance_dest_dir"], exist_ok=True)

    # stage configs
    configs["stage_config_dest_dir"] = os.path.join(
        configs["dataset_config_dest_dir"], configs["stage_config_dest_subdir"]
    )
    if configs["build_stage_configs"]:
        os.makedirs(configs["stage_config_dest_dir"], exist_ok=True)

    # object configs
    configs["obj_config_dest_dir"] = os.path.join(
        configs["dataset_config_dest_dir"], configs["obj_config_dest_subdir"]
    )
    if configs["build_object_configs"]:
        os.makedirs(configs["obj_config_dest_dir"], exist_ok=True)

    # lighting configs
    configs["light_config_dest_dir"] = os.path.join(
        configs["dataset_config_dest_dir"], configs["light_config_dest_subdir"]
    )
    if configs["build_lighting_configs"]:
        os.makedirs(configs["light_config_dest_dir"], exist_ok=True)

    # Export glb scenes as gltf scenes
    configs["light_config_dest_dir"] = os.path.join(
        configs["dataset_config_dest_dir"], configs["light_config_dest_subdir"]
    )
    if configs["build_lighting_configs"]:
        os.makedirs(configs["light_config_dest_dir"], exist_ok=True)

    configs["gltf_export_dir"] = os.path.join(
        configs["dataset_config_dest_dir"], configs["gltf_export_subdir"]
    )
    if configs["export_glbs_as_gltf"]:
        os.makedirs(configs["gltf_export_dir"], exist_ok=True)

    # Directory to save scene graph json structure - for diagnostics
    # Don't put in dataset directory since this isn't used by habitat
    configs["scenegraph_diagnostics_out_dir"] = os.path.join(
        configs["dataset_config_dest_dir"], configs["scenegraph_diagnostics_subdir"]
    )
    if configs["save_scenegraph_hierarchy"]:
        os.makedirs(configs["scenegraph_diagnostics_out_dir"], exist_ok=True)

    return configs


def extract_stage_from_scene(
    scene_graph,
    scene_name_base: str,
    configs: Dict[str, Any],
):
    # the scenegraph tag for where the stage components can be found
    stage_tag = configs["stage_tag"]

    # world-space transformation of stage node
    stage_transform = scene_graph.graph.get(stage_tag)[0]

    # print(f"Stage global transform \n{stage_transform}")
    # base stage name - fully qualified directories + stub
    stage_name_base = scene_name_base  # + "_stage"

    stage_glb_dest_filename_base = os.path.join(
        configs["stage_glb_dest_dir"], stage_name_base
    )

    # export stage mesh result to glb file
    stage_glb_dest_filename = stage_glb_dest_filename_base + ".glb"

    # build stage glb by deconstructing aggregate scene
    if configs["build_stage_glbs"]:

        # objects in the objects-tag subgraph that should be included in the stage and not treated as objects
        include_obj_dict = {}
        include_obj_dict[configs["objects_tag"]] = configs["stage_include_obj_substr"]

        # subnodes to exclude, either because they should be objects, or because they belong to
        # neither objects nor stages
        exclude_subnode_dict = {}
        exclude_subnode_dict[configs["objects_tag"]] = configs["obj_override_names"]
        exclude_subnode_dict[stage_tag] = configs["obj_exclude_names"]
        # Extract the stage mesh and its transform in the world
        stage_graph = gut.extract_obj_mesh_from_scenegraph(
            scene_graph,
            stage_tag,
            configs["world_tag"],
            include_obj_dict,
            exclude_subnode_dict,
            True,
        )
        # display stage
        # stage_graph.show(viewer="gl")

        stage_graph.export(stage_glb_dest_filename)

    if configs["build_stage_configs"]:
        # get relative path
        stage_config_dest_dir = configs["stage_config_dest_dir"]
        rel_stage_asset_filename = ut.transform_path_relative(
            stage_glb_dest_filename, stage_config_dest_dir
        )
        stage_config_dest_filename_base = os.path.join(
            stage_config_dest_dir, stage_name_base
        )
        # set up stage configuration file
        stage_config_filename = (
            stage_config_dest_filename_base + ut.CONFIG_EXTENSIONS["stage"] + ".json"
        )
        # print(f"Relative stage path :{rel_stage_asset_filename}")
        stage_config_json_dict = {
            "render_asset": rel_stage_asset_filename,
            "collision_asset": rel_stage_asset_filename,
        }
        dflt_stage_dict = configs["default_attributes"]["stages"]
        # set defaults for stage config
        for k, v in dflt_stage_dict.items():
            stage_config_json_dict[k] = v

        # save config
        ut.mod_json_val_and_save(("", stage_config_filename, stage_config_json_dict))
    # stage component of scene instance config dict
    stage_instance_dict = gut.build_instance_config_json(
        stage_name_base, stage_transform
    )
    stage_instance_dict["translation_origin"] = configs["stage_translation_origin"]

    if len(configs["stage_instance_file_tag"]) != 0:
        # mapping is provided to map scene name to prebuilt/predefined stage names
        replace_stage_dict = configs["stage_instance_file_tag"]
        for k, v in replace_stage_dict.items():
            if k.lower() in stage_name_base.lower():
                stage_instance_dict["template_name"] = v
                break
    return stage_instance_dict


def extract_objects_from_scene(
    scene_graph,
    configs: Dict[str, Any],
    match_names_dict: Dict[str, Tuple[str, str, str]],
):
    # the scenegraph subnode name for where the object components can be found
    objects_tag = configs["objects_tag"]
    # objects in the objects-tag subgraph that should be included in the stage and not treated as objects
    exclude_obj_dict = {}
    exclude_obj_dict[configs["objects_tag"]] = configs["stage_include_obj_substr"]

    build_glbs = configs["build_object_glbs"]
    build_configs = configs["build_object_configs"]
    # default object config values
    dflt_obj_dict = configs["default_attributes"]["objects"]

    # init objects
    objects_raw = scene_graph.graph.transforms.children_dict[objects_tag]
    object_instance_configs = []
    objects = []

    # exclude object names that have been added to stage already
    # get set of object names that are ref'ed by objects tag -
    # these were included in stage already

    excl_str_set = exclude_obj_dict[objects_tag]
    # get set of object names that we wish to exclude -
    # these are neither objects nor stage components
    obj_excl_str_set = configs["obj_exclude_names"]
    for obj_name in objects_raw:
        obj_is_valid = True
        for substr in excl_str_set:
            if substr.lower() in obj_name.lower():
                obj_is_valid = False
                break
        # check if invalid object specification should be overridden
        if not obj_is_valid:
            for substr in configs["obj_override_names"]:
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
    default_str = "STATIC" if configs["objects_all_static"] else "DYNAMIC"
    # override motion type
    obj_motion_type_dict = defaultdict(lambda: default_str)
    obj_created_static = configs["obj_created_static"]
    for obj_name in objects:
        for obj_substr in obj_created_static:
            if obj_substr.lower() in obj_name.lower():
                obj_motion_type_dict[obj_name] = "STATIC"
    # override now for objects in "obj_created_dynamic" list if all static specified
    if configs["objects_all_static"]:
        obj_created_dynamic = configs["obj_created_dynamic"]
        for obj_name in objects:
            for obj_substr in obj_created_dynamic:
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

        obj_glb_dest_filename_base = os.path.join(
            configs["obj_glb_dest_dir"], obj_name_base
        )
        # build file names for output
        obj_glb_dest_filename = obj_glb_dest_filename_base + ".glb"
        # print(f"Object dest filename : {obj_glb_dest_filename}")

        if build_glbs:
            # extract the object "scene" for obj_name object instance in mesh
            # (scene is the mesh + other assets to save for object glb)
            object_scene = gut.extract_obj_mesh_from_scenegraph(
                # last arg changed to true for floorplanner, which needs to recurse all the way to the leaf
                scene_graph,
                obj_name,
                objects_tag,
                {},
                obj_exclude_dict,
                configs["obj_recurse_subnodes"],
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
            reframe_transform=configs["apply_object_transform"],
            calc_scale=configs["calc_object_uniform_scale"],
        )
        obj_instance_dict["motion_type"] = obj_motion_type_dict[obj_name]
        obj_instance_dict["translation_origin"] = configs["obj_translation_origin"]

        object_instance_configs.append(obj_instance_dict)

        # set object instance motion type

        if build_configs:
            # build a config stub for this object with derived render and collision asset names
            rel_obj_dest_filename = ut.transform_path_relative(
                obj_glb_dest_filename, configs["obj_config_dest_dir"]
            )
            obj_config_json_dict = {
                "render_asset": rel_obj_dest_filename,
                "collision_asset": rel_obj_dest_filename,
            }
            # set defaults for stage config
            for k, v in dflt_obj_dict.items():
                obj_config_json_dict[k] = v

            if "scale" in obj_instance_dict:
                obj_config_json_dict["scale"] = obj_instance_dict["scale"]
            obj_config_filename_base = os.path.join(
                configs["obj_config_dest_dir"], obj_name_base
            )
            obj_config_filename = (
                obj_config_filename_base + ut.CONFIG_EXTENSIONS["object"] + ".json"
            )
            # save object config
            ut.mod_json_val_and_save(("", obj_config_filename, obj_config_json_dict))
        if "scale" in obj_instance_dict:
            # scale key not supported in object instances
            del obj_instance_dict["scale"]
    return object_instance_configs


# Currently only supports AOs in ReplicaCAD
def extract_articulated_objects_from_scene(
    scene_graph, scene_name_base: str, configs: Dict[str, Any]
):
    # objects node tag
    objects_tag = configs["objects_tag"]
    # Build articulated object instances
    # currently supported only for ReplicaCAD - use name tags to copy default settings from decon config
    ao_res_list = []
    if len(configs["ao_instance_mappings"]) != 0:
        import numpy as np

        # mapping is provided to map scene name to prebuilt/predefined stage names
        ao_instance_dict = configs["ao_instance_mappings"]
        # mapping of scene object names to AO model names, to use the locations in the scene to place the AOs
        # if scene object name specified in dict is empty, just use json-specified absolute transforms
        ao_objects_dict = configs["ao_scene_mapping_obj_names"]
        # list of desired AOs to include
        art_obj_template_names = []
        art_src_object_names = {}
        if len(ao_objects_dict) != 0:
            # get all the objects in the scene so we can find the appropriate objects to use for AO placement, if any do this
            objects_raw = scene_graph.graph.transforms.children_dict[objects_tag]
            for ao_model_name, obj_name_substr in ao_objects_dict.items():
                if not obj_name_substr:
                    # empty scene node name, so not going to use scene graph for any transform
                    art_obj_template_names.append(ao_model_name)
                    art_src_object_names[ao_model_name] = ""
                else:
                    for obj_name in objects_raw:
                        # get actual object name within scene graph
                        if obj_name_substr.lower() in obj_name.lower():
                            art_src_object_names[ao_model_name] = obj_name
                            art_obj_template_names.append(ao_model_name)
                            break

        # find mapping_dict for passed scene in config
        for scene_name, ao_object_mapping_dict in ao_instance_dict.items():
            if scene_name.lower() in scene_name_base.lower():
                mapping_dict = ao_object_mapping_dict
                break

        for ao_name in art_obj_template_names:
            # get object mapping dictionary from config for this object
            ao_mapping_dict = mapping_dict[ao_name]

            # set articulated object default scene instance values
            art_obj_instance_dict = {}
            art_obj_instance_dict["template_name"] = ao_name
            art_obj_instance_dict["fixed_base"] = True
            art_obj_instance_dict["auto_clamp_joint_limits"] = True
            art_obj_instance_dict["translation_origin"] = "COM"
            art_obj_instance_dict["motion_type"] = "DYNAMIC"
            art_obj_instance_dict["translation"] = [0, 0, 0]
            art_obj_instance_dict["rotation"] = [1, 0, 0, 0]

            if ao_name in mapping_dict:
                # apply scaling if present
                if "uniform_scale" in ao_mapping_dict:
                    art_obj_instance_dict["uniform_scale"] = ao_mapping_dict[
                        "uniform_scale"
                    ]

                # apply absoulate transformations, if provided in mapping dict
                # or present in scene graph
                sg_obj_name = art_src_object_names[ao_name]
                # if sg_obj_name is empty, then transform is either 0 vec, or provided in json
                if not sg_obj_name:
                    # Use absolute transforms if provided in json
                    if "absolute_translation" in ao_mapping_dict:
                        art_obj_instance_dict["translation"] = ao_mapping_dict[
                            "absolute_translation"
                        ]
                    if "absolute_rotation" in ao_mapping_dict:
                        art_obj_instance_dict["rotation"] = ao_mapping_dict[
                            "absolute_rotation"
                        ]
                    print(
                        f"\tArticulated Object {ao_name} placed using absolute transforms in JSON."
                    )

                else:
                    # Use placement in scene graph as absolute transforms, then mod
                    sg_obj_transform = scene_graph.graph.get(sg_obj_name)[0]
                    # temporary instance, just to provide transformations in dictionary form
                    tmp_instance_dict = gut.build_instance_config_json(
                        "not_to_be_used",
                        sg_obj_transform,
                        True,
                        calc_scale=False,
                    )
                    # "absolute" transform provided by scene graph placement
                    new_trans = np.array(tmp_instance_dict["translation"])
                    new_rot = np.array(tmp_instance_dict["rotation"])
                    # process translation overrides
                    if "translation_override_x" in ao_mapping_dict:
                        new_trans[0] = ao_mapping_dict["translation_override_x"]
                    if "translation_override_y" in ao_mapping_dict:
                        new_trans[1] = ao_mapping_dict["translation_override_y"]
                    if "translation_override_z" in ao_mapping_dict:
                        new_trans[2] = ao_mapping_dict["translation_override_z"]

                    # process relative translation components - modify existing transform using relative
                    if "relative_translation" in ao_mapping_dict:
                        rel_trans = ao_mapping_dict["relative_translation"]
                        for i in range(len(new_trans)):
                            new_trans[i] += rel_trans[i]

                    if "local_relative_rotation" in ao_mapping_dict:
                        rel_rotation = np.array(
                            ao_mapping_dict["local_relative_rotation"]
                        )
                        new_rot = gut.rotate_quat_by_quat(new_rot, rel_rotation)

                    if "global_relative_rotation" in ao_mapping_dict:
                        glbl_rotation = np.array(
                            ao_mapping_dict["global_relative_rotation"]
                        )
                        new_rot = gut.rotate_quat_by_quat(glbl_rotation, new_rot)

                    new_rot /= np.linalg.norm(new_rot)
                    art_obj_instance_dict["translation"] = list(new_trans)
                    art_obj_instance_dict["rotation"] = list(new_rot)

                    print(
                        f"\tArticulated Object {ao_name} made from existing object transformation."
                    )
            ao_res_list.append(art_obj_instance_dict)

    return ao_res_list


def extract_lighting_from_scene(
    scene_filename_glb,
    scene_graph,
    scene_name_base: str,
    configs: Dict[str, Any],
    debug: Optional[bool] = False,
):
    # scene graph node holding lights info
    lights_tag = configs["lighting_tag"]
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
                    print(f"\t{k}[{i}] : {elem}")
                    i += 1
            else:
                print(f"\t{k} : {v}")

    # base stage name - fully qualified directories + stub
    lighting_name_base = scene_name_base + "_lighting"
    lighting_dest_filename_base = os.path.join(
        configs["light_config_dest_dir"], lighting_name_base
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


def build_scene_dataset_config(configs):
    scene_dataset_dest_dir = os.path.join(
        configs["dataset_dest_dir"],
        configs["dataset_name"] + ut.CONFIG_EXTENSIONS["dataset"] + ".json",
    )
    scene_dataset_config = {
        "stages": {
            "paths": {
                ".json": [
                    os.path.join(
                        ut.transform_path_relative(
                            configs["stage_config_dest_dir"],
                            configs["dataset_dest_dir"],
                        ),
                        "*",
                    )
                ]
            }
        },
        "objects": {
            "paths": {
                ".json": [
                    os.path.join(
                        ut.transform_path_relative(
                            configs["obj_config_dest_dir"],
                            configs["dataset_dest_dir"],
                        ),
                        "*",
                    )
                ]
            }
        },
        "light_setups": {
            "paths": {
                ".json": [
                    os.path.join(
                        ut.transform_path_relative(
                            configs["light_config_dest_dir"],
                            configs["dataset_dest_dir"],
                        ),
                        "*",
                    )
                ]
            }
        },
        "scene_instances": {
            "paths": {
                ".json": [
                    os.path.join(
                        ut.transform_path_relative(
                            configs["scene_instance_dest_dir"],
                            configs["dataset_dest_dir"],
                        ),
                        "*",
                    )
                ]
            }
        },
    }
    # add default attributes specs if any exist
    for k, v in configs["default_attributes"].items():
        if len(v) > 0:
            scene_dataset_config[k]["default_attributes"] = v
    # Save scene dataset config
    ut.mod_json_val_and_save(
        (
            "",
            scene_dataset_dest_dir,
            scene_dataset_config,
        )
    )


# build dict rep of scene graph and save to json
def build_scene_graph_diagnostic(configs, scene_graph, scene_name_base):
    # get node graph dictionary
    sg_node_dict = gut.build_glb_scene_graph_dict(scene_graph, configs["world_tag"])
    # print(f"{sg_node_dict}")
    # build file name
    abs_sg_diagnostic_filename = os.path.join(
        configs["scenegraph_diagnostics_out_dir"], f"{scene_name_base}_sg_layout.json"
    )
    # save node graph dictionary as json
    ut.mod_json_val_and_save(("", abs_sg_diagnostic_filename, sg_node_dict))


def main():
    # Load configuration describing the scene meshes we wish to deconstruct into stages and objects
    decon_configs = load_decon_global_config_values(MESH_DECON_CONFIG_JSON_FILENAME)

    # get listing of all scene glbs
    file_list = ut.get_files_matching_regex(decon_configs["scenes_src_dir"])

    # if we wish to match mesh object instance names with existing object files, get a
    # listing of all the existing object files from the specified object source dir
    existing_obj_dict = {}
    if decon_configs["match_object_names"]:
        existing_obj_list = ut.get_files_matching_regex(
            decon_configs["objects_src_dir"]
        )
        for tup in existing_obj_list:
            obj_name = tup[2].split(".")[0]
            existing_obj_dict[obj_name] = tup

    if decon_configs["build_dataset_config"]:
        build_scene_dataset_config(decon_configs)

    # count the instances of objects by name
    object_instance_count_dict = defaultdict(int)
    # file name to save object counts
    object_instance_count_filename = os.path.join(
        decon_configs["dataset_config_dest_dir"],
        decon_configs["dataset_name"] + "_object_counts.json",
    )

    # Go through every scene glb file
    for path_file_tuple in file_list:
        scene_name = path_file_tuple[-1]
        # test this scene
        # scene_name = "FloorPlan320_physics.glb"
        scene_name_base = scene_name.split(".glb")[0]

        src_scene_filename = os.path.join(decon_configs["scenes_src_dir"], scene_name)
        # build scene instance config file
        abs_scene_instance_filename = os.path.join(
            decon_configs["scene_instance_dest_dir"],
            scene_name_base + ut.CONFIG_EXTENSIONS["scene"] + ".json",
        )
        print(f"Scene {scene_name} Scene Instance name {abs_scene_instance_filename}")

        # Save individual stage and objects extracted from scene and
        # construct scene instance json based on layout
        scene_graph = gut.load_glb_as_scene(src_scene_filename)
        if scene_graph is None:
            continue

        if decon_configs["export_glbs_as_gltf"]:
            # Convert glb scenes to gltfs and save
            gltf_scene_name = os.path.join(
                decon_configs["gltf_export_dir"], scene_name_base + ".gltf"
            )
            convert_success = gut.convert_glb_to_gltf(
                src_scene_filename, gltf_scene_name, True
            )
            print(
                f"\t{src_scene_filename} {('' if convert_success else 'not ')}successfully converted."
            )

        # If requested build diagnostic info about scene graph and save
        if decon_configs["save_scenegraph_hierarchy"]:
            build_scene_graph_diagnostic(decon_configs, scene_graph, scene_name_base)

        # Empty string corresponds to default lighting within habitat sim
        lighting_setup_config_name = decon_configs["default_lighting_tag"]
        if decon_configs["build_lighting_configs"]:
            # extract all lighting configs in the scene
            lighting_setup_config_name = extract_lighting_from_scene(
                src_scene_filename,
                scene_graph,
                scene_name_base,
                decon_configs,
            )

        # extract the stage from the scene glb
        stage_instance_config = extract_stage_from_scene(
            scene_graph,
            scene_name_base,
            decon_configs,
        )

        # extract all the object instances within the scene
        obj_instance_config_list = extract_objects_from_scene(
            scene_graph,
            decon_configs,
            existing_obj_dict,
        )

        # get counts of object instances
        for elem in obj_instance_config_list:
            object_instance_count_dict[elem["template_name"]] += 1

        if decon_configs["build_scene_configs"]:
            # compose the scene instance configuration JSON
            scene_instance_dict = {
                "stage_instance": stage_instance_config,
                "default_lighting": lighting_setup_config_name,
            }

            if decon_configs["save_object_instances"]:
                scene_instance_dict["object_instances"] = obj_instance_config_list

            if decon_configs["save_articulated_object_instances"]:
                art_obj_instance_config_list = extract_articulated_objects_from_scene(
                    scene_graph, scene_name_base, decon_configs
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
