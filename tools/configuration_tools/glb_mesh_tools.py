#!/usr/bin/env python

# glb_mesh_tools : Tools for analyzing and modifying glb meshes

from collections import defaultdict

import numpy as np
import trimesh


# build a scene instance stage/object json dict from
# configuration name and transformation (4x4 mat)
def build_instance_config_json(template_name, transform):
    translation = list(trimesh.transformations.translation_from_matrix(transform))
    rotation = list(trimesh.transformations.quaternion_from_matrix(transform))
    json_dict = {
        "template_name": template_name,
        "translation": translation,
        "rotation": rotation,
        "translation_origin": "asset_local",
    }
    return json_dict


# This function will return a list of translations given a list of transformations
def get_trans_list_from_transforms(transforms):
    results = []
    for transform in transforms:
        results.append(trimesh.transformations.translation_from_matrix(transform))
    return results


# This function will construct a list of quaternion rotations
# given a list of transformations
def get_rot_list_from_transforms(transforms):
    results = []
    for transform in transforms:
        results.append(trimesh.transformations.quaternion_from_matrix(transform))
    return results


# Given a list of glb scene file names, this function will build
# dictionaries holding various quantities
def get_objects_in_scenes(scene_glbs):
    # keyed by scene, value is default dict holding scene data
    each_scene_obj_dict = {}
    for new_scene_filename in scene_glbs:
        # returns a default dict with key == object name and value is a list of transformations
        each_scene_obj_dict[new_scene_filename] = process_scene_for_objects(
            new_scene_filename
        )

    all_scene_translations_dict = defaultdict(list)
    # per object scene population
    obj_per_scene_dict = defaultdict(dict)
    # per object max counts
    obj_max_per_scene_counts = defaultdict(int)
    for scene_name, obj_in_scene_dict in each_scene_obj_dict.items():
        # obj_in_scene_dict is a scene's default dict
        # print("\n{}\nObjects and counts : ".format(scene_name))
        for obj, transforms in obj_in_scene_dict.items():
            # get list of translations from list of matrix transformations
            translations = get_trans_list_from_transforms(transforms)
            # print("{} | {}".format(obj, translations))
            all_scene_translations_dict[obj].extend(translations)
            obj_per_scene_dict[obj][scene_name] = translations
            if obj_max_per_scene_counts[obj] < len(translations):
                obj_max_per_scene_counts[obj] = len(translations)
    print("\nTotals:\n")
    for obj, translations in all_scene_translations_dict.items():
        print("{} | {}  ".format(obj, len(translations)))
    print("\n")


# this function will load a scene .glb file and find the individual objects
# and their transformations
def process_scene_for_objects(filename):
    # dict of key == object name, value == list of transforms
    scene_obj_dict = defaultdict(list)
    scene = trimesh.load(filename)
    root_children = scene.graph.transforms[scene.graph.base_frame]

    for k, v in root_children.items():
        obj_name = k.split(".")[0].replace("-", "_")
        scene_obj_dict[obj_name].append(v["matrix"])
    return scene_obj_dict


# Calculate the stats on the passed list of population data
# pop data can be a list of scalars or vectors of any size
# returned stats will be per dof of input values
def calc_stats(pop_data):
    obj_location_stats = {}
    obj_location_stats["num_entries"] = len(pop_data)
    obj_location_stats["mean"] = np.mean(pop_data, axis=0)
    obj_location_stats["std"] = np.std(pop_data, axis=0, dtype=np.float64)
    obj_location_stats["variance"] = np.var(pop_data, axis=0, dtype=np.float64)
    return obj_location_stats


# Query the given scene_graph with the given scene_object_tag
# to acquire the named object's constituent trimesh.Scene mesh
# and world transformation
def extract_obj_mesh_from_scenegraph(scene_graph, scene_object_tag):
    # world-space transformation of object in scene
    # idx 1 is geometry
    global_transform = scene_graph.graph.get(scene_object_tag)[0]
    # get scene_graph transformations hierarchy
    transforms_tree = scene_graph.graph.transforms
    # build set of successor nodes of scene graph
    sub_nodes = set(transforms_tree.successors(scene_object_tag))
    sub_nodes.add(scene_object_tag)
    # [print("{} of type {}".format(x, type(x))) for x in sub_nodes]

    # build list of edges that are part of the named object
    # by checking to see if either vert of edge is part of the subnode set
    edges = [
        e
        for e in scene_graph.graph.to_edgelist()
        if e[0] in sub_nodes or e[1] in sub_nodes
    ]

    # build a transformation graqh representing all the components of the object
    g = trimesh.scene.transforms.TransformForest(base_frame=scene_object_tag)
    # add all the edges from the source
    g.from_edgelist(edges)

    # build geometry from scene_graph that is part of this object
    geometry_names = (e[2]["geometry"] for e in edges if "geometry" in e[2])

    geometry = {k: scene_graph.geometry[k] for k in geometry_names}
    # build a new trimesh scene with constructed transformation graph and geometry
    # representing the named object
    new_scene = trimesh.Scene(geometry=geometry, graph=g)
    # undo transformation
    new_scene.apply_transform(trimesh.transformations.inverse_matrix(global_transform))
    assert new_scene.is_valid
    return new_scene, global_transform
