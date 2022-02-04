#!/usr/bin/env python

# glb_mesh_tools : Tools for analyzing and modifying glb meshes

import contextlib
from collections import defaultdict
from typing import Any, Dict, List, Optional, Set, Tuple

import config_utils as ut
import numpy as np
import trimesh

# To support the use of trimesh in this code, you need to install
# conda install -c conda-forge trimesh networkx scikit-image shapely rtree pyembree


def convert_glb_to_gltf(
    src_glb_file_name: str, dest_gltf_file_name: str, overwrite: bool
):
    """Convert the passed glb file to a gltf and save. Uses pygltflib to perform conversion,
    since trimesh's support for this function is incomplete/questionable.  The output of this process is
    a JSON and a bin file holding assets.
    :param src_glb_file_name: The glb file to read and convert. Extension must be .glb or .bin
    :param dest_gltf_file_name: The name of the gltf file to write
    :param overwrite: Overwrite existing file?
    :return: Whether successful or not
    """
    try:
        from pygltflib.utils import glb2gltf
    except ImportError:
        print(
            "pygltflib must be installed to use the glb to gltf functionality. Aborting"
        )
        return False

    # function writes garbage to console so redirect stdout
    with contextlib.redirect_stdout(None):
        success = glb2gltf(src_glb_file_name, dest_gltf_file_name, overwrite)

    return success


def load_glb_as_gltf(glb_file_name: str):
    """Extract the JSON data from the glb file specified.
    :param glb_file_name: the file to acquire the json from.  Must be .glb or .bin
    :return: dict of JSON
    """
    try:
        from pygltflib import GLTF2
    except ImportError:
        print(
            f"pygltflib must be installed to access the JSON in the glb file {glb_file_name}. Aborting"
        )
        return {}

    return GLTF2().load(glb_file_name)


def extract_json_from_glb(glb_file_name: str):
    """Extract the JSON data from the glb file specified.
    :param glb_file_name: the file to acquire the json from.  Must be .glb or .bin
    :return: dict of JSON
    """
    try:
        from pygltflib import GLTF2  # , delete_empty_keys, gltf_asdict
    except ImportError:
        print(
            f"pygltflib must be installed to access the JSON in the glb file {glb_file_name}. Aborting"
        )
        return {}

    gltf_obj = GLTF2().load(glb_file_name)
    if gltf_obj == {}:
        return gltf_obj

    import json

    # function writes garbage to console so redirect stdout
    with contextlib.redirect_stdout(None):
        # want dictionary representation
        # res = delete_empty_keys(gltf_asdict(gltf_obj))
        res = json.loads(gltf_obj.gltf_to_json())
    return res


def build_lights_dict_from_nodes(
    light_list: List[Dict[str, Any]], node_list: List[Dict[str, Any]]
):
    # light_list holds light info, accessed by index
    # node_list holds all nodes - find extension.kh
    pass


def load_glb_as_scene(src_scene_filename: str):
    """This function attempst to import the given glb file as a Trimesh Scene
    file.
    """
    import sys

    try:
        with contextlib.redirect_stderr(None):
            scene_graph = trimesh.load(src_scene_filename)
            # add default dict structure of node hierarchy to scene_graph, to enable faster access later
            children_dict = defaultdict(list)
            # append children to parent references
            # skip self-references to avoid a node loop
            [
                children_dict[v].append(u)
                for u, v in scene_graph.graph.transforms.parents.items()
                if u != v
            ]
            scene_graph.graph.transforms.children_dict = children_dict
    except BaseException:
        print(
            f"Unable to load file {src_scene_filename} - not recognized as valid mesh file. Error thrown : {sys.exc_info()}. Aborting"
        )
        return None
    return scene_graph


def build_glb_scene_graph_dict(scene_graph, root_tag: str):
    """This function will recursively walk through a scene graph starting
    at root_tag and build a dictionary of node hierarchies found that can
    then be written out to a json file for diagnostic purposes
    :param scene_graph: The trimesh Scene object
    :param root_tag: the name of the node to grow the tree from
    :return: a dictionary representation of the scene graph hierarchy suitable for
    writing to json
    """

    def build_dict_from_children(transforms_tree, node_tag):
        sub_nodes = set(transforms_tree.children_dict[node_tag])
        sub_nodes_config = []
        for n in sub_nodes:
            sub_nodes_config.append(build_dict_from_children(transforms_tree, n))
        if len(sub_nodes_config) > 0:
            return {node_tag: sub_nodes_config}
        else:
            return node_tag

    transforms_tree = scene_graph.graph.transforms
    return build_dict_from_children(transforms_tree, root_tag)


def rotate_vector_by_quat_list(target_vector: List[float], rot_quat: List[float]):
    return rotate_vector_by_quat(np.array(target_vector), np.array(rot_quat))


def rotate_vector_by_quat(target_vector: np.ndarray, rot_quat: np.ndarray):
    """Rotate target_quat by rot_quat.  Quats are assumed to be np arrays with scalar
    component in idx 0.
    """
    # normalize source rotation quat
    rot_quat_norm = rot_quat / np.linalg.norm(rot_quat)

    # precalc
    rot_vec = rot_quat_norm[1:]
    rot_w = rot_quat_norm[0]
    u_cross_p = np.cross(rot_vec, target_vector)

    # calculate simplified nested hamitlonian prods
    res_vector1 = target_vector + 2 * (
        (rot_w * u_cross_p) + np.cross(rot_vec, u_cross_p)
    )

    return res_vector1


def hamilton_prod_two_quats(quat_a: np.ndarray, quat_b: np.ndarray):
    """Multiply two quaternions via hamilton product."""
    res_quat = np.zeros(4)
    # not a real quat, just a numpy array
    vec_a = quat_a[1:]
    vec_b = quat_b[1:]
    w = quat_a[0] * quat_b[0] - (np.dot(vec_a, vec_b))
    vec = (quat_a[0] * vec_b) + (quat_b[0] * vec_a) + (np.cross(vec_a, vec_b))

    res_quat[0] = w
    res_quat[1:] = vec
    return res_quat


def rotate_quat_by_quat(target_quat: np.ndarray, rot_quat: np.ndarray):
    """Rotate target_quat by rot_quat.  Quats are assumed to be np arrays with scalar
    component in idx 0.
    """

    res_quat = hamilton_prod_two_quats(target_quat, rot_quat)
    # normalize to return non-scaling rotation quat
    res_quat /= np.linalg.norm(res_quat)
    return res_quat


def rotate_quat_by_halfpi(target_quat: np.ndarray):
    """Rotate target_quat by pi/2 around x - correct blender gltf export issues."""

    # pi/2 rotation around x axis
    x_rot = np.zeros(4)
    rot_val = 0.5 * np.sqrt(2.0)
    x_rot[0] = rot_val
    x_rot[1] = -rot_val
    x_rot /= np.linalg.norm(x_rot)
    # rotate roation quat by pi/2 around x axis
    return rotate_quat_by_quat(target_quat, x_rot)


def build_instance_config_json(
    template_name: str,
    transform: np.ndarray,
    reframe_transform: Optional[bool] = False,
    calc_scale: Optional[bool] = True,
):
    """This function builds a dictionary holding json configuration data
    for a instance of either the stage or an object for scene instance configuration
    data.
    :param template_name: Th  name of the template representing the stage or object
    in the scene dataset.
    :param transform: 4x4 matrix transformation of the object in the scene instance
    :param reframe_transform: Whether we need to
    :param calc_scale: Whether or not to calculate and save the scale found in the transformation.
    :return dictionary holding json values to add to scene instance configuration.
    """

    translation = list(trimesh.transformations.translation_from_matrix(transform))
    rotation_quat = trimesh.transformations.quaternion_from_matrix(transform)
    if reframe_transform:
        rotation_quat = rotate_quat_by_halfpi(rotation_quat)

    rotation = list(rotation_quat)

    json_dict = {
        "template_name": template_name,
        "translation": translation,
        "rotation": rotation,
    }
    if calc_scale:
        scale = [
            np.linalg.norm(transform[:3, 0]),
            np.linalg.norm(transform[:3, 1]),
            np.linalg.norm(transform[:3, 2]),
        ]
        # do not save near unit scale
        if not np.allclose(scale, np.ones(3)):
            if np.allclose(scale / scale[0], np.ones(3)):
                json_dict["uniform_scale"] = scale[0]
            else:
                json_dict["scale"] = scale
    return json_dict


def get_trans_list_from_transforms(transforms: List[np.ndarray]):
    """This function takes a list of transformations and returns a list
    of the translation component of each transformation.
    :param transforms: List of 4x4 transformations
    :return: List of vectors representing translations
    """

    results = []
    for transform in transforms:
        results.append(trimesh.transformations.translation_from_matrix(transform))
    return results


# This function will construct a list of quaternion rotations
# given a list of transformations
def get_rot_list_from_transforms(transforms: List[np.ndarray]):
    """This function takes a list of transformations and returns a list
    of the quaternion rotations of each transformation.
    :param transforms: List of 4x4 transformations
    :return: List of quats representing rotations
    """

    results = []
    for transform in transforms:
        results.append(trimesh.transformations.quaternion_from_matrix(transform))
    return results


def process_scene_for_objects(filename: str, ignore_object_names: List[str]):
    """This function will load a scene .glb file and find the individual
    objects and their transformations, ignoring any objects whose names
    contain a substring found in ignore_object_names.

    :param filename: The file name of the scene .glb
    :param ignore_object_names A list of substrings of names that should be
    ignored

    :return: A default dictionary of lists keyed by object holding the
    transformations of every instance of that object in the given scene
    """

    # dict of key == object name, value == list of transforms
    scene_obj_dict = defaultdict(list)
    # load the trimesh.Scene corresponding to the given filename
    scene = trimesh.load(filename)
    root_children = scene.graph.transforms[scene.graph.base_frame]

    for k, v in root_children.items():
        ignore = False
        # if k contains a pattern we wish to ignore, break out of the loop
        for pattern in ignore_object_names:
            if pattern in k:
                ignore = True
                break
        if ignore:
            continue
        obj_name = k.split(".")[0].replace("-", "_")
        scene_obj_dict[obj_name].append(v["matrix"])
    return scene_obj_dict


def get_objects_in_scenes(
    scene_glb_list: List[str],
    ignore_object_names: List[str],
    debug: Optional[bool] = False,
):
    """Given a list of scene glb names, this function will build dictionaries holding
    per-object quantities for each of the objects found in the given scenes
    :param scene_glb_list: List of scene glb files to be processed together.
    This probably represents some subset of a scene dataset.
    :param ignore_object_names: A list of substrings of names that should be
    ignored when aggregating object stats
    :param debug: Set to true to print out relevant information

    :return: a tuple containing two dictionaries keyed by object, one whose values
    are dictionaries keyed by scene of the objects transformations, and the other
    holding the max count of instances in any particular scene of the object.
    """
    # keyed by scene, value is default dict holding scene data
    each_scene_obj_dict = {}
    for new_scene_filename in scene_glb_list:
        # returns a default dict with key == object name and value is a list of transformations
        each_scene_obj_dict[new_scene_filename] = process_scene_for_objects(
            new_scene_filename, ignore_object_names
        )
    # keyed by object, list of all translations found for that object across all scenes
    all_scene_transforms = defaultdict(list)
    # per object per scene list of translations
    obj_per_scene_transforms = defaultdict(dict)
    # per object max counts in a scene over all scenes
    obj_max_per_scene_counts = defaultdict(int)
    # for each scene, get dictionary of all objects in that scene
    for scene_name, obj_in_scene_dict in each_scene_obj_dict.items():
        # obj_in_scene_dict is a scene's default dict
        for obj, transforms in obj_in_scene_dict.items():
            all_scene_transforms[obj].extend(transforms)
            obj_per_scene_transforms[obj][scene_name] = transforms
            num_trans = len(transforms)
            if obj_max_per_scene_counts[obj] < num_trans:
                obj_max_per_scene_counts[obj] = num_trans

    if debug:
        for scene_name, obj_in_scene_dict in each_scene_obj_dict.items():
            print(f"\nScene {scene_name}\nObjects and counts : ")
            for obj in obj_in_scene_dict:
                print(
                    f"\t{obj} | Translation List : {obj_per_scene_transforms[obj][scene_name]}"
                )

        print("\nTotals:\n")
        for obj, transforms in all_scene_transforms.items():
            print(
                f"Object : {obj} | Total counts across all scenes : {len(transforms)}  "
            )
        print("\n")
    return obj_per_scene_transforms, obj_max_per_scene_counts


def calc_stats(pop_data: List[np.ndarray]):
    """Calculate the stats on the passed list of population data
    pop data can be a list of scalars or vectors of any size
    :param pop_data: list of scalars or vectors of data
    :return: Dictionary holding the stats of this passed data
    """
    obj_location_stats = {}
    obj_location_stats["num_entries"] = len(pop_data)
    obj_location_stats["mean"] = np.mean(pop_data, axis=0)
    obj_location_stats["std"] = np.std(pop_data, axis=0, dtype=np.float64)
    obj_location_stats["variance"] = np.var(pop_data, axis=0, dtype=np.float64)
    return obj_location_stats


def show_edges_geometry(edge_types: str, edges: List[Tuple[str, str, Dict[str, Any]]]):
    print(f"\n\n{edge_types} has geometry:")
    for e in edges:
        if "geometry" in e[2]:
            print(f"Edge : {e}")

    print(f"\n\n{edge_types} does not have geometry:")
    for e in edges:
        if "geometry" not in e[2]:
            print(f"Edge : {e}")


def get_nodes_that_match_set(
    graph,
    tag_dict: Dict[str, Set[str]],
    allOrSuccessor: bool,
    debug: Optional[bool] = False,
):
    """This function returns the set of all node names in graph
    whose names contain the substrings in tag_dict
    :param graph: a networkx digraph (used by trimesh for scene graph)
    :param tag_dict: a dict where key is node to search and value is
    set of substrings to search for in successor node names in graph.
    :return: a set of node names that match the substring values in tag_dict
    """

    res_dict = {}
    # get a list of all the nodes in the graph
    all_graph_nodes = graph.node_data.keys()

    for parent_node, sub_str_set in tag_dict.items():
        match_node_names = set()
        # get the sub_graph nodes corresponding to this
        if allOrSuccessor:
            # use all nodes to check for matches
            match_nodes = all_graph_nodes
        else:
            # use only successor nodes to check for matches
            match_nodes = set(graph.children_dict[parent_node])
        for node_name in match_nodes:
            for sub_str in sub_str_set:
                if sub_str.lower() in node_name.lower():
                    match_node_names.add(node_name)
                    break
        res_dict[parent_node] = match_node_names
    if debug:
        # display node names matching substrings under
        # each parent node in scene graph
        for k, v in tag_dict.items():
            print(f"Parent node : {k} Substr to match : {v}")

        print(f"Node Matches : {len(match_node_names)}")
        for k, v in res_dict.items():
            print(f"\nFor parent object : {k}")
            for n in v:
                print(f"\t{n}")

    return res_dict


def extract_obj_mesh_from_scenegraph(
    scene_graph,
    scene_object_tag: str,
    parent_node_name: str,
    include_obj_dict: Dict[str, Set[str]],
    exclude_obj_dict: Dict[str, Set[str]],
    recurse_subnodes: bool,
):
    """This function takes a trimesh Scene object and will extract a
    new Scene object corresponding to the geometry and connectivity
    accessed via the passed object tag.
    :param scene_graph: The trimesh Scene object
    :param scene_object_tag: The name of the object to look for in the
    Scene graph
    :param parent_node_name: The name of the node in the scene graph
    that is the parent of scene_object_tag node
    :param include_obj_dict: dict where key is parent node and value is set of
    substrings to use to find desired nodes to add to the new object that are
    not directly connected to existing object's sub_graph.
    :param exclude_obj_dict: dict where key is parent node (ignored here) and
    value is set of substrings to use to find node names to exclude from this
    object that might be connected via the scene graph.
    :param recurse_subnodes: Whether or not to recurse through node tree to leafs
    or just use the first level to build edge set.
    :return: The new Scene object corresponding to scene_object_tag.
    """
    # get scene_graph transformations hierarchy
    transforms_tree = scene_graph.graph.transforms
    # build set of nodes that should be excluded
    exclude_nodes = set()
    if len(exclude_obj_dict) > 0:
        # remove excluded tags
        exclude_subnodes = get_nodes_that_match_set(
            transforms_tree, exclude_obj_dict, allOrSuccessor=True, debug=False
        )
        for node_set in exclude_subnodes.values():
            exclude_nodes.update(node_set)

    # build set of successor node names of scene graph for scene_object_tag
    if recurse_subnodes:
        sub_nodes = get_node_set_recurse(
            transforms_tree, exclude_nodes, scene_object_tag
        )
    else:
        sub_nodes = set(transforms_tree.children_dict[scene_object_tag])
    sub_nodes.add(scene_object_tag)
    # print(f"{scene_object_tag} # of subnodes : {len(sub_nodes)}")
    # for n in sub_nodes:
    #     print(f"\t{n}")

    # remove any excluded node names
    if len(exclude_nodes) > 0:
        sub_nodes -= exclude_nodes
        # may have removed all nodes
        if len(sub_nodes) == 0:
            # removed/excluded all elements
            return None, None

    # build list of edges that are part of the named object
    # by checking to see if either vert of edge is part of the subnode set
    edges = [
        e
        for e in scene_graph.graph.to_edgelist()
        if (e[1] not in exclude_nodes) and (e[0] in sub_nodes or e[1] in sub_nodes)
    ]

    # add include tags from object subtree and attach to root
    if len(include_obj_dict) > 0:
        # find all nodes with names matching elements in include
        # add node names to this object's scene graph list
        incl_node_names_dict = get_nodes_that_match_set(
            transforms_tree, include_obj_dict, allOrSuccessor=False, debug=False
        )

        for obj_parent_node, node_name_set in incl_node_names_dict.items():
            # Get rid of anynode names that should be excluded
            node_name_set -= exclude_nodes
            # include object's nodes
            include_subnodes = set()
            for node_name in node_name_set:
                if recurse_subnodes:
                    new_set = get_node_set_recurse(
                        transforms_tree, exclude_nodes, node_name
                    )
                else:
                    new_set = set(transforms_tree.children_dict[node_name])
                new_set.add(node_name)
                include_subnodes.update(new_set)

            include_subnodes -= exclude_nodes

            # need to build edges from node to included subnodes
            # first build list of child edges, these will not be changed
            include_edges_succ = [
                e
                for e in scene_graph.graph.to_edgelist()
                if e[0] in include_subnodes and e[1] not in exclude_nodes
            ]

            edges.extend(include_edges_succ)
            # now get parent edges, where with each parent edge, we
            # change parent node to be scene_object_tag if parent node was
            # included object's old parent
            include_edges_reparent = [
                (scene_object_tag if e[0] == obj_parent_node else e[0], e[1], e[2])
                for e in scene_graph.graph.to_edgelist()
                if e[0] not in exclude_nodes and e[1] in include_subnodes
            ]
            edges.extend(include_edges_reparent)

    # show_edges_geometry(f"{scene_object_tag} After include edges", edges)

    # build a transformation graqh representing all the components of the object
    g = trimesh.scene.transforms.SceneGraph(base_frame=parent_node_name)
    # add all the edges from the source
    g.from_edgelist(edges)

    # build geometry from scene_graph that is part of this object,
    # restricted to edges that point to geometry nodes
    geometry_names = (e[2]["geometry"] for e in edges if "geometry" in e[2])
    geometry = {k: scene_graph.geometry[k] for k in geometry_names}

    # build a new trimesh scene with constructed transformation graph and geometry
    # representing the named object
    new_scene = trimesh.Scene(geometry=geometry, graph=g)
    # force scene_graph root to be identiy transformation
    new_scene.graph.update(
        scene_object_tag, frame_from=parent_node_name, matrix=np.identity(4)
    )

    assert (
        new_scene.is_valid
    ), f"Constructed {scene_object_tag} scene object is not valid!"
    return new_scene


def get_node_set_recurse(transforms_tree, exclude_nodes, root_node):
    """This function will recurse through the transformation graph of the Trimesh Scene
    and build a set containing the entire subtree of the passed root_node
    """
    # return all successors in a dictionary/tree structure
    node_res = set(transforms_tree.children_dict[root_node])
    tmp_set = set()
    for n in node_res:
        # if n is in exclude then we want to prune entire subtree
        if n not in exclude_nodes:
            tmp_set.add(n)
            if len(transforms_tree.children_dict[n]) > 0:
                new_set = get_node_set_recurse(transforms_tree, exclude_nodes, n)
                tmp_set.update(new_set)
    if len(tmp_set) > 0:
        node_res.update(tmp_set)
    return node_res


def get_node_dict_recurse(transforms_tree, root_node):
    """This function will recurse through the transformation graph of the Trimesh Scene
    and build a dictionary of dictionaries containing the subtree of the root_node
    """

    nodes_present = transforms_tree.children_dict[root_node]
    res_dict = {}
    for n in nodes_present:
        if len(transforms_tree.children_dict[n]) > 0:
            res_dict[n] = get_node_dict_recurse(transforms_tree, n)
        else:
            res_dict[n] = transforms_tree.children_dict[n]

    return res_dict


def print_node_dict_recurse(res_dict: Dict[str, Any], tab_space: str):
    for k, v in res_dict.items():
        if isinstance(v, dict):
            print(f"{tab_space}{k} :")
            print_node_dict_recurse(v, tab_space + "\t")
        else:
            print(f"{tab_space}{k} : {v}")


def extract_ligthing_from_scenegraph(
    scene_graph, lighting_tag: str, parent_node_name: str
):
    """This function will find the lighting nodes in the provided scene
    graph and will build an
    Habitat's Scene Dataset lighting.
    :param scene_graph: The trimesh Scene object
    :param scene_object_tag: The name of the object to look for in the
    Scene graph
    :param parent_node_name: The name of the node in the scene graph
    that is the parent of scene_object_tag node
    """

    transforms_tree = scene_graph.graph.transforms
    sub_nodes = get_node_set_recurse(transforms_tree, {}, lighting_tag)
    sub_nodes.add(lighting_tag)
    sub_nodes.add(parent_node_name)
    # build list of edges that are part of the named object
    # by checking to see if either vert of edge is part of the subnode set
    edges = [
        e
        for e in scene_graph.graph.to_edgelist()
        if e[0] in sub_nodes or e[1] in sub_nodes
    ]

    show_edges_geometry("Lighting edges:", edges)

    print("\n\n")

    res_dict = {}
    for k, v in transforms_tree.children_dict.items():
        if k in sub_nodes:
            if len(v) == 0:
                continue
            print(f"k : {k} | size of map : {len(v)}")
            res_dict1 = {}
            for k1, v1 in v.items():
                res_dict2 = {}
                if k1 in sub_nodes:
                    print(f"\tk1 : {k1} | size of map : {len(v1)}")
                    for k2, v2 in v1.items():
                        if "matrix" in k2.lower():
                            print(f"\t\tk2 : {k2} | node : {v2}")
                            res_dict2[k2] = v2
                    if len(res_dict2) > 0:
                        res_dict1[k1] = res_dict2
            if len(res_dict1) > 0:
                res_dict[k] = res_dict1

    res_dict = {lighting_tag: get_node_dict_recurse(transforms_tree, lighting_tag)}
    print("\nnode dict recurse:\n")
    print_node_dict_recurse(res_dict, "")
    print("\nnode dict recurse done\n")
    # base_edge = [e for e in lighting_edges if e[1] in lighting_tag][0]
    # separate into primary and secondary lighting edges
    # primary lighting edges have the lighting tag as parent node
    # primary_lighting_edges = [e for e in lighting_edges if e[0] in lighting_tag]
    # secondary edges link lighting child nodes to their successors

    # show_edges_geometry("Lighting edges:", lighting_edges)

    return res_dict


def extract_light_from_json(
    nodes_list: List[Dict[str, Any]], lights_list, light_nodes_idxs, level_name: str
):
    """This function will take the listing of node info and the listing of lights from a gltf JSON and
    map light instances to specific light configurations, recursively.

    """
    # check if data dictionary has children nodes - this would indicate a named sub-grouping of lights
    child_res_dict = {}
    for idx in light_nodes_idxs:
        data_dict = nodes_list[idx]
        # check if data dictionary has children nodes;
        # this would indicate a named sub-grouping of lights
        obj_name = data_dict["name"].lower().replace(" ", "_")

        # print(f"{level_name}:{obj_name} : {data_dict}")
        if "children" in data_dict:
            child_idxs = data_dict["children"]
            child_res_dict[obj_name] = extract_light_from_json(
                nodes_list, lights_list, child_idxs, obj_name
            )
        else:
            # is final entry in chain - light, lightprobe, etc
            entry_name = (level_name + "_" + obj_name).lower().replace(" ", "_")
            if "extensions" in data_dict:
                light_idx = data_dict["extensions"]["KHR_lights_punctual"]["light"]
                # json entry for this light
                light_val_dict = lights_list[light_idx].copy()
                for k, v in data_dict.items():
                    if "scale" in k.lower() or "extensions" in k.lower():
                        continue
                    light_val_dict[k] = v
            else:
                # light probe or object that does not map to a light in gltf
                light_val_dict = data_dict.copy()
                light_val_dict["type"] = "unknown"
            del light_val_dict["name"]
            child_res_dict[entry_name] = light_val_dict
    return child_res_dict


def extract_lighting_from_gltf(scene_filename_glb: str, lights_tag: str):

    # Get json from glb file
    # TODO : translation/rotation information for lights to our format of position/direction
    print(
        f"glb_mesh_tools.extract_lighting_from_gltf (Still WIP) : {scene_filename_glb}"
    )
    base_json = extract_json_from_glb(scene_filename_glb)
    # if nothing found, return empty res
    if len(base_json) == 0:
        return {}
    for k, v in base_json.items():
        if hasattr(v, "__len__"):
            print(f"K: {k} : len V : {len(v)}")
        else:
            print(f"K: {k} : V : {v}")
    # list of lights - accessed in scene_graph by idx
    lights_list = base_json["extensions"]["KHR_lights_punctual"]["lights"]
    # print(f"Number of lights: {len(lights_list)}")
    # for light in lights_list :
    #     print(f"{light}")

    # get the list of node idxs of the root of the scene
    scene_root_nodes = base_json["scenes"][base_json["scene"]]["nodes"]

    # print(f"Scene root nodes : {scene_root_nodes}\n")
    nodes_list = base_json["nodes"]
    # find node index list corresponding to children of lighting node
    for idx in scene_root_nodes:
        if lights_tag in nodes_list[idx]["name"]:
            lighting_nodes = nodes_list[idx]["children"]
            break
    # print(f"Light root nodes : {lighting_nodes}\n")

    # iterate through all known idxs to find hierarchy

    # for idx in lighting_nodes:
    #     print(f"{idx} : {nodes_list[idx]}")

    lighting_dict_res = extract_light_from_json(
        nodes_list, lights_list, lighting_nodes, "Lighting"
    )

    return lighting_dict_res


def convert_file_to_unlit(src_glb_file: str, dest_glb_file: str):
    import os

    # first convert glb to gltf in dest dir
    dest_json_file = os.path.splitext(dest_glb_file)[0] + ".json"
    dest_bin_file = os.path.splitext(dest_json_file)[0] + ".bin"

    # convert glb to gltf
    conv_glb_to_gltf(src_glb_file, dest_json_file, dest_bin_file)

    # modify json
    convert_to_unlit(dest_json_file, dest_json_file)

    # convert modified glt back to glb file
    conv_gltf_to_glb(dest_json_file, dest_glb_file)

    # remove json and bin files
    os.remove(dest_json_file)
    os.remove(dest_bin_file)

    # remove


def convert_to_unlit(input_file, output_file):
    """Load and convert a gltf (as json) to be unlit."""
    import json

    with open(input_file) as f:
        json_data = json.load(f)
    # add references to the KHR_materials_unlit extension in
    # gltf root-level tags
    ext_gltf_tags = ["extensionsUsed", "extensionsRequired"]
    for ext_tag in ext_gltf_tags:
        if ext_tag not in json_data:
            json_data[ext_tag] = ["KHR_materials_unlit"]
        elif "KHR_materials_unlit" not in json_data[ext_tag]:
            json_data[ext_tag].append("KHR_materials_unlit")

    for material in json_data["materials"]:
        assert "pbrMetallicRoughness" in material

        # Drop everything except base color and base color texture
        pbrMetallicRoughness = material["pbrMetallicRoughness"]
        for key in list(pbrMetallicRoughness.keys()):
            if key not in ["baseColorFactor", "baseColorTexture"]:
                del pbrMetallicRoughness[key]
        for key in [
            "normalTexture",
            "occlusionTexture",
            "emissiveFactor",
            "emissiveTexture",
        ]:
            if key in material:
                del material[key]

        # Add the extension
        if "extensions" not in material:
            material["extensions"] = {}
        material["extensions"]["KHR_materials_unlit"] = {}

    with open(output_file, "wb") as of:
        of.write(json.dumps(json_data, indent=2).encode("utf-8"))


def conv_glb_to_gltf(src_glb_file, dest_json_file, dest_bin_file):
    import json
    import os
    import struct

    CHUNK_TYPE_JSON = 0x4E4F534A
    CHUNK_TYPE_BIN = 0x004E4942

    glb_header = struct.Struct("<4sII")
    chunk_header = struct.Struct("<II")
    dest_path = os.path.dirname(dest_json_file)

    def pad_size_32b(size):
        return (4 - size % 4) % 4

    print(f"Converting {src_glb_file} to {dest_json_file} and {dest_bin_file}")

    with open(src_glb_file, "rb") as f:
        data: bytes = f.read()

    # Get header
    header, version, length = glb_header.unpack_from(data)
    assert header == b"glTF", "glTF signature invalid: %s" % data[:4]
    assert version == 2, "glTF version invalid: %d" % version

    # Go through chunks. Assume JSON chunk first, BIN chunk next
    data = data[glb_header.size :]
    chunk_length, chunk_type = chunk_header.unpack_from(data)
    assert chunk_type == CHUNK_TYPE_JSON

    data = data[chunk_header.size :]
    json_data = json.loads(data[:chunk_length].decode("utf-8"))

    data = data[chunk_length:]
    chunk_length, chunk_type = chunk_header.unpack_from(data)
    assert chunk_type == CHUNK_TYPE_BIN
    bin_data = data[chunk_header.size :]

    assert len(json_data["buffers"]) == 1
    json_data["buffers"][0]["uri"] = dest_bin_file

    # Separate images. To make this easy, we expect the images to be stored in
    # a continuous suffix of buffer views.

    image_buffer_views = set()
    earliest_image_buffer_offset = json_data["buffers"][0]["byteLength"]
    for image in json_data["images"]:
        assert "bufferView" in image
        assert "uri" not in image

        # Figure out an extension
        ext = None
        if image["mimeType"] == "image/jpeg":
            ext = ".jpg"
        elif image["mimeType"] == "image/png":
            ext = ".png"
        elif image["mimeType"] == "image/x-basis":
            ext = ".basis"
        elif image["mimeType"] == "image/unknown" and "name" in image:
            ext = os.path.splitext(image["name"])[1]
        assert ext, "Unknown MIME type %s" % image["mimeType"]

        # Err, if an extension is already present in the name, strip it
        if "name" in image and image["name"].endswith(ext):
            ext = ""

        # Remember the earliest buffer view used
        buffer_view_id = image["bufferView"]
        image_buffer_views.add(buffer_view_id)

        # Expect there's just one buffer, containing everything. Remember the
        # earliest offset in that buffer
        buffer_view = json_data["bufferViews"][buffer_view_id]
        assert buffer_view["buffer"] == 0
        earliest_image_buffer_offset = min(
            buffer_view["byteOffset"], earliest_image_buffer_offset
        )

        # Save the image data. If the image doesn't have a name, pick the glb
        # filename (and assume there's just one image)
        if "name" not in image:
            assert len(json_data["images"]) == 1
            image_out = os.path.splitext(os.path.basename(src_glb_file))[0] + ext
        else:
            image_out = image["name"] + ext
        image_out = os.path.join(dest_path, image_out)
        # print("Extracting", image_out)
        with open(image_out, "wb") as imf:
            imf.write(
                bin_data[
                    buffer_view["byteOffset"] : buffer_view["byteOffset"]
                    + buffer_view["byteLength"]
                ]
            )

        # Replace the buffer view reference with a file URI
        del image["bufferView"]
        image["uri"] = image_out

    # Check that all buffer views before the first image one are before the
    # first offset as well. I doubt the views will overlap
    for i in range(list(image_buffer_views)[0]):
        assert (
            json_data["bufferViews"][i]["byteOffset"]
            + json_data["bufferViews"][i]["byteLength"]
            <= earliest_image_buffer_offset
        )

    # Remove image-related buffer views, move back everything after
    original_offset = 0
    current_id = 0
    original_bin_data = bin_data
    bin_data = bytearray()
    for i, view in enumerate(json_data["bufferViews"]):
        # A view that we keep, put it to the new bin data
        if i not in image_buffer_views:
            # TODO: This will probably mess up with alignment when there are
            #   data that are not multiples of 4 bytes
            original_offset = view["byteOffset"]
            view["byteOffset"] = len(bin_data)
            bin_data += original_bin_data[
                original_offset : original_offset + view["byteLength"]
            ]

            current_id += 1

        # A view that we remove, update subsequent IDs in all accessors
        else:
            for a in json_data["accessors"]:
                if a["bufferView"] > current_id:
                    a["bufferView"] -= 1

    # Remove now unused views
    json_data["bufferViews"] = [
        view
        for i, view in enumerate(json_data["bufferViews"])
        if i not in image_buffer_views
    ]

    # Update with new bin data size
    json_data["buffers"][0]["byteLength"] = len(bin_data)

    with open(dest_json_file, "wb") as of:
        of.write(json.dumps(json_data, indent=2).encode("utf-8"))

    with open(dest_bin_file, "wb") as bf:
        bf.write(bin_data)


def conv_gltf_to_glb(src_gltf_file, dest_glb_file):
    import base64
    import json
    import os
    import struct

    CHUNK_TYPE_JSON = 0x4E4F534A
    CHUNK_TYPE_BIN = 0x004E4942

    glb_header = struct.Struct("<4sII")
    chunk_header = struct.Struct("<II")

    def pad_size_32b(size):
        return (4 - size % 4) % 4

    print(f"Converting {src_gltf_file} to {dest_glb_file}")

    with open(src_gltf_file) as f:
        data = json.load(f)

    bin_data = bytearray()

    if "buffers" in data:
        assert len(data["buffers"]) <= 1
        if data["buffers"]:
            uri = data["buffers"][0]["uri"]
            if uri[:5] == "data:":
                d = base64.b64decode(uri.split("base64,")[1])
            else:
                with open(uri, "rb") as bf:
                    d = bf.read()
            bin_data.extend(d)

    assert "buffers" in data

    for image in data["images"]:
        assert "bufferView" not in image
        assert "uri" in image

        # Set image name and mime type if not already present, so we have
        # something to extract the file name / extension from in glb2gltf
        basename, ext = os.path.splitext(image["uri"])
        if "mimeType" not in image:
            if ext == ".jpg":
                image["mimeType"] = "image/jpeg"
            elif ext == ".png":
                image["mimeType"] = "image/png"
            elif ext == ".basis":  # noqa: SIM106
                image["mimeType"] = "image/x-basis"
            else:
                raise AssertionError("Unknown image file extension %s" % ext)
        if "name" not in image:
            image["name"] = basename

        # Load the image data, put it into a new buffer view
        # print("Bundling", image["uri"])
        with open(image["uri"], "rb") as imf:
            image_data = imf.read()
        data["bufferViews"] += [
            {
                "buffer": 0,
                "byteOffset": len(bin_data),
                "byteLength": len(image_data),
            }
        ]
        bin_data += image_data
        del image["uri"]
        image["bufferView"] = len(data["bufferViews"]) - 1

    # Pad the buffer, update its length
    if "buffers" in data and data["buffers"]:
        bin_data.extend(b" " * pad_size_32b(len(bin_data)))

        del data["buffers"][0]["uri"]
        data["buffers"][0]["byteLength"] = len(bin_data)

    json_data = json.dumps(data, separators=(",", ":")).encode("utf-8")
    # Append padding bytes so that BIN chunk is aligned to 4 bytes
    json_chunk_align = pad_size_32b(len(json_data))
    json_chunk_length = len(json_data) + json_chunk_align

    with open(dest_glb_file, "wb") as outfile:
        length = glb_header.size + chunk_header.size + json_chunk_length
        if bin_data:
            length += chunk_header.size + len(bin_data)

        # Write header
        outfile.write(glb_header.pack(b"glTF", 2, length))

        # Write JSON chunk
        outfile.write(chunk_header.pack(json_chunk_length, CHUNK_TYPE_JSON))
        outfile.write(json_data)
        outfile.write(b" " * json_chunk_align)

        # Write BIN chunk
        if bin_data:
            outfile.write(chunk_header.pack(len(bin_data), CHUNK_TYPE_BIN))
            outfile.write(bin_data)


def save_glb_as_unlit_old(src_filename_glb: str, dest_filename_glb: str):
    """This will modify the passed glb file so that it will be considered unlit,
    and return the modified json.  Assumes object has a PBR material"""
    try:
        from pygltflib.utils import GLTF2, BufferFormat
    except ImportError:
        print(
            "set_glb_as_unlit : pygltflib must be installed to set glbs to unlit. Aborting"
        )
        return False
    # Need to process json
    import json

    # load GLB, save as gltf
    glb_file = GLTF2().load_binary(src_filename_glb)
    tmp_gltf_filename = dest_filename_glb + ".gltf"
    glb_file.convert_buffers(BufferFormat.BINARYBLOB)
    glb_file.save(tmp_gltf_filename)

    # load gltf JSON file
    gltf = GLTF2().load(tmp_gltf_filename)
    # convert buffers to be able to save
    gltf.convert_buffers(BufferFormat.BINARYBLOB)

    # get JSON data as dict
    base_data_dict = json.loads(gltf.gltf_to_json())

    print(f"Json from glb source {src_filename_glb} : \n{base_data_dict}")

    for material in base_data_dict["materials"]:
        if "pbrMetallicRoughness" in material:
            # Drop everything except base color and base color texture
            pbrMetallicRoughness = material["pbrMetallicRoughness"]
            for key in list(pbrMetallicRoughness.keys()):
                print(f"Key : {key}")
                if key not in ["baseColorFactor", "baseColorTexture"]:
                    del pbrMetallicRoughness[key]
            for k, v in pbrMetallicRoughness.items():
                print(f"Key {k} : Val {v}")

        for key in [
            "normalTexture",
            "occlusionTexture",
            "emissiveFactor",
            "emissiveTexture",
        ]:
            if key in material:
                del material[key]

        # Add the extension
        if "extensions" not in material:
            material["extensions"] = {}
        material["extensions"]["KHR_materials_unlit"] = {}

    print(
        f"New data dict {type(base_data_dict)} :\n{ut.dict_to_disp_string(base_data_dict, '')}"
    )

    gltf = gltf.from_json(json.dumps(base_data_dict), infer_missing=False)

    print(
        f"After editing JSON info for glb to be saved at {dest_filename_glb} : \n{ gltf.gltf_to_json()}"
    )

    success = gltf.save_binary(dest_filename_glb)

    return success
