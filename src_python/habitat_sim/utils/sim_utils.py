#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


import os
from collections import defaultdict
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Set, Union

import magnum as mn
import numpy as np

import habitat_sim
from habitat_sim import physics as HSim_Phys
from habitat_sim.physics import MotionType as HSim_MT


def get_link_normalized_joint_position(
    object_a: HSim_Phys.ManagedArticulatedObject, link_ix: int
) -> float:
    """
    Normalize the joint limit range [min, max] -> [0,1] and return the current joint state in this range.

    :param object_a: The parent ArticulatedObject of the link.
    :param link_ix: The index of the link within the parent object. Not the link's object_id.

    :return: normalized joint position [0,1]
    """

    assert object_a.get_link_joint_type(link_ix) in [
        HSim_Phys.JointType.Revolute,
        HSim_Phys.JointType.Prismatic,
    ], f"Invalid joint type '{object_a.get_link_joint_type(link_ix)}'. Open/closed not a valid check for multi-dimensional or fixed joints."

    joint_pos_ix = object_a.get_link_joint_pos_offset(link_ix)
    joint_pos = object_a.joint_positions[joint_pos_ix]
    limits = object_a.joint_position_limits

    # compute the normalized position [0,1]
    n_pos = (joint_pos - limits[0][joint_pos_ix]) / (
        limits[1][joint_pos_ix] - limits[0][joint_pos_ix]
    )
    return n_pos


def set_link_normalized_joint_position(
    object_a: HSim_Phys.ManagedArticulatedObject,
    link_ix: int,
    normalized_pos: float,
) -> None:
    """
    Set the joint's state within its limits from a normalized range [0,1] -> [min, max]

    Assumes the joint has valid joint limits.

    :param object_a: The parent ArticulatedObject of the link.
    :param link_ix: The index of the link within the parent object. Not the link's object_id.
    :param normalized_pos: The normalized position [0,1] to set.
    """

    assert object_a.get_link_joint_type(link_ix) in [
        HSim_Phys.JointType.Revolute,
        HSim_Phys.JointType.Prismatic,
    ], f"Invalid joint type '{object_a.get_link_joint_type(link_ix)}'. Open/closed not a valid check for multi-dimensional or fixed joints."

    assert (
        normalized_pos <= 1.0 and normalized_pos >= 0
    ), "values outside the range [0,1] are by definition beyond the joint limits."

    joint_pos_ix = object_a.get_link_joint_pos_offset(link_ix)
    limits = object_a.joint_position_limits
    joint_positions = object_a.joint_positions
    joint_positions[joint_pos_ix] = limits[0][joint_pos_ix] + (
        normalized_pos * (limits[1][joint_pos_ix] - limits[0][joint_pos_ix])
    )
    object_a.joint_positions = joint_positions


def link_is_open(
    object_a: HSim_Phys.ManagedArticulatedObject,
    link_ix: int,
    threshold: float = 0.4,
) -> bool:
    """
    Check whether a particular AO link is in the "open" state.
    We assume that joint limits define the closed state (min) and open state (max).

    :param object_a: The parent ArticulatedObject of the link to check.
    :param link_ix: The index of the link within the parent object. Not the link's object_id.
    :param threshold: The normalized threshold ratio of joint ranges which are considered "open". E.g. 0.8 = 80%

    :return: Whether or not the link is considered "open".
    """

    return get_link_normalized_joint_position(object_a, link_ix) >= threshold


def link_is_closed(
    object_a: HSim_Phys.ManagedArticulatedObject,
    link_ix: int,
    threshold: float = 0.1,
) -> bool:
    """
    Check whether a particular AO link is in the "closed" state.
    We assume that joint limits define the closed state (min) and open state (max).

    :param object_a: The parent ArticulatedObject of the link to check.
    :param link_ix: The index of the link within the parent object. Not the link's object_id.
    :param threshold: The normalized threshold ratio of joint ranges which are considered "closed". E.g. 0.1 = 10%

    :return: Whether or not the link is considered "closed".
    """

    return get_link_normalized_joint_position(object_a, link_ix) <= threshold


def close_link(object_a: HSim_Phys.ManagedArticulatedObject, link_ix: int) -> None:
    """
    Set a link to the "closed" state. Sets the joint position to the minimum joint limit.

    TODO: does not do any collision checking to validate the state or move any other objects which may be contained in or supported by this link.

    :param object_a: The parent ArticulatedObject of the link to check.
    :param link_ix: The index of the link within the parent object. Not the link's object_id.
    """

    set_link_normalized_joint_position(object_a, link_ix, 0)


def open_link(object_a: HSim_Phys.ManagedArticulatedObject, link_ix: int) -> None:
    """
    Set a link to the "open" state. Sets the joint position to the maximum joint limit.

    TODO: does not do any collision checking to validate the state or move any other objects which may be contained in or supported by this link.

    :param object_a: The parent ArticulatedObject of the link to check.
    :param link_ix: The index of the link within the parent object. Not the link's object_id.
    """

    set_link_normalized_joint_position(object_a, link_ix, 1.0)


def get_obj_from_id(
    sim: habitat_sim.Simulator,
    obj_id: int,
    ao_link_map: Optional[Dict[int, int]] = None,
) -> Union[HSim_Phys.ManagedRigidObject, HSim_Phys.ManagedArticulatedObject,]:
    """
    Get a ManagedRigidObject or ManagedArticulatedObject from an object_id.

    ArticulatedLink object_ids will return the ManagedArticulatedObject.
    If you want link id, use ManagedArticulatedObject.link_object_ids[obj_id].

    :param sim: The Simulator instance.
    :param obj_id: object id for which ManagedObject is desired.
    :param ao_link_map: A pre-computed map from link object ids to their parent ArticulatedObject's object id.

    :return: a ManagedObject or None
    """

    rom = sim.get_rigid_object_manager()
    if rom.get_library_has_id(obj_id):
        return rom.get_object_by_id(obj_id)

    if ao_link_map is None:
        # Note: better to pre-compute this and pass it around
        ao_link_map = get_ao_link_id_map(sim)

    aom = sim.get_articulated_object_manager()
    if obj_id in ao_link_map:
        return aom.get_object_by_id(ao_link_map[obj_id])

    return None


def get_ao_link_id_map(sim: habitat_sim.Simulator) -> Dict[int, int]:
    """
    Construct a dict mapping ArticulatedLink object_id to parent ArticulatedObject object_id.
    NOTE: also maps ao's root object id to itself for ease of use.

    :param sim: The Simulator instance.

    :return: dict mapping ArticulatedLink object ids to parent object ids.
    """

    aom = sim.get_articulated_object_manager()
    ao_link_map: Dict[int, int] = {}
    for ao in aom.get_objects_by_handle_substring().values():
        # add the ao itself for ease of use
        ao_link_map[ao.object_id] = ao.object_id
        # add the links
        for link_id in ao.link_object_ids:
            ao_link_map[link_id] = ao.object_id

    return ao_link_map


def get_obj_from_handle(
    sim: habitat_sim.Simulator, obj_handle: str
) -> Union[HSim_Phys.ManagedRigidObject, HSim_Phys.ManagedArticulatedObject,]:
    """
    Get a ManagedRigidObject or ManagedArticulatedObject from its instance handle.

    :param sim: The Simulator instance.
    :param obj_handle: object instance handle for which ManagedObject is desired.

    :return: a ManagedObject or None
    """

    rom = sim.get_rigid_object_manager()
    if rom.get_library_has_handle(obj_handle):
        return rom.get_object_by_handle(obj_handle)
    aom = sim.get_articulated_object_manager()
    if aom.get_library_has_handle(obj_handle):
        return aom.get_object_by_handle(obj_handle)

    return None


def get_all_ao_objects(
    sim: habitat_sim.Simulator,
) -> List[HSim_Phys.ManagedArticulatedObject]:
    """
    Get a list of all ManagedArticulatedObjects in the scene.

    :param sim: The Simulator instance.

    :return: a list of ManagedObject wrapper instances containing all articulated objects currently instantiated in the scene.
    """
    return (
        sim.get_articulated_object_manager().get_objects_by_handle_substring().values()
    )


def get_all_rigid_objects(
    sim: habitat_sim.Simulator,
) -> List[HSim_Phys.ManagedArticulatedObject]:
    """
    Get a list of all ManagedRigidObjects in the scene.

    :param sim: The Simulator instance.

    :return: a list of ManagedObject wrapper instances containing all rigid objects currently instantiated in the scene.
    """
    return sim.get_rigid_object_manager().get_objects_by_handle_substring().values()


def get_all_objects(
    sim: habitat_sim.Simulator,
) -> List[Union[HSim_Phys.ManagedRigidObject, HSim_Phys.ManagedArticulatedObject,]]:
    """
    Get a list of all ManagedRigidObjects and ManagedArticulatedObjects in the scene.

    :param sim: The Simulator instance.

    :return: a list of ManagedObject wrapper instances containing all objects currently instantiated in the scene.
    """

    managers = [
        sim.get_rigid_object_manager(),
        sim.get_articulated_object_manager(),
    ]
    all_objects = []
    for mngr in managers:
        all_objects.extend(mngr.get_objects_by_handle_substring().values())
    return all_objects


def get_bb_corners(range3d: mn.Range3D) -> List[mn.Vector3]:
    """
    Return a list of AABB (Range3D) corners in object local space.
    """
    return [
        range3d.back_bottom_left,
        range3d.back_bottom_right,
        range3d.back_top_right,
        range3d.back_top_left,
        range3d.front_top_left,
        range3d.front_top_right,
        range3d.front_bottom_right,
        range3d.front_bottom_left,
    ]


def get_ao_root_bb(
    ao: HSim_Phys.ManagedArticulatedObject,
) -> mn.Range3D:
    """
    Get the local bounding box of all links of an articulated object in the root frame.

    :param ao: The ArticulatedObject instance.
    """

    # NOTE: we'd like to use SceneNode AABB, but this won't work because the links are not in the subtree of the root:
    # ao.root_scene_node.compute_cumulative_bb()

    ao_local_part_bb_corners = []

    link_nodes = [ao.get_link_scene_node(ix) for ix in range(-1, ao.num_links)]
    for link_node in link_nodes:
        local_bb_corners = get_bb_corners(link_node.cumulative_bb)
        global_bb_corners = [
            link_node.absolute_transformation().transform_point(bb_corner)
            for bb_corner in local_bb_corners
        ]
        ao_local_bb_corners = [
            ao.transformation.inverted().transform_point(p) for p in global_bb_corners
        ]
        ao_local_part_bb_corners.extend(ao_local_bb_corners)

    # get min and max of each dimension
    # TODO: use numpy arrays for more elegance...
    max_vec = mn.Vector3(ao_local_part_bb_corners[0])
    min_vec = mn.Vector3(ao_local_part_bb_corners[0])
    for point in ao_local_part_bb_corners:
        for dim in range(3):
            max_vec[dim] = max(max_vec[dim], point[dim])
            min_vec[dim] = min(min_vec[dim], point[dim])
    return mn.Range3D(min_vec, max_vec)


class MarkerSetsInfo:
    def __init__(
        self, sim: habitat_sim.simulator.Simulator, task_names_set: Set = None
    ):
        # Handle default being none
        if task_names_set is None:
            task_names_set = set()
        self.sim = sim
        self.marker_sets_per_obj = self.get_all_markersets()

        # initialize list of possible taskSet names along with those being added specifically
        for _obj_handle, MarkerSet in self.marker_sets_per_obj.items():
            task_names_set.update(MarkerSet.get_all_taskset_names())

        # Necessary class-level variables.
        self.markerset_taskset_names = list(task_names_set)
        self.current_markerset_taskset_idx = 0

        self.marker_sets_changed = {}
        self.marker_debug_random_colors: Dict[str, Any] = {}
        for key in self.marker_sets_per_obj:
            self.marker_sets_changed[key] = False
            self.marker_debug_random_colors[key] = {}

        # for debugging
        self.glbl_marker_point_dicts_per_obj = self.get_all_global_markers()

    def get_current_markerset_taskname(self):
        """
        Retrieve the name of the currently used markerset taskname, as specified in the current object's markersets
        """
        return self.markerset_taskset_names[self.current_markerset_taskset_idx]

    def get_all_markersets(self):
        """
        Get all the markersets defined in the currently loaded objects and articulated objects.
        Note : any modified/saved markersets may require their owning Configurations
        to be reloaded before this function would expose them.
        """
        print("Start getting all markersets")
        # marker set cache of existing markersets for all objs in scene, keyed by object name
        marker_sets_per_obj = {}
        rom = self.sim.get_rigid_object_manager()
        obj_dict = rom.get_objects_by_handle_substring("")
        for handle, obj in obj_dict.items():
            print(f"setting rigid markersets for {handle}")
            marker_sets_per_obj[handle] = obj.marker_sets
        aom = self.sim.get_articulated_object_manager()
        ao_obj_dict = aom.get_objects_by_handle_substring("")
        for handle, obj in ao_obj_dict.items():
            print(f"setting ao markersets for {handle}")
            marker_sets_per_obj[handle] = obj.marker_sets
        print("Done getting all markersets")

        return marker_sets_per_obj

    def place_marker_at_hit_location(
        self, hit_info, ao_link_map: Dict[int, int], add_marker: bool
    ):
        selected_object = None

        # object or ao at hit location. If none, hit stage
        obj = get_obj_from_id(self.sim, hit_info.object_id, ao_link_map)
        print(
            f"Marker : Mouse click object : {hit_info.object_id} : Point : {hit_info.point} "
        )
        # TODO these values need to be modifiable
        task_set_name = self.get_current_markerset_taskname()
        marker_set_name = "faucet_000"
        if obj is None:
            print(
                f"Currently can't add a marker to the stage : ID : ({hit_info.object_id})."
            )
            # TODO get stage's marker_sets properly
            obj_marker_sets = None
            obj_handle = "stage"
            link_ix = -1
            link_name = "root"
            # TODO need to support stage properly including root transformation
            local_hit_point = hit_info.point
        else:
            selected_object = obj
            # get a reference to the object/ao 's markerSets
            obj_marker_sets = obj.marker_sets
            obj_handle = obj.handle
            if obj.is_articulated:
                obj_type = "articulated object"
                # this is an ArticulatedLink, so we can add markers'
                link_ix = obj.link_object_ids[hit_info.object_id]
                link_name = obj.get_link_name(link_ix)

            else:
                obj_type = "rigid object"
                # this is an ArticulatedLink, so we can add markers'
                link_ix = -1
                link_name = "root"

            local_hit_point_list = obj.transform_world_pts_to_local(
                [hit_info.point], link_ix
            )
            # get location in local space
            local_hit_point = local_hit_point_list[0]

            print(
                f"Marker on this {obj_type} : {obj_handle} link Idx : {link_ix} : link name : {link_name} world point : {hit_info.point} local_hit_point : {local_hit_point}"
            )

            if obj_marker_sets is not None:
                # add marker if left button clicked
                if add_marker:
                    # if the desired hierarchy does not exist, create it
                    if not obj_marker_sets.has_task_link_markerset(
                        task_set_name, link_name, marker_set_name
                    ):
                        obj_marker_sets.init_task_link_markerset(
                            task_set_name, link_name, marker_set_name
                        )
                    # get points for current task_set ("faucets"), link_name, marker_set_name ("faucet_000")
                    curr_markers = obj_marker_sets.get_task_link_markerset_points(
                        task_set_name, link_name, marker_set_name
                    )
                    # add point to list
                    curr_markers.append(local_hit_point)
                    # save list
                    obj_marker_sets.set_task_link_markerset_points(
                        task_set_name, link_name, marker_set_name, curr_markers
                    )
                else:
                    # right click is remove marker
                    print(f"About to check obj {obj_handle} if it has any markersets")
                    if obj_marker_sets.has_task_link_markerset(
                        task_set_name, link_name, marker_set_name
                    ):
                        # Non-empty markerset so find closest point to target and delete
                        curr_markers = obj_marker_sets.get_task_link_markerset_points(
                            task_set_name, link_name, marker_set_name
                        )
                        # go through every point to find closest
                        closest_marker_index = None
                        closest_marker_dist = 999999
                        for m_idx in range(len(curr_markers)):
                            m_dist = (local_hit_point - curr_markers[m_idx]).length()
                            if m_dist < closest_marker_dist:
                                closest_marker_dist = m_dist
                                closest_marker_index = m_idx
                        if closest_marker_index is not None:
                            del curr_markers[closest_marker_index]
                        # save new list
                        obj_marker_sets.set_task_link_markerset_points(
                            task_set_name, link_name, marker_set_name, curr_markers
                        )
                    else:
                        print(
                            f"There are no points in MarkerSet : {marker_set_name}, LinkSet :{link_name}, TaskSet :{task_set_name} so removal aborted."
                        )
            self.marker_sets_per_obj[obj_handle] = obj_marker_sets
            self.marker_sets_changed[obj_handle] = True
            self.save_markerset_attributes(obj)
        return selected_object

    def draw_marker_sets_debug(
        self, debug_line_render: Any, camera_position: mn.Vector3
    ) -> None:
        """
        Draw the global state of all configured marker sets.
        """
        for obj_handle, obj_markerset in self.marker_sets_per_obj.items():
            marker_points_dict = obj_markerset.get_all_marker_points()
            if obj_markerset.num_tasksets > 0:
                obj = get_obj_from_handle(self.sim, obj_handle)
                for task_name, task_set_dict in marker_points_dict.items():
                    if task_name not in self.marker_debug_random_colors[obj_handle]:
                        self.marker_debug_random_colors[obj_handle][task_name] = {}
                    for link_name, link_set_dict in task_set_dict.items():
                        if (
                            link_name
                            not in self.marker_debug_random_colors[obj_handle][
                                task_name
                            ]
                        ):
                            self.marker_debug_random_colors[obj_handle][task_name][
                                link_name
                            ] = {}
                        if link_name == "root":
                            link_id = -1
                        else:
                            link_id = obj.get_link_id_from_name(link_name)

                        for markerset_name, marker_pts_list in link_set_dict.items():
                            # print(f"markerset_name : {markerset_name} : marker_pts_list : {marker_pts_list} type : {type(marker_pts_list)} : len : {len(marker_pts_list)}")
                            if (
                                markerset_name
                                not in self.marker_debug_random_colors[obj_handle][
                                    task_name
                                ][link_name]
                            ):
                                self.marker_debug_random_colors[obj_handle][task_name][
                                    link_name
                                ][markerset_name] = mn.Color4(
                                    mn.Vector3(np.random.random(3))
                                )
                            marker_set_color = self.marker_debug_random_colors[
                                obj_handle
                            ][task_name][link_name][markerset_name]
                            global_points = obj.transform_local_pts_to_world(
                                marker_pts_list, link_id
                            )
                            for global_marker_pos in global_points:
                                debug_line_render.draw_circle(
                                    translation=global_marker_pos,
                                    radius=0.005,
                                    color=marker_set_color,
                                    normal=camera_position - global_marker_pos,
                                )

    def save_all_dirty_markersets(self) -> None:
        # save config for object handle's markersets
        for obj_handle, is_dirty in self.marker_sets_changed.items():
            if is_dirty:
                obj = get_obj_from_handle(self.sim, obj_handle)
                self.save_markerset_attributes(obj)

    def save_markerset_attributes(self, obj) -> None:
        """
        Modify the attributes for the passed object to include the
        currently edited markersets and save those attributes to disk
        """
        # get the name of the attrs used to initialize the object
        obj_init_attr_handle = obj.creation_attributes.handle

        if obj.is_articulated:
            # save AO config
            attrMgr = self.sim.metadata_mediator.ao_template_manager
        else:
            # save obj config
            attrMgr = self.sim.metadata_mediator.object_template_manager
        # get copy of initialization attributes as they were in manager,
        # unmodified by scene instance values such as scale
        init_attrs = attrMgr.get_template_by_handle(obj_init_attr_handle)
        # put edited subconfig into initial attributes
        markersets = init_attrs.get_marker_sets()
        # manually copying because the markersets type is getting lost from markersets
        edited_marker_sets = self.marker_sets_per_obj[obj.handle]
        for subconfig_key in edited_marker_sets.get_subconfig_keys():
            markersets.save_subconfig(
                subconfig_key, edited_marker_sets.get_subconfig(subconfig_key)
            )

        # reregister template
        attrMgr.register_template(init_attrs, init_attrs.handle, True)
        # save to original location - uses saved location in attributes
        attrMgr.save_template_by_handle(init_attrs.handle, True)
        # clear out dirty flag
        self.marker_sets_changed[obj.handle] = False

    def correct_and_save_object_markersets(self):
        import json

        """
        Debug function used to correct issues with improper markerset loading/saving.
        Should not be needed any longer.
        """

        def save_markerset_as_json(obj_handle, obj, markers_dict, attrMgr):
            # get the name of the attrs used to initialize the object
            obj_init_attr_handle = obj.creation_attributes.handle

            # get copy of initialization attributes as they were in manager,
            # unmodified by scene instance values such as scale
            init_attrs = attrMgr.get_template_by_handle(obj_init_attr_handle)
            filename = init_attrs.handle.replace(
                ".object_config.json", ".markersets.json"
            )

            marker_sets = {}
            new_markerset_dict = {}
            for task, task_dict in markers_dict.items():
                new_task_dict = {}
                for link, link_dict in task_dict.items():
                    new_link_dict = {}
                    tmp_dict = {}
                    for subset, markers_list in link_dict.items():
                        new_markers_dict = {}
                        key = 0
                        for pt in markers_list:
                            key_str = f"{key:03}"
                            new_markers_dict[key_str] = list(pt)
                            key += 1
                        tmp_dict["markers"] = new_markers_dict
                        new_link_dict[subset] = tmp_dict
                    new_task_dict[link] = new_link_dict
                new_markerset_dict[task] = new_task_dict

            marker_sets["marker_sets"] = new_markerset_dict

            with open(filename, "w") as f:
                f.write(json.dumps(marker_sets, indent=2))

        rom = self.sim.get_rigid_object_manager()
        obj_dict = rom.get_objects_by_handle_substring("")
        for handle, obj in obj_dict.items():
            if ":0000" not in handle:
                continue
            obj_com = obj.transform_world_pts_to_local(
                [obj.uncorrected_translation], -1
            )[0]
            markers_dict = obj.marker_sets.get_all_marker_points()
            changed = False
            for _task, task_dict in markers_dict.items():
                for _link, link_dict in task_dict.items():
                    for _subset, markers_list in link_dict.items():
                        for pt in markers_list:
                            pt += obj_com
                            print(f"new point location {pt}")
                            changed = True
            if changed:
                print(
                    f"Obj: {handle} | Location: {obj.translation} | uncorrected COM : {obj.uncorrected_translation} | Correction to be added to point : {obj_com} "
                )
                self.marker_sets_per_obj[handle].set_all_points(markers_dict)

                # save obj config
                attrMgr = self.sim.metadata_mediator.object_template_manager

                save_markerset_as_json(handle, obj, markers_dict, attrMgr)

    def get_all_global_markers(self):
        """
        Debug function. Get all markers in global space, in nested hierarchy
        """

        def get_points_as_global(obj, marker_points_dict):
            new_markerset_dict = {}
            # for every task
            for task_name, task_dict in marker_points_dict.items():
                new_task_dict = {}
                # for every link
                for link_name, link_dict in task_dict.items():
                    if link_name == "root":
                        link_id = -1
                    else:
                        # articulated object
                        link_id = obj.get_link_id_from_name(link_name)
                    new_link_dict = {}
                    # for every markerset
                    for subset, markers_list in link_dict.items():
                        new_markers_list = obj.transform_local_pts_to_world(
                            markers_list, link_id
                        )
                        new_link_dict[subset] = new_markers_list
                    new_task_dict[link_name] = new_link_dict
                new_markerset_dict[task_name] = new_task_dict
            return new_markerset_dict

        # marker set cache of existing markersets for all objs in scene, keyed by object name
        marker_set_global_dicts_per_obj = {}
        rom = self.sim.get_rigid_object_manager()
        obj_dict = rom.get_objects_by_handle_substring("")
        for handle, obj in obj_dict.items():
            marker_set_global_dicts_per_obj[handle] = get_points_as_global(
                obj, obj.marker_sets.get_all_marker_points()
            )
        aom = self.sim.get_articulated_object_manager()
        ao_obj_dict = aom.get_objects_by_handle_substring("")
        for handle, obj in ao_obj_dict.items():
            marker_set_global_dicts_per_obj[handle] = get_points_as_global(
                obj, obj.marker_sets.get_all_marker_points()
            )

        return marker_set_global_dicts_per_obj

    def draw_markersets_glbl_debug(
        self, debug_line_render: Any, camera_position: mn.Vector3
    ) -> None:
        for (
            obj_handle,
            marker_points_dict,
        ) in self.glbl_marker_point_dicts_per_obj.items():
            for task_name, task_set_dict in marker_points_dict.items():
                if task_name not in self.marker_debug_random_colors[obj_handle]:
                    self.marker_debug_random_colors[obj_handle][task_name] = {}
                for link_name, link_set_dict in task_set_dict.items():
                    if (
                        link_name
                        not in self.marker_debug_random_colors[obj_handle][task_name]
                    ):
                        self.marker_debug_random_colors[obj_handle][task_name][
                            link_name
                        ] = {}

                    for markerset_name, global_points in link_set_dict.items():
                        # print(f"markerset_name : {markerset_name} : marker_pts_list : {marker_pts_list} type : {type(marker_pts_list)} : len : {len(marker_pts_list)}")
                        if (
                            markerset_name
                            not in self.marker_debug_random_colors[obj_handle][
                                task_name
                            ][link_name]
                        ):
                            self.marker_debug_random_colors[obj_handle][task_name][
                                link_name
                            ][markerset_name] = mn.Color4(
                                mn.Vector3(np.random.random(3))
                            )
                        marker_set_color = self.marker_debug_random_colors[obj_handle][
                            task_name
                        ][link_name][markerset_name]
                        for global_marker_pos in global_points:
                            debug_line_render.draw_circle(
                                translation=global_marker_pos,
                                radius=0.005,
                                color=marker_set_color,
                                normal=camera_position - global_marker_pos,
                            )


# Class to control editing objects
# Create an instance and then map various keyboard keys/interactive inputs to appropriate functions.
# Be sure to always specify selected object when it changes.
class ObjectEditor:
    # Describe edit type
    class EditMode(Enum):
        MOVE = 0
        ROTATE = 1
        NUM_VALS = 2  # last value

    EDIT_MODE_NAMES = ["Move object", "Rotate object"]

    # Describe edit distance values
    class DistanceMode(Enum):
        TINY = 0
        VERY_SMALL = 1
        SMALL = 2
        MEDIUM = 3
        LARGE = 4
        HUGE = 5
        NUM_VALS = 6

    # distance values in m
    DISTANCE_MODE_VALS = [0.001, 0.01, 0.02, 0.05, 0.1, 0.5]
    # angle value multipliers (in degrees) - multiplied by conversion
    ROTATION_MULT_VALS = [1.0, 10.0, 30.0, 45.0, 60.0, 90.0]
    # 1 radian
    BASE_EDIT_ROT_AMT = np.pi / 180.0

    def __init__(self, sim: habitat_sim.simulator.Simulator):
        self.sim = sim
        # Dict to hold currently selected objects
        self.sel_objs: Dict[int, Any] = {}
        # Complete list of transformations, for undo chaining,
        # keyed by object id, value is before and after transform
        self.obj_transform_edits: Dict[
            int, List[tuple[mn.Matrix4, mn.Matrix4, bool]]
        ] = defaultdict(list)

        # Cache removed objects in dictionary
        # These objects should be hidden/moved out of vieew until the scene is
        # saved, when they should be actually removed (to allow for undo).
        # First key is whether they are articulated or not, value is dict with key == object id, value is object,
        self._removed_objs: Dict[bool, Dict[int, Any]] = defaultdict(dict)
        # Edit mode
        self.curr_edit_mode = ObjectEditor.EditMode.MOVE
        # Edit distance/amount
        self.curr_edit_multiplier = ObjectEditor.DistanceMode.VERY_SMALL
        # Set initial values
        self.set_edit_vals()

    def set_edit_vals(self):
        # Set current scene object edit values for translation and rotation
        # 1 cm * multiplier
        self.edit_translation_dist = ObjectEditor.DISTANCE_MODE_VALS[
            self.curr_edit_multiplier.value
        ]
        # 1 radian * multiplier
        self.edit_rotation_amt = (
            ObjectEditor.BASE_EDIT_ROT_AMT
            * ObjectEditor.ROTATION_MULT_VALS[self.curr_edit_multiplier.value]
        )

    def edit_disp_str(self):
        """
        Specify display quantities for editing
        """

        edit_mode_string = ObjectEditor.EDIT_MODE_NAMES[self.curr_edit_mode.value]

        dist_mode_substr = (
            f"Translation: {self.edit_translation_dist}m"
            if self.curr_edit_mode == ObjectEditor.EditMode.MOVE
            else f"Rotation:{ObjectEditor.ROTATION_MULT_VALS[self.curr_edit_multiplier.value]} deg "
        )
        edit_distance_mode_string = f"{dist_mode_substr}"

        disp_str = f"""Edit Mode: {edit_mode_string}
Edit Value: {edit_distance_mode_string}
Num Sel Objs: {len(self.sel_objs)}
          """
        return disp_str

    def set_sel_obj(self, obj):
        """
        Set the selected objects to just be the passed obj
        """
        self.sel_objs = {}
        if obj is not None:
            self.sel_objs[obj.id] = obj

    def modify_sel_objs(self, obj):
        """
        Add the passed object to the selected objects and cache its original transformation
        """
        if obj.id in self.sel_objs:
            # Remove object from obj selected dict
            self.sel_objs.pop(obj.id, None)

        else:
            # add object to selected dict
            self.sel_objs[obj.id] = obj

    def set_ao_joint_states(
        self, do_open: bool, selected: bool, agent_name: str = "hab_spot"
    ):
        """
        Set either the selected articulated object states to either fully open or fully closed, or all the articulated object states (except for any robots)
        """
        if selected:
            # Only selected objs if they are articulated
            ao_objs = [ao for ao in self.sel_objs.values() if ao.is_articulated]
        else:
            # all articulated bjs that do not contain agent's name
            ao_objs = (
                self.sim.get_articulated_object_manager()
                .get_objects_by_handle_substring(search_str=agent_name, contains=False)
                .values()
            )

        if do_open:
            # Open AOs
            for ao in ao_objs:
                # open the selected receptacle
                for link_ix in ao.get_link_ids():
                    if ao.get_link_joint_type(link_ix) in [
                        HSim_Phys.JointType.Prismatic,
                        HSim_Phys.JointType.Revolute,
                    ]:
                        open_link(ao, link_ix)
        else:
            # Close AOs
            for ao in ao_objs:
                j_pos = ao.joint_positions
                ao.joint_positions = [0.0 for _ in range(len(j_pos))]
                j_vel = ao.joint_velocities
                ao.joint_velocities = [0.0 for _ in range(len(j_vel))]

    def _move_one_object(
        self,
        obj,
        navmesh_dirty: bool,
        translation: Optional[mn.Vector3] = None,
        rotation: Optional[mn.Quaternion] = None,
        removal: bool = False,
    ) -> bool:
        """
        Internal. Move a single object with a given modification and save the resulting state to the buffer.
        Returns whether the navmesh should be rebuilt due to an object changing position, or previous edits.
        """
        if obj is None:
            print("No object is selected so ignoring move request")
            return navmesh_dirty
        if translation is not None or rotation is not None:
            orig_transform = obj.transformation
            orig_mt = obj.motion_type
            obj.motion_type = HSim_MT.KINEMATIC
            action_str = ""
            if translation is not None:
                obj.translation = obj.translation + translation
                action_str = f"translated to {obj.translation};"
            if rotation is not None:
                obj.rotation = rotation * obj.rotation
                action_str = f"{action_str}rotated to {obj.rotation};"
            print(action_str)
            obj.motion_type = orig_mt
            trans_tuple = (orig_transform, obj.transformation, removal)
            self.obj_transform_edits[obj.id].append(trans_tuple)
            return True
        return navmesh_dirty

    def move_sel_objects(
        self,
        navmesh_dirty: bool,
        translation: Optional[mn.Vector3] = None,
        rotation: Optional[mn.Quaternion] = None,
        removal: bool = False,
    ) -> bool:
        """
        Move all selected objects with a given modification and save the resulting state to the buffer.
        Returns whether the navmesh should be rebuilt due to an object's transformation changing, or previous edits.
        """
        for obj in self.sel_objs.values():
            new_navmesh_dirty = self._move_one_object(
                obj,
                navmesh_dirty,
                translation=translation,
                rotation=rotation,
                removal=removal,
            )
            navmesh_dirty = new_navmesh_dirty or navmesh_dirty
        return navmesh_dirty

    def remove_sel_objects(self):
        """
        'Removes' all selected objects from the scene by hiding them and putting them in queue to be deleted on
        scene save update. Returns list of the handles of all the objects removed if successful

        Note : removal is not permanent unless scene is saved.
        """
        removed_obj_handles = []
        for obj in self.sel_objs.values():
            if obj is None:
                continue
            # move selected object outside of render area
            translation = obj.translation + mn.Vector3(0.0, 1000.0, 0.0)
            # ignore navmesh result; removal always recomputes
            self._move_one_object(obj, True, translation=translation, removal=True)
            # record removed object for eventual deletion upon scene save
            self._removed_objs[obj.is_articulated][obj.id] = obj
            # record handle of removed objects, for return
            removed_obj_handles.append(obj.handle)
            print(
                f"Moved {obj.handle} out of view and marked for removal. Removal becomes permanent when scene is saved."
            )
        # unselect all objects, since they were all 'removed'
        self.sel_objs = {}
        # retain all object selected transformations.
        return removed_obj_handles

    def edit_left(self, navmesh_dirty: bool):
        """
        Edit selected objects for left key input
        """
        # if movement mode
        if self.curr_edit_mode == ObjectEditor.EditMode.MOVE:
            return self.move_sel_objects(
                navmesh_dirty=navmesh_dirty,
                translation=mn.Vector3.x_axis() * self.edit_translation_dist,
            )
        # if rotation mode : rotate around y axis
        return self.move_sel_objects(
            navmesh_dirty=navmesh_dirty,
            rotation=mn.Quaternion.rotation(
                mn.Rad(self.edit_rotation_amt), mn.Vector3.y_axis()
            ),
        )

    def edit_right(self, navmesh_dirty: bool):
        """
        Edit selected objects for right key input
        """
        # if movement mode
        if self.curr_edit_mode == ObjectEditor.EditMode.MOVE:
            return self.move_sel_objects(
                navmesh_dirty=navmesh_dirty,
                translation=-mn.Vector3.x_axis() * self.edit_translation_dist,
            )
        # if rotation mode : rotate around y axis
        return self.move_sel_objects(
            navmesh_dirty=navmesh_dirty,
            rotation=mn.Quaternion.rotation(
                -mn.Rad(self.edit_rotation_amt), mn.Vector3.y_axis()
            ),
        )
        return navmesh_dirty

    def edit_up(self, navmesh_dirty: bool, toggle: bool):
        """
        Edit selected objects for up key input
        """
        # if movement mode
        if self.curr_edit_mode == ObjectEditor.EditMode.MOVE:
            trans_axis = mn.Vector3.y_axis() if toggle else mn.Vector3.z_axis()
            return self.move_sel_objects(
                navmesh_dirty=navmesh_dirty,
                translation=trans_axis * self.edit_translation_dist,
            )

        # if rotation mode : rotate around x or z axis
        rot_axis = mn.Vector3.x_axis() if toggle else mn.Vector3.z_axis()
        return self.move_sel_objects(
            navmesh_dirty=navmesh_dirty,
            rotation=mn.Quaternion.rotation(mn.Rad(self.edit_rotation_amt), rot_axis),
        )

    def edit_down(self, navmesh_dirty: bool, toggle: bool):
        """
        Edit selected objects for down key input
        """
        # if movement mode
        if self.curr_edit_mode == ObjectEditor.EditMode.MOVE:
            trans_axis = -mn.Vector3.y_axis() if toggle else -mn.Vector3.z_axis()

            return self.move_sel_objects(
                navmesh_dirty=navmesh_dirty,
                translation=trans_axis * self.edit_translation_dist,
            )
        # if rotation mode : rotate around x or z axis
        rot_axis = mn.Vector3.x_axis() if toggle else mn.Vector3.z_axis()
        return self.move_sel_objects(
            navmesh_dirty=navmesh_dirty,
            rotation=mn.Quaternion.rotation(-mn.Rad(self.edit_rotation_amt), rot_axis),
        )

    def _undo_obj_edit(self, obj, transform_tuple):
        """
        Changes the object's current transformation to the passed, previous transformation (in idx 0)
        """
        old_transform = transform_tuple[0]
        print(
            f"Sel Obj : {obj.handle} : Current object transformation : \n{obj.transformation}\n Being replaced by saved transformation : \n{old_transform}"
        )
        orig_mt = obj.motion_type
        obj.motion_type = HSim_MT.KINEMATIC
        obj.transformation = old_transform
        obj.motion_type = orig_mt

    def undo_sel_edits(self):
        """
        Undo the edits that have been performed on all the currently selected objects one step, including removal marks
        """
        if len(self.sel_objs) == 0:
            return
        # For every object in selected object
        for obj_id, obj in self.sel_objs.items():
            # Verify there are transforms to undo for this object
            if len(self.obj_transform_edits[obj_id]) > 0:
                # Last edit state is last element in transforms list
                # In tuple idxs : 0 : previous transform, 1 : current transform, 2 : whether was a removal op
                # Retrieve and remove last edit
                transform_tuple = self.obj_transform_edits[obj_id].pop()
                self._undo_obj_edit(obj, transform_tuple)
                # If this was a removal, remove object from removal queue
                if transform_tuple[2]:
                    # Remove object from removal queue if there - undo removal
                    self._removed_objs[obj.is_articulated].pop(obj_id, None)

    def save_current_scene(self):
        # update scene with removals before saving
        ao_removed_objs = self._removed_objs[True]
        if len(ao_removed_objs) > 0:
            ao_mgr = self.sim.get_articulated_object_manager()
            for obj_id in ao_removed_objs:
                ao_mgr.remove_object_by_id(obj_id)
                # Get rid of all recorded transforms of specified object
                self.obj_transform_edits.pop(obj_id, None)
        ro_removed_objs = self._removed_objs[False]
        if len(ro_removed_objs) > 0:
            ro_mgr = self.sim.get_rigid_object_manager()
            for obj_id in ro_removed_objs:
                ro_mgr.remove_object_by_id(obj_id)
                # Get rid of all recorded transforms of specified object
                self.obj_transform_edits.pop(obj_id, None)

        # clear out cache of removed objects by resetting dictionary
        self._removed_objs = defaultdict(dict)
        # Reset all AOs to be 0
        self.set_ao_joint_states(do_open=False, selected=False)
        # Save current scene
        self.sim.save_current_scene_config(overwrite=True)

    def recompute_ao_bbs(self, ao: HSim_Phys.ManagedArticulatedObject) -> None:
        """
        Recomputes the link SceneNode bounding boxes for all ao links.
        NOTE: Gets around an observed loading bug. Call before trying to peek an AO.
        """
        for link_ix in range(-1, ao.num_links):
            link_node = ao.get_link_scene_node(link_ix)
            link_node.compute_cumulative_bb()

    def _build_object_from_handle(
        self, obj_temp_handle: str, build_ao: bool, attr_mgr, obj_mgr
    ):
        """
        Given a specific handle and location, build a new rigid or articulated object
        """
        # Build an object using obj_temp_handle, getting template from attr_mgr and object manager obj_mgr
        temp = attr_mgr.get_template_by_handle(obj_temp_handle)

        if build_ao:
            obj_type = "Articulated"
            temp.base_type = "FIXED"
            attr_mgr.register_template(temp)
            new_obj = obj_mgr.add_articulated_object_by_template_handle(obj_temp_handle)
            if new_obj is not None:
                self.recompute_ao_bbs(new_obj)
        else:
            # If any changes to template, put them here and re-register template
            # attr_mgr.register_template(temp)
            obj_type = "Rigid"
            new_obj = obj_mgr.add_object_by_template_handle(obj_temp_handle)
        if new_obj is None:
            print(
                f"Failed to load/create {obj_type} Object from template named {obj_temp_handle}."
            )
        return new_obj

    def build_objects(
        self, navmesh_dirty: bool, load_by_name: bool, build_loc: mn.Vector3
    ):
        """
        Make a copy of the selected object(s), or load a named item at some distance away
        """
        sim = self.sim
        mm = sim.metadata_mediator
        if len(self.sel_objs) > 0:
            # Copy all selected objects
            res_objs = []
            for obj in self.sel_objs.values():
                # if selected, build object at location of new
                obj_temp_handle = obj.creation_attributes.handle
                if obj.is_articulated:
                    # build an ao via template
                    attr_mgr = mm.ao_template_manager
                    obj_mgr = sim.get_articulated_object_manager()
                else:
                    # build a rigid via template
                    attr_mgr = mm.object_template_manager
                    obj_mgr = sim.get_rigid_object_manager()
                new_obj = self._build_object_from_handle(
                    obj_temp_handle, obj.is_articulated, attr_mgr, obj_mgr
                )
                if new_obj is not None:
                    # set new object location to be above location of selected object
                    new_obj_translation = obj.translation + mn.Vector3(0.0, 1.0, 0.0)
                    new_obj.motion_type = obj.motion_type
                    # move new object to appropriate location
                    new_navmesh_dirty = self._move_one_object(
                        obj, navmesh_dirty, new_obj_translation, obj.rotation
                    )
                    navmesh_dirty = new_navmesh_dirty or navmesh_dirty
                    res_objs.append(new_obj)
            # duplicated all currently selected objects
            # clear currently set selected objects
            self.sel_objs = {}
            # Select all new objects
            for new_obj in res_objs:
                # add object to selected objects
                self.modify_sel_objs(new_obj)
            return res_objs, navmesh_dirty

        elif load_by_name:
            # get user input to load a single object
            obj_substring = input(
                "Load Object or AO. Enter a Rigid Object or AO handle substring, first match will be added:"
            ).strip()

            if len(obj_substring) == 0:
                print("No valid name given. Aborting")
                return [], navmesh_dirty

            template_mgr = mm.ao_template_manager
            template_handles = template_mgr.get_template_handles(obj_substring)
            if len(template_handles) == 1:
                # Specific AO template found
                obj_mgr = sim.get_articulated_object_manager()
                base_motion_type = HSim_MT.DYNAMIC
            else:
                template_mgr = mm.object_template_manager
                template_handles = template_mgr.get_template_handles(obj_substring)
                if len(template_handles) != 1:
                    print(
                        f"No distinct Rigid or Articulated Object handle found matching substring: '{obj_substring}'. Aborting"
                    )
                    return [], navmesh_dirty
                # Specific Rigid template found
                obj_mgr = sim.get_rigid_object_manager()
                base_motion_type = HSim_MT.STATIC

            new_obj = self._build_object_from_handle(
                template_handles[0], False, template_mgr, obj_mgr
            )
            if new_obj is not None:
                # set new object location to be above location of selected object
                new_obj.motion_type = base_motion_type
                self.set_sel_obj(new_obj)
                # move new object to appropriate location
                new_navmesh_dirty = self._move_one_object(
                    new_obj, navmesh_dirty, translation=build_loc
                )
                navmesh_dirty = new_navmesh_dirty or navmesh_dirty
            else:
                # creation failing would have its own message
                return [], navmesh_dirty
            return [new_obj], navmesh_dirty

        # no object selected and no name input
        print(
            "No object was selected to copy and load_by_name was not specifieed so no known object handle was input. Aborting"
        )
        return [], navmesh_dirty

    def change_edit_mode(self, toggle: bool):
        # toggle edit mode
        mod_val = -1 if toggle else 1
        self.curr_edit_mode = ObjectEditor.EditMode(
            (self.curr_edit_mode.value + ObjectEditor.EditMode.NUM_VALS.value + mod_val)
            % ObjectEditor.EditMode.NUM_VALS.value
        )

    def change_edit_vals(self, toggle: bool):
        # cycle through edit dist/amount multiplier
        mod_val = -1 if toggle else 1
        self.curr_edit_multiplier = ObjectEditor.DistanceMode(
            (
                self.curr_edit_multiplier.value
                + ObjectEditor.DistanceMode.NUM_VALS.value
                + mod_val
            )
            % ObjectEditor.DistanceMode.NUM_VALS.value
        )
        # update the edit values
        self.set_edit_vals()

    def _draw_selected_obj(self, obj, debug_line_render):
        """
        Draw a selection box around and axis frame at the origin of a single object
        """
        aabb = get_ao_root_bb(obj) if obj.is_articulated else obj.collision_shape_aabb
        debug_line_render.push_transform(obj.transformation)
        debug_line_render.draw_box(aabb.min, aabb.max, mn.Color4.magenta())
        debug_line_render.pop_transform()

        ot = obj.translation
        # draw global coordinate axis
        debug_line_render.draw_transformed_line(
            ot - mn.Vector3.x_axis(), ot + mn.Vector3.x_axis(), mn.Color4.red()
        )
        debug_line_render.draw_transformed_line(
            ot - mn.Vector3.y_axis(), ot + mn.Vector3.y_axis(), mn.Color4.green()
        )
        debug_line_render.draw_transformed_line(
            ot - mn.Vector3.z_axis(), ot + mn.Vector3.z_axis(), mn.Color4.blue()
        )
        debug_line_render.draw_circle(
            ot + mn.Vector3.x_axis() * 0.95,
            radius=0.05,
            color=mn.Color4.red(),
            normal=mn.Vector3.x_axis(),
        )
        debug_line_render.draw_circle(
            ot + mn.Vector3.y_axis() * 0.95,
            radius=0.05,
            color=mn.Color4.green(),
            normal=mn.Vector3.y_axis(),
        )
        debug_line_render.draw_circle(
            ot + mn.Vector3.z_axis() * 0.95,
            radius=0.05,
            color=mn.Color4.blue(),
            normal=mn.Vector3.z_axis(),
        )

    def draw_selected_objects(self, debug_line_render):
        if len(self.sel_objs) == 0:
            return
        for obj in self.sel_objs.values():
            self._draw_selected_obj(obj, debug_line_render=debug_line_render)


# Class to instantiate and maneuver spot from a viewer
class SpotAgent:
    from habitat.articulated_agents.robots.spot_robot import SpotRobot
    from habitat.datasets.rearrange.navmesh_utils import get_largest_island_index
    from omegaconf import DictConfig

    SPOT_DIR = "data/robots/hab_spot_arm/urdf/hab_spot_arm.urdf"
    if not os.path.isfile(SPOT_DIR):
        # support other layout
        SPOT_DIR = "data/scene_datasets/robots/hab_spot_arm/urdf/hab_spot_arm.urdf"

    class ExtractedBaseVelNonCylinderAction:
        def __init__(self, sim, spot):
            self._sim = sim
            self.spot = spot
            self.base_vel_ctrl = HSim_Phys.VelocityControl()
            self.base_vel_ctrl.controlling_lin_vel = True
            self.base_vel_ctrl.lin_vel_is_local = True
            self.base_vel_ctrl.controlling_ang_vel = True
            self.base_vel_ctrl.ang_vel_is_local = True
            self._allow_dyn_slide = True
            self._allow_back = True
            self._longitudinal_lin_speed = 10.0
            self._lateral_lin_speed = 10.0
            self._ang_speed = 10.0
            self._navmesh_offset = [[0.0, 0.0], [0.25, 0.0], [-0.25, 0.0]]
            self._enable_lateral_move = True
            self._collision_threshold = 1e-5
            self._noclip = False
            # If we just changed from noclip to clip - make sure
            # that if spot is not on the navmesh he gets snapped to it
            self._transition_to_clip = False

        def collision_check(
            self, trans, target_trans, target_rigid_state, compute_sliding
        ):
            """
            trans: the transformation of the current location of the robot
            target_trans: the transformation of the target location of the robot given the center original Navmesh
            target_rigid_state: the target state of the robot given the center original Navmesh
            compute_sliding: if we want to compute sliding or not
            """

            def step_spot(
                num_check_cylinder: int,
                cur_height: float,
                pos_calc: Callable[[np.ndarray, np.ndarray], np.ndarray],
                cur_pos: List[np.ndarray],
                goal_pos: List[np.ndarray],
            ):
                end_pos = []
                for i in range(num_check_cylinder):
                    pos = pos_calc(cur_pos[i], goal_pos[i])
                    # Sanitize the height
                    pos[1] = cur_height
                    cur_pos[i][1] = cur_height
                    goal_pos[i][1] = cur_height
                    end_pos.append(pos)
                return end_pos

            # Get the offset positions
            num_check_cylinder = len(self._navmesh_offset)
            nav_pos_3d = [np.array([xz[0], 0.0, xz[1]]) for xz in self._navmesh_offset]
            cur_pos: List[np.ndarray] = [
                trans.transform_point(xyz) for xyz in nav_pos_3d
            ]
            goal_pos: List[np.ndarray] = [
                target_trans.transform_point(xyz) for xyz in nav_pos_3d
            ]

            # For step filter of offset positions
            end_pos = []

            no_filter_step = lambda _, val: val
            if self._noclip:
                cur_height = self.spot.base_pos[1]
                # ignore navmesh
                end_pos = step_spot(
                    num_check_cylinder=num_check_cylinder,
                    cur_height=cur_height,
                    pos_calc=no_filter_step,
                    cur_pos=cur_pos,
                    goal_pos=goal_pos,
                )
            else:
                cur_height = self._sim.pathfinder.snap_point(self.spot.base_pos)[1]
                # constrain to navmesh
                end_pos = step_spot(
                    num_check_cylinder=num_check_cylinder,
                    cur_height=cur_height,
                    pos_calc=self._sim.step_filter,
                    cur_pos=cur_pos,
                    goal_pos=goal_pos,
                )

            # Planar move distance clamped by NavMesh
            move = []
            for i in range(num_check_cylinder):
                move.append((end_pos[i] - goal_pos[i]).length())

            # For detection of linear or angualr velocities
            # There is a collision if the difference between the clamped NavMesh position and target position is too great for any point.
            diff = len([v for v in move if v > self._collision_threshold])

            if diff > 0:
                # Wrap the move direction if we use sliding
                # Find the largest diff moving direction, which means that there is a collision in that cylinder
                if compute_sliding:
                    max_idx = np.argmax(move)
                    move_vec = end_pos[max_idx] - cur_pos[max_idx]
                    new_end_pos = trans.translation + move_vec
                    return True, mn.Matrix4.from_(
                        target_rigid_state.rotation.to_matrix(), new_end_pos
                    )
                return True, trans
            else:
                return False, target_trans

        def update_base(self, if_rotation):
            """
            Update the base of the robot
            if_rotation: if the robot is rotating or not
            """
            # Get the control frequency
            inv_ctrl_freq = 1.0 / 60.0
            # Get the current transformation
            trans = self.spot.sim_obj.transformation
            # Get the current rigid state
            rigid_state = habitat_sim.RigidState(
                mn.Quaternion.from_matrix(trans.rotation()), trans.translation
            )
            # Integrate to get target rigid state
            target_rigid_state = self.base_vel_ctrl.integrate_transform(
                inv_ctrl_freq, rigid_state
            )
            # Get the traget transformation based on the target rigid state
            target_trans = mn.Matrix4.from_(
                target_rigid_state.rotation.to_matrix(),
                target_rigid_state.translation,
            )
            # We do sliding only if we allow the robot to do sliding and current
            # robot is not rotating
            compute_sliding = self._allow_dyn_slide and not if_rotation
            # Check if there is a collision
            did_coll, new_target_trans = self.collision_check(
                trans, target_trans, target_rigid_state, compute_sliding
            )
            # Update the base
            self.spot.sim_obj.transformation = new_target_trans

            if self.spot._base_type == "leg":
                # Fix the leg joints
                self.spot.leg_joint_pos = self.spot.params.leg_init_params

        def toggle_clip(self, largest_island_ix: int):
            """
            Handle transition to/from no clipping/navmesh disengaged.
            """
            # Transitioning to clip from no clip or vice versa
            self._transition_to_clip = self._noclip
            self._noclip = not self._noclip

            spot_cur_point = self.spot.base_pos
            # Find reasonable location to return to navmesh
            if self._transition_to_clip and not self._sim.pathfinder.is_navigable(
                spot_cur_point
            ):
                # Clear transition flag - only transition once
                self._transition_to_clip = False
                # Find closest point on navmesh to snap spot to
                print(
                    f"Trying to find closest navmesh point to spot_cur_point: {spot_cur_point}"
                )
                new_point = self._sim.pathfinder.snap_point(
                    spot_cur_point, largest_island_ix
                )
                if not np.any(np.isnan(new_point)):
                    print(
                        f"Closest navmesh point to spot_cur_point: {spot_cur_point} is {new_point} on largest island {largest_island_ix}. Snapping to it."
                    )
                    # Move spot to this point
                    self.spot.base_pos = new_point
                else:
                    # try again to any island
                    new_point = self._sim.pathfinder.snap_point(spot_cur_point)
                    if not np.any(np.isnan(new_point)):
                        print(
                            f"Closest navmesh point to spot_cur_point: {spot_cur_point} is {new_point} not on largest island. Snapping to it."
                        )
                        # Move spot to this point
                        self.spot.base_pos = new_point
                    else:
                        print(
                            "Unable to leave no-clip mode, too far from navmesh. Try again when closer."
                        )
                        self._noclip = True
            return self._noclip

        def step(self, forward, lateral, angular):
            """
            provide forward, lateral, and angular velocities as [-1,1].
            """
            longitudinal_lin_vel = forward
            lateral_lin_vel = lateral
            ang_vel = angular
            longitudinal_lin_vel = (
                np.clip(longitudinal_lin_vel, -1, 1) * self._longitudinal_lin_speed
            )
            lateral_lin_vel = np.clip(lateral_lin_vel, -1, 1) * self._lateral_lin_speed
            ang_vel = np.clip(ang_vel, -1, 1) * self._ang_speed
            if not self._allow_back:
                longitudinal_lin_vel = np.maximum(longitudinal_lin_vel, 0)

            self.base_vel_ctrl.linear_velocity = mn.Vector3(
                longitudinal_lin_vel, 0, -lateral_lin_vel
            )
            self.base_vel_ctrl.angular_velocity = mn.Vector3(0, ang_vel, 0)

            if longitudinal_lin_vel != 0.0 or lateral_lin_vel != 0.0 or ang_vel != 0.0:
                self.update_base(ang_vel != 0.0)

    def __init__(self, sim: habitat_sim.Simulator):
        self.sim = sim
        # changed when spot is put on navmesh
        self.largest_island_ix = -1
        self.spot_forward = 0.0
        self.spot_lateral = 0.0
        self.spot_angular = 0.0
        self.load_and_init()
        # angle and azimuth of camera orientation
        self.camera_angles = mn.Vector2()
        self.init_spot_cam()

        self.spot_rigid_state = self.spot.sim_obj.rigid_state
        self.spot_motion_type = self.spot.sim_obj.motion_type

    def load_and_init(self):
        # add the robot to the world via the wrapper
        robot_path = SpotAgent.SPOT_DIR
        agent_config = SpotAgent.DictConfig({"articulated_agent_urdf": robot_path})
        self.spot: SpotAgent.SpotRobot = SpotAgent.SpotRobot(
            agent_config, self.sim, fixed_base=True
        )
        self.spot.reconfigure()
        self.spot.update()
        self.spot_action: SpotAgent.ExtractedBaseVelNonCylinderAction = (
            SpotAgent.ExtractedBaseVelNonCylinderAction(self.sim, self.spot)
        )

    def init_spot_cam(self):
        # Camera relative to spot
        self.camera_distance = 2.0
        # height above spot to target lookat
        self.lookat_height = 0.0

    def mod_spot_cam(
        self,
        scroll_mod_val: float = 0,
        mse_rel_pos: List = None,
        shift_pressed: bool = False,
        alt_pressed: bool = False,
    ):
        """
        Modify the camera agent's orientation, distance and lookat target relative to spot via UI input
        """
        # use shift for fine-grained zooming
        if scroll_mod_val != 0:
            mod_val = 0.3 if shift_pressed else 0.15
            scroll_delta = scroll_mod_val * mod_val
            if alt_pressed:
                # lookat going up and down
                self.lookat_height -= scroll_delta
            else:
                self.camera_distance -= scroll_delta
        if mse_rel_pos is not None:
            self.camera_angles[0] -= mse_rel_pos[1] * 0.01
            self.camera_angles[1] -= mse_rel_pos[0] * 0.01
            self.camera_angles[0] = max(-1.55, min(0.5, self.camera_angles[0]))
            self.camera_angles[1] = np.fmod(self.camera_angles[1], np.pi * 2.0)

    def set_agent_camera_transform(self, agent_node):
        # set camera agent position relative to spot
        x_rot = mn.Quaternion.rotation(
            mn.Rad(self.camera_angles[0]), mn.Vector3(1, 0, 0)
        )
        y_rot = mn.Quaternion.rotation(
            mn.Rad(self.camera_angles[1]), mn.Vector3(0, 1, 0)
        )
        local_camera_vec = mn.Vector3(0, 0, 1)
        local_camera_position = y_rot.transform_vector(
            x_rot.transform_vector(local_camera_vec * self.camera_distance)
        )
        spot_pos = self.base_pos()
        lookat_disp = mn.Vector3(0, self.lookat_height, 0)
        lookat_pos = spot_pos + lookat_disp
        camera_position = local_camera_position + lookat_pos
        agent_node.transformation = mn.Matrix4.look_at(
            camera_position,
            lookat_pos,
            mn.Vector3(0, 1, 0),
        )

    def base_pos(self):
        return self.spot.base_pos

    def place_on_navmesh(self):
        if self.sim.pathfinder.is_loaded:
            self.largest_island_ix = SpotAgent.get_largest_island_index(
                pathfinder=self.sim.pathfinder,
                sim=self.sim,
                allow_outdoor=False,
            )
            print(f"Largest indoor island index = {self.largest_island_ix}")
            valid_spot_point = None
            max_attempts = 1000
            attempt = 0
            while valid_spot_point is None and attempt < max_attempts:
                spot_point = self.sim.pathfinder.get_random_navigable_point(
                    island_index=self.largest_island_ix
                )
                if self.sim.pathfinder.distance_to_closest_obstacle(spot_point) >= 0.25:
                    valid_spot_point = spot_point
                attempt += 1
            if valid_spot_point is not None:
                self.spot.base_pos = valid_spot_point
            else:
                print(
                    f"Unable to find a valid spot for Spot on the navmesh after {max_attempts} attempts"
                )

    def toggle_clip(self):
        # attempt to turn on or off noclip
        clipstate = self.spot_action.toggle_clip(self.largest_island_ix)

        # Turn off dynamics if spot is being moved kinematically
        # self.spot.sim_obj.motion_type = HSim_MT.KINEMATIC if clipstate else self.spot_motion_type
        print(f"After toggle, clipstate is {clipstate}")

    def move_spot(
        self,
        move_fwd: bool,
        move_back: bool,
        move_up: bool,
        move_down: bool,
        slide_left: bool,
        slide_right: bool,
        turn_left: bool,
        turn_right: bool,
    ):
        inc = 0.02
        min_val = 0.1

        if move_fwd and not move_back:
            self.spot_forward = max(min_val, self.spot_forward + inc)
        elif move_back and not move_fwd:
            self.spot_forward = min(-min_val, self.spot_forward - inc)
        else:
            self.spot_forward *= 0.5
            if abs(self.spot_forward) < min_val:
                self.spot_forward = 0

        if slide_left and not slide_right:
            self.spot_lateral = max(min_val, self.spot_lateral + inc)
        elif slide_right and not slide_left:
            self.spot_lateral = min(-min_val, self.spot_lateral - inc)
        else:
            self.spot_lateral *= 0.5
            if abs(self.spot_lateral) < min_val:
                self.spot_lateral = 0

        if turn_left and not turn_right:
            self.spot_angular = max(min_val, self.spot_angular + inc)
        elif turn_right and not turn_left:
            self.spot_angular = min(-min_val, self.spot_angular - inc)
        else:
            self.spot_angular *= 0.5
            if abs(self.spot_angular) < min_val:
                self.spot_angular = 0

        self.spot_action.step(
            forward=self.spot_forward,
            lateral=self.spot_lateral,
            angular=self.spot_angular,
        )

    def save_and_remove(self):
        """
        Save spot's current location and remove from scene, for saving scene instance.
        """
        aom = self.sim.get_articulated_object_manager()
        self.spot_rigid_state = self.spot.sim_obj.rigid_state
        aom.remove_object_by_id(self.spot.sim_obj.id)

    def restore_at_previous_loc(self):
        """
        Reload spot and restore from saved location.
        """
        # rebuild spot
        self.load_and_init()
        # put em back
        self.spot.sim_obj.rigid_state = self.spot_rigid_state

    def get_point_in_front(self, disp_in_front: mn.Vector3 = None):
        if disp_in_front is None:
            disp_in_front = [1.5, 0.0, 0.0]
        return self.spot.base_transformation.transform_point(disp_in_front)


# Class to manage semantic interaction and display
class SemanticManager:
    def __init__(self, sim: habitat_sim.simulator.Simulator):
        self.sim = sim
        # Descriptive strings for semantic region debug draw possible choices
        self.semantic_region_debug_draw_choices = ["None", "Kitchen Only", "All"]
        # draw semantic region debug visualizations if present : should be [0 : len(semantic_region_debug_draw_choices)-1]
        self.semantic_region_debug_draw_state = 0
        # Colors to use for each region's semantic rendering.
        self.debug_semantic_colors: Dict[str, mn.Color4] = {}

    def cycle_semantic_region_draw(self):
        new_state_idx = (self.semantic_region_debug_draw_state + 1) % len(
            self.semantic_region_debug_draw_choices
        )
        info_str = f"Change Region Draw from {self.semantic_region_debug_draw_choices[self.semantic_region_debug_draw_state]} to {self.semantic_region_debug_draw_choices[new_state_idx]}"

        # Increment visualize semantic bboxes. Currently only regions supported
        self.semantic_region_debug_draw_state = new_state_idx
        return info_str

    def draw_region_debug(self, debug_line_render: Any) -> None:
        """
        Draw the semantic region wireframes.
        """
        if self.semantic_region_debug_draw_state == 0:
            return
        if len(self.debug_semantic_colors) != len(self.sim.semantic_scene.regions):
            self.debug_semantic_colors = {}
            for region in self.sim.semantic_scene.regions:
                self.debug_semantic_colors[region.id] = mn.Color4(
                    mn.Vector3(np.random.random(3))
                )
        if self.semantic_region_debug_draw_state == 1:
            for region in self.sim.semantic_scene.regions:
                if "kitchen" not in region.id.lower():
                    continue
                color = self.debug_semantic_colors.get(region.id, mn.Color4.magenta())
                for edge in region.volume_edges:
                    debug_line_render.draw_transformed_line(
                        edge[0],
                        edge[1],
                        color,
                    )
        else:
            # Draw all
            for region in self.sim.semantic_scene.regions:
                color = self.debug_semantic_colors.get(region.id, mn.Color4.magenta())
                for edge in region.volume_edges:
                    debug_line_render.draw_transformed_line(
                        edge[0],
                        edge[1],
                        color,
                    )
