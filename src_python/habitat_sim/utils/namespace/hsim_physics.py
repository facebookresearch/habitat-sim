#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


from typing import Dict, List, Optional, Union

import magnum as mn

import habitat_sim
from habitat_sim import physics as HSim_Phys


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


def open_link(object_a: HSim_Phys.ManagedArticulatedObject, link_ix: int) -> None:
    """
    Set a link to the "open" state. Sets the joint position to the maximum joint limit.

    TODO: does not do any collision checking to validate the state or move any other objects which may be contained in or supported by this link.

    :param object_a: The parent ArticulatedObject of the link to check.
    :param link_ix: The index of the link within the parent object. Not the link's object_id.
    """

    set_link_normalized_joint_position(object_a, link_ix, 1.0)


def close_link(object_a: HSim_Phys.ManagedArticulatedObject, link_ix: int) -> None:
    """
    Set a link to the "closed" state. Sets the joint position to the minimum joint limit.

    TODO: does not do any collision checking to validate the state or move any other objects which may be contained in or supported by this link.

    :param object_a: The parent ArticulatedObject of the link to check.
    :param link_ix: The index of the link within the parent object. Not the link's object_id.
    """

    set_link_normalized_joint_position(object_a, link_ix, 0)


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


def get_ao_default_link(
    ao: habitat_sim.physics.ManagedArticulatedObject,
    compute_if_not_found: bool = False,
) -> Optional[int]:
    """
    Get the "default" link index for a ManagedArticulatedObject.
    The "default" link is the one link which should be used if only one joint can be actuated. For example, the largest or most accessible drawer or door.

    :param ao: The ManagedArticulatedObject instance.
    :param compute_if_not_found: If true, try to compute the default link if it isn't found.
    :return: The default link index or None if not found. Cannot be base link (-1).

    The default link is determined by:

        - must be "prismatic" or "revolute" joint type
        - first look in the metadata Configuration for an annotated link.
        - (if compute_if_not_found) - if not annotated, it is programmatically computed from a heuristic.

    Default link heuristic: the link with the lowest Y value in the bounding box with appropriate joint type.
    """

    # first look in metadata
    default_link = ao.user_attributes.get("default_link")

    if default_link is None and compute_if_not_found:
        valid_joint_types = [
            habitat_sim.physics.JointType.Revolute,
            habitat_sim.physics.JointType.Prismatic,
        ]
        lowest_link = None
        lowest_y: int = None
        # compute the default link
        for link_id in ao.get_link_ids():
            if ao.get_link_joint_type(link_id) in valid_joint_types:
                # use minimum global keypoint Y value
                link_lowest_y = min(
                    get_articulated_link_global_keypoints(ao, link_id),
                    key=lambda x: x[1],
                )[1]
                if lowest_y is None or link_lowest_y < lowest_y:
                    lowest_y = link_lowest_y
                    lowest_link = link_id
        if lowest_link is not None:
            default_link = lowest_link
            # if found, set in metadata for next time
            ao.user_attributes.set("default_link", default_link)

    return default_link


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


def get_global_keypoints_from_bb(
    aabb: mn.Range3D, local_to_global: mn.Matrix4
) -> List[mn.Vector3]:
    """
    Get a list of bounding box keypoints in global space.
    0th point is the bounding box center, others are bounding box corners.

    :param aabb: The local bounding box.
    :param local_to_global: The local to global transformation matrix.

    :return: A set of global 3D keypoints for the bounding box.
    """
    local_keypoints = [aabb.center()]
    local_keypoints.extend(get_bb_corners(aabb))
    global_keypoints = [
        local_to_global.transform_point(key_point) for key_point in local_keypoints
    ]
    return global_keypoints


def get_articulated_link_global_keypoints(
    object_a: habitat_sim.physics.ManagedArticulatedObject, link_index: int
) -> List[mn.Vector3]:
    """
    Get global bb keypoints for an ArticulatedLink.

    :param object_a: The parent ManagedArticulatedObject for the link.
    :param link_index: The local index of the link within the parent ArticulatedObject. Not the object_id of the link.

    :return: A set of global 3D keypoints for the link.
    """
    link_node = object_a.get_link_scene_node(link_index)

    return get_global_keypoints_from_bb(
        link_node.cumulative_bb, link_node.absolute_transformation()
    )


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

    if ao_link_map is None:
        # Note: better to pre-compute this and pass it around
        ao_link_map = get_ao_link_id_map(sim)

    aom = sim.get_articulated_object_manager()
    if obj_id in ao_link_map:
        return aom.get_object_by_id(ao_link_map[obj_id])

    rom = sim.get_rigid_object_manager()
    if rom.get_library_has_id(obj_id):
        return rom.get_object_by_id(obj_id)

    return None


def get_obj_from_handle(
    sim: habitat_sim.Simulator, obj_handle: str
) -> Union[HSim_Phys.ManagedRigidObject, HSim_Phys.ManagedArticulatedObject,]:
    """
    Get a ManagedRigidObject or ManagedArticulatedObject from its instance handle.

    :param sim: The Simulator instance.
    :param obj_handle: object instance handle for which ManagedObject is desired.

    :return: a ManagedObject or None
    """
    aom = sim.get_articulated_object_manager()
    if aom.get_library_has_handle(obj_handle):
        return aom.get_object_by_handle(obj_handle)

    rom = sim.get_rigid_object_manager()
    if rom.get_library_has_handle(obj_handle):
        return rom.get_object_by_handle(obj_handle)

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
