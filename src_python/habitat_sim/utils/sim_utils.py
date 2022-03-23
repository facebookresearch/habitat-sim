# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from typing import Any, Dict, List, Optional, Tuple, Union

import habitat_sim
from habitat_sim import physics

def get_sim_object(sim, object_id:int) -> Optional[Tuple[Union[physics.ManagedRigidObject, physics.ManagedArticulatedObject], int]]:
    """
    Get a Rigid or Articulated ManagedObject from the Simulator from its object id.
    Handles the awkward necessity to check for object ids in both RigidObjectManager and ArticulatedObjectManager.
    Also handles the mapping from link object ids to parent ArticulatedObject.

    :param sim: The Simulator object.
    :param object_id: The object id for the object to get.

    :return: A Tuple containing the ManagedRigidObject or ManagedArticuledObject specified by the object_id and the link_id if the passed object_id references a link or -1 otherwise. None if object_id is invalid.
    """

    if object_id == -1:
        #This is the stage
        return None
    
    # check rigid object first
    rom = sim.get_rigid_object_manager()
    if rom.get_library_has_id(object_id):
        ro = rom.get_object_by_id(object_id)
        return (ro, -1)
    # check articulated object
    aom = sim.get_articulated_object_manager()
    # first check directly for ArticulatedObjects
    if aom.get_library_has_id(object_id):
        ao = aom.get_object_by_id(object_id)
        return (ao, -1)
    # next check for links
    for ao in aom.get_objects_by_handle_substring().values():
        if object_id in ao.link_object_ids:
            return (ao, ao.link_object_ids[object_id])

    # invalid object id
    return None

def get_rigid_component_name(sim, object_id, link_id):
    """
    Computes a descriptive name for the rigid component identified by the passed object and optional link_id.
    example: 'stage' or 'ao--body_name--link_name' or 'ro--body_name'
    """
    # handle the stage
    if object_id == -1:
        return "stage"
    # check rigid object first
    rom = sim.get_rigid_object_manager()
    if rom.get_library_has_id(object_id):
        ro = rom.get_object_by_id(object_id)
        short_handle = ro.handle.split("/")[-1]
        return f"ro--{short_handle}"
    # check articulated object
    aom = sim.get_articulated_object_manager()
    if aom.get_library_has_id(object_id):
        ao = aom.get_object_by_id(object_id)
        short_handle = ao.handle.split("/")[-1]
        link_name = ao.get_link_name(link_id)
        return f"ao--{short_handle}--{link_name}"
    # not a RO or AO, so bad object_id
    raise ValueError("object_id is not valid")