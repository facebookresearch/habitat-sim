#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


from enum import Enum
from typing import Any, Dict, Optional, Set, Union

import magnum as mn
import numpy as np

import habitat_sim
from habitat_sim import physics


def get_obj_from_id(
    sim: habitat_sim.Simulator,
    obj_id: int,
    ao_link_map: Optional[Dict[int, int]] = None,
) -> Union[
    habitat_sim.physics.ManagedRigidObject,
    habitat_sim.physics.ManagedArticulatedObject,
]:
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
) -> Union[
    habitat_sim.physics.ManagedRigidObject,
    habitat_sim.physics.ManagedArticulatedObject,
]:
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
            if isinstance(obj, physics.ManagedArticulatedObject):
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

    def save_markerset_attributes(self, obj) -> None:
        """
        Modify the attributes for the passed object to include the
        currently edited markersets and save those attributes to disk
        """
        # get the name of the attrs used to initialize the object
        obj_init_attr_handle = obj.creation_attributes.handle

        if isinstance(obj, physics.ManagedArticulatedObject):
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

    def __init__(self):
        # Editing
        # Edit mode
        self.curr_edit_mode = ObjectEditor.EditMode.MOVE
        # Edit distance/amount
        self.curr_edit_multiplier = ObjectEditor.DistanceMode.VERY_SMALL
        # cache modified states of any objects moved by the interface.
        self.modified_objects_buffer: Dict[
            habitat_sim.physics.ManagedRigidObject, mn.Matrix4
        ] = {}
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
          """
        return disp_str

    def move_object(
        self,
        obj,
        navmesh_dirty,
        translation: Optional[mn.Vector3] = None,
        rotation: Optional[mn.Quaternion] = None,
    ):
        """
        Move the selected object with a given modification and save the resulting state to the buffer.
        """
        modify_buffer = translation is not None or rotation is not None
        if obj is not None and modify_buffer:
            orig_mt = obj.motion_type
            obj.motion_type = habitat_sim.physics.MotionType.KINEMATIC
            if translation is not None:
                obj.translation = obj.translation + translation
            if rotation is not None:
                obj.rotation = rotation * obj.rotation
            obj.motion_type = orig_mt
            self.modified_objects_buffer[obj] = obj.transformation
            return True
        return navmesh_dirty

    def edit_left(self, obj, navmesh_dirty: bool):
        # if movement mode
        if self.curr_edit_mode == ObjectEditor.EditMode.MOVE:
            navmesh_dirty = self.move_object(
                obj=obj,
                navmesh_dirty=navmesh_dirty,
                translation=mn.Vector3.x_axis() * self.edit_translation_dist,
            )
        # if rotation mode : rotate around y axis
        else:
            navmesh_dirty = self.move_object(
                obj=obj,
                navmesh_dirty=navmesh_dirty,
                rotation=mn.Quaternion.rotation(
                    mn.Rad(self.edit_rotation_amt), mn.Vector3.y_axis()
                ),
            )
        return navmesh_dirty

    def edit_right(self, obj, navmesh_dirty: bool):
        # if movement mode
        if self.curr_edit_mode == ObjectEditor.EditMode.MOVE:
            navmesh_dirty = self.move_object(
                obj=obj,
                navmesh_dirty=navmesh_dirty,
                translation=-mn.Vector3.x_axis() * self.edit_translation_dist,
            )
        # if rotation mode : rotate around y axis
        else:
            navmesh_dirty = self.move_object(
                obj=obj,
                navmesh_dirty=navmesh_dirty,
                rotation=mn.Quaternion.rotation(
                    -mn.Rad(self.edit_rotation_amt), mn.Vector3.y_axis()
                ),
            )
        return navmesh_dirty

    def edit_up(self, obj, navmesh_dirty: bool, alt_pressed: bool):
        # if movement mode
        if self.curr_edit_mode == ObjectEditor.EditMode.MOVE:
            if alt_pressed:
                navmesh_dirty = self.move_object(
                    obj=obj,
                    navmesh_dirty=navmesh_dirty,
                    translation=mn.Vector3.y_axis() * self.edit_translation_dist,
                )
            else:
                navmesh_dirty = self.move_object(
                    obj=obj,
                    navmesh_dirty=navmesh_dirty,
                    translation=mn.Vector3.z_axis() * self.edit_translation_dist,
                )
        # if rotation mode : rotate around x or z axis
        else:
            if alt_pressed:
                # rotate around x axis
                navmesh_dirty = self.move_object(
                    obj=obj,
                    navmesh_dirty=navmesh_dirty,
                    rotation=mn.Quaternion.rotation(
                        mn.Rad(self.edit_rotation_amt), mn.Vector3.x_axis()
                    ),
                )
            else:
                # rotate around z axis
                navmesh_dirty = self.move_object(
                    obj=obj,
                    navmesh_dirty=navmesh_dirty,
                    rotation=mn.Quaternion.rotation(
                        mn.Rad(self.edit_rotation_amt), mn.Vector3.z_axis()
                    ),
                )

        return navmesh_dirty

    def edit_down(self, obj, navmesh_dirty: bool, alt_pressed: bool):
        # if movement mode
        if self.curr_edit_mode == ObjectEditor.EditMode.MOVE:
            if alt_pressed:
                navmesh_dirty = self.move_object(
                    obj=obj,
                    navmesh_dirty=navmesh_dirty,
                    translation=-mn.Vector3.y_axis() * self.edit_translation_dist,
                )
            else:
                navmesh_dirty = self.move_object(
                    obj=obj,
                    navmesh_dirty=navmesh_dirty,
                    translation=-mn.Vector3.z_axis() * self.edit_translation_dist,
                )
        # if rotation mode : rotate around x or z axis
        else:
            if alt_pressed:
                # rotate around x axis
                navmesh_dirty = self.move_object(
                    obj=obj,
                    navmesh_dirty=navmesh_dirty,
                    rotation=mn.Quaternion.rotation(
                        -mn.Rad(self.edit_rotation_amt), mn.Vector3.x_axis()
                    ),
                )
            else:
                # rotate around z axis
                navmesh_dirty = self.move_object(
                    obj=obj,
                    navmesh_dirty=navmesh_dirty,
                    rotation=mn.Quaternion.rotation(
                        -mn.Rad(self.edit_rotation_amt), mn.Vector3.z_axis()
                    ),
                )
        return navmesh_dirty

    def change_edit_mode(self, shift_pressed: bool):
        # toggle edit mode
        mod_val = -1 if shift_pressed else 1
        self.curr_edit_mode = ObjectEditor.EditMode(
            (self.curr_edit_mode.value + ObjectEditor.EditMode.NUM_VALS.value + mod_val)
            % ObjectEditor.EditMode.NUM_VALS.value
        )

    def change_edit_vals(self, shift_pressed: bool):
        # cycle through edit dist/amount multiplier
        mod_val = -1 if shift_pressed else 1
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
