#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os
from typing import Any, Dict, Set

import magnum as mn
import numpy as np

import habitat_sim
from habitat_sim.utils.namespace.hsim_physics import (
    get_obj_from_handle,
    get_obj_from_id,
)


class MarkerSetsEditor:
    def __init__(
        self, sim: habitat_sim.simulator.Simulator, task_names_set: Set = None
    ):
        # Handle default being none
        if task_names_set is None:
            task_names_set = set()
        self.sim = sim
        self.markerset_taskset_names = list(task_names_set)
        self.update_markersets()

    def update_markersets(self):
        """
        Call when created and when a new AO is added or removed
        """
        task_names_set = set(self.markerset_taskset_names)

        self.marker_sets_per_obj = self.get_all_markersets()

        # initialize list of possible taskSet names along with those being added specifically
        for sub_dict in self.marker_sets_per_obj.values():
            for _obj_handle, MarkerSet in sub_dict.items():
                task_names_set.update(MarkerSet.get_all_taskset_names())

        # Necessary class-level variables.
        self.markerset_taskset_names = list(task_names_set)
        # remove trailing s from taskset name for each markerset label prefix
        self.markerset_label_prefix = [
            s[:-1] if s.endswith("s") else s for s in self.markerset_taskset_names
        ]

        self.current_markerset_taskset_idx = 0

        self.marker_sets_changed: Dict[str, Dict[str, bool]] = {}
        # Hierarchy of colors to match the markerset hierarchy
        self.marker_debug_random_colors: Dict[str, Dict[str, Any]] = {}
        for sub_key, sub_dict in self.marker_sets_per_obj.items():
            tmp_dict_changed = {}
            tmp_dict_colors: Dict[str, Any] = {}
            for key in sub_dict:
                print(f"subkey : {sub_key} | key : {key}")
                tmp_dict_changed[key] = False
                tmp_dict_colors[key] = {}
            self.marker_sets_changed[sub_key] = tmp_dict_changed
            self.marker_debug_random_colors[sub_key] = tmp_dict_colors

        # for debugging
        self.glbl_marker_point_dicts_per_obj = self.get_all_global_markers()

    def get_current_taskname(self):
        """
        Retrieve the name of the currently used markerset taskname, as specified in the current object's markersets
        """
        return self.markerset_taskset_names[self.current_markerset_taskset_idx]

    def cycle_current_taskname(self, forward: bool = True):
        mod_val = 1 if forward else -1
        self.current_markerset_taskset_idx = (
            self.current_markerset_taskset_idx
            + len(self.markerset_taskset_names)
            + mod_val
        ) % len(self.markerset_taskset_names)

    def set_current_taskname(self, taskname: str):
        if taskname in self.markerset_taskset_names:
            self.current_markerset_taskset_idx = self.markerset_taskset_names.index(
                taskname
            )
        else:
            print(
                f"Specified taskname {taskname} not valid, so taskname is remaining {self.get_current_taskname()}"
            )

    def get_all_markersets(self):
        """
        Get all the markersets defined in the currently loaded objects and articulated objects.
        Note : any modified/saved markersets may require their owning Configurations
        to be reloaded before this function would expose them.
        """
        print("Start getting all markersets")
        # marker set cache of existing markersets for all objs in scene, keyed by object name
        marker_sets_per_obj = {}
        ro_marker_sets = {}
        rom = self.sim.get_rigid_object_manager()
        obj_dict = rom.get_objects_by_handle_substring("")
        for handle, obj in obj_dict.items():
            print(f"setting rigid markersets for {handle}")
            ro_marker_sets[handle] = obj.marker_sets
        ao_marker_sets = {}
        aom = self.sim.get_articulated_object_manager()
        ao_obj_dict = aom.get_objects_by_handle_substring("")
        for handle, obj in ao_obj_dict.items():
            print(f"setting ao markersets for {handle}")
            ao_marker_sets[handle] = obj.marker_sets
        print("Done getting all markersets")

        marker_sets_per_obj["ao"] = ao_marker_sets
        marker_sets_per_obj["ro"] = ro_marker_sets

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
        task_set_name = self.get_current_taskname()
        marker_set_name = (
            f"{self.markerset_label_prefix[self.current_markerset_taskset_idx]}_000"
        )
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
                obj_type_key = "ao"

            else:
                obj_type = "rigid object"
                # this is an ArticulatedLink, so we can add markers'
                link_ix = -1
                link_name = "root"
                obj_type_key = "ro"

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
                    # get points for current task_set i.e. ("faucets"), link_name, marker_set_name i.e.("faucet_000")
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
                    print(
                        f"About to check obj {obj_handle} if it has points in MarkerSet : {marker_set_name}, LinkSet :{link_name}, TaskSet :{task_set_name} so removal aborted."
                    )
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
            self.marker_sets_per_obj[obj_type_key][obj_handle] = obj_marker_sets
            self.marker_sets_changed[obj_type_key][obj_handle] = True
            self.save_markerset_attributes(obj)
        return selected_object

    def draw_marker_sets_debug(
        self, debug_line_render: Any, camera_position: mn.Vector3
    ) -> None:
        """
        Draw the global state of all configured marker sets.
        """
        for obj_type_key, sub_dict in self.marker_sets_per_obj.items():
            color_dict = self.marker_debug_random_colors[obj_type_key]
            for obj_handle, obj_markerset in sub_dict.items():
                marker_points_dict = obj_markerset.get_all_marker_points()
                if obj_markerset.num_tasksets > 0:
                    obj = get_obj_from_handle(self.sim, obj_handle)
                    for task_name, task_set_dict in marker_points_dict.items():
                        if task_name not in color_dict[obj_handle]:
                            color_dict[obj_handle][task_name] = {}
                        for link_name, link_set_dict in task_set_dict.items():
                            if link_name not in color_dict[obj_handle][task_name]:
                                color_dict[obj_handle][task_name][link_name] = {}
                            if link_name in ["root", "body"]:
                                link_id = -1
                            else:
                                link_id = obj.get_link_id_from_name(link_name)

                            for (
                                markerset_name,
                                marker_pts_list,
                            ) in link_set_dict.items():
                                # print(f"markerset_name : {markerset_name} : marker_pts_list : {marker_pts_list} type : {type(marker_pts_list)} : len : {len(marker_pts_list)}")
                                if (
                                    markerset_name
                                    not in color_dict[obj_handle][task_name][link_name]
                                ):
                                    color_dict[obj_handle][task_name][link_name][
                                        markerset_name
                                    ] = mn.Color4(mn.Vector3(np.random.random(3)))
                                marker_set_color = color_dict[obj_handle][task_name][
                                    link_name
                                ][markerset_name]
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
        for subdict in self.marker_sets_changed.values():
            for obj_handle, is_dirty in subdict.items():
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
            subdict = "ao"
        else:
            # save obj config
            attrMgr = self.sim.metadata_mediator.object_template_manager
            subdict = "ro"
        # get copy of initialization attributes as they were in manager,
        # unmodified by scene instance values such as scale
        init_attrs = attrMgr.get_template_by_handle(obj_init_attr_handle)
        # TEMP TODO Remove this when fixed in Simulator
        # Clean up sub-dirs being added to asset handles.
        if obj.is_articulated:
            init_attrs.urdf_filepath = init_attrs.urdf_filepath.split(os.sep)[-1]
            init_attrs.render_asset_handle = init_attrs.render_asset_handle.split(
                os.sep
            )[-1]
        else:
            init_attrs.render_asset_handle = init_attrs.render_asset_handle.split(
                os.sep
            )[-1]
            init_attrs.collision_asset_handle = init_attrs.collision_asset_handle.split(
                os.sep
            )[-1]
        # put edited subconfig into initial attributes' markersets
        markersets = init_attrs.get_marker_sets()
        # manually copying because the markersets type is getting lost from markersets
        edited_markersets = self.marker_sets_per_obj[subdict][obj.handle]
        if edited_markersets.top_level_num_entries == 0:
            # if all subconfigs of edited_markersets are gone, clear out those in
            # markersets ref within attributes.
            for subconfig_key in markersets.get_subconfig_keys():
                markersets.remove_subconfig(subconfig_key)
        else:
            # Copy subconfigs from local copy of markersets to init attributes' copy
            for subconfig_key in edited_markersets.get_subconfig_keys():
                markersets.save_subconfig(
                    subconfig_key, edited_markersets.get_subconfig(subconfig_key)
                )

        # reregister template
        attrMgr.register_template(init_attrs, init_attrs.handle, True)
        # save to original location - uses saved location in attributes
        attrMgr.save_template_by_handle(init_attrs.handle, True)
        # clear out dirty flag
        self.marker_sets_changed[subdict][obj.handle] = False

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
                    if link_name in ["root", "body"]:
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
        marker_set_dict_ao = {}
        aom = self.sim.get_articulated_object_manager()
        ao_obj_dict = aom.get_objects_by_handle_substring("")
        for handle, obj in ao_obj_dict.items():
            marker_set_dict_ao[handle] = get_points_as_global(
                obj, obj.marker_sets.get_all_marker_points()
            )
        marker_set_dict_ro = {}
        rom = self.sim.get_rigid_object_manager()
        obj_dict = rom.get_objects_by_handle_substring("")
        for handle, obj in obj_dict.items():
            marker_set_dict_ro[handle] = get_points_as_global(
                obj, obj.marker_sets.get_all_marker_points()
            )
        marker_set_global_dicts_per_obj["ao"] = marker_set_dict_ao
        marker_set_global_dicts_per_obj["ro"] = marker_set_dict_ro
        return marker_set_global_dicts_per_obj

    def _draw_markersets_glbl_debug_objtype(
        self, obj_type_key: str, debug_line_render: Any, camera_position: mn.Vector3
    ) -> None:
        obj_dict = self.glbl_marker_point_dicts_per_obj[obj_type_key]
        color_dict = self.marker_debug_random_colors[obj_type_key]
        for (
            obj_handle,
            marker_points_dict,
        ) in obj_dict.items():
            for task_name, task_set_dict in marker_points_dict.items():
                if task_name not in color_dict[obj_handle]:
                    color_dict[obj_handle][task_name] = {}
                for link_name, link_set_dict in task_set_dict.items():
                    if link_name not in color_dict[obj_handle][task_name]:
                        color_dict[obj_handle][task_name][link_name] = {}

                    for markerset_name, global_points in link_set_dict.items():
                        # print(f"markerset_name : {markerset_name} : marker_pts_list : {marker_pts_list} type : {type(marker_pts_list)} : len : {len(marker_pts_list)}")
                        if (
                            markerset_name
                            not in color_dict[obj_handle][task_name][link_name]
                        ):
                            color_dict[obj_handle][task_name][link_name][
                                markerset_name
                            ] = mn.Color4(mn.Vector3(np.random.random(3)))
                        marker_set_color = color_dict[obj_handle][task_name][link_name][
                            markerset_name
                        ]
                        for global_marker_pos in global_points:
                            debug_line_render.draw_circle(
                                translation=global_marker_pos,
                                radius=0.005,
                                color=marker_set_color,
                                normal=camera_position - global_marker_pos,
                            )

    def draw_markersets_glbl_debug(
        self, debug_line_render: Any, camera_position: mn.Vector3
    ) -> None:
        self._draw_markersets_glbl_debug_objtype(
            "ao", debug_line_render=debug_line_render, camera_position=camera_position
        )
        self._draw_markersets_glbl_debug_objtype(
            "ro", debug_line_render=debug_line_render, camera_position=camera_position
        )
