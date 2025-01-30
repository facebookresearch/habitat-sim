#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


from collections import defaultdict
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple, Union

import magnum as mn
from numpy import pi

import habitat_sim
from habitat_sim import physics as HSim_Phys
from habitat_sim.physics import MotionType as HSim_Phys_MT
from habitat_sim.utils.namespace.hsim_physics import (
    get_ao_root_bb,
    get_obj_from_handle,
    open_link,
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

    # What kind of objects to display boxes around
    class ObjectTypeToDraw(Enum):
        NONE = 0
        ARTICULATED = 1
        RIGID = 2
        BOTH = 3
        NUM_VALS = 4

    OBJECT_TYPE_NAMES = ["", "Articulated", "Rigid", "Both"]

    # distance values in m
    DISTANCE_MODE_VALS = [0.001, 0.01, 0.02, 0.05, 0.1, 0.5]
    # angle value multipliers (in degrees) - multiplied by conversion
    ROTATION_MULT_VALS = [1.0, 10.0, 30.0, 45.0, 60.0, 90.0]
    # 1 radian
    BASE_EDIT_ROT_AMT = pi / 180.0
    # Vector to displace removed objects
    REMOVAL_DISP_VEC = mn.Vector3(0.0, -20.0, 0.0)

    def __init__(self, sim: habitat_sim.simulator.Simulator):
        self.sim = sim
        # Set up object and edit collections/caches
        self._init_obj_caches()
        # Edit mode
        self.curr_edit_mode = ObjectEditor.EditMode.MOVE
        # Edit distance/amount
        self.curr_edit_multiplier = ObjectEditor.DistanceMode.VERY_SMALL
        # Type of objects to draw highlight aabb boxes around
        self.obj_type_to_draw = ObjectEditor.ObjectTypeToDraw.NONE
        # Set initial values
        self.set_edit_vals()

    def _init_obj_caches(self):
        # Internal: Dict of currently selected object ids to index in
        self._sel_obj_ids: Dict[int, int] = {}
        # list of current objects selected. Last object in list is "target" object
        self.sel_objs: List[Any] = []
        # Complete list of per-objec transformation edits, for undo chaining,
        # keyed by object id, value is before and after transform
        self.obj_transform_edits: Dict[
            int, List[tuple[mn.Matrix4, mn.Matrix4, bool]]
        ] = defaultdict(list)
        # Dictionary by object id of transformation when object was most recently saved
        self.obj_last_save_transform: Dict[int, mn.Matrix4] = {}

        # maps a pair of handles to a translation for duplicate detection and debug drawing
        self.duplicate_rigid_object_cache: Dict[Tuple[str, str], mn.Vector3] = {}
        self.duplicate_articulated_object_cache: Dict[Tuple[str, str], mn.Vector3] = {}

        # Complete list of undone transformation edits, for redo chaining,
        # keyed by object id, value is before and after transform.
        # Cleared when any future edits are performed.
        self.obj_transform_undone_edits: Dict[
            int, List[tuple[mn.Matrix4, mn.Matrix4, bool]]
        ] = defaultdict(list)

        # Cache removed objects in dictionary
        # These objects should be hidden/moved out of vieew until the scene is
        # saved, when they should be actually removed (to allow for undo).
        # First key is whether they are articulated or not, value is dict with key == object id, value is object,
        self._removed_objs: Dict[bool, Dict[int, Any]] = defaultdict(dict)
        # Initialize a flag tracking if the scene is dirty or not
        self.modified_scene: bool = False

    def set_edit_mode_rotate(self):
        self.curr_edit_mode = ObjectEditor.EditMode.ROTATE

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

    def get_target_sel_obj(self):
        """
        Retrieve the primary/target selected object. If none then will return none
        """
        if len(self.sel_objs) == 0:
            return None
        return self.sel_objs[-1]

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
        obj_str = self.edit_obj_disp_str()
        obj_type_disp_str = (
            ""
            if self.obj_type_to_draw == ObjectEditor.ObjectTypeToDraw.NONE
            else f"\nObject Types Being Displayed :{ObjectEditor.OBJECT_TYPE_NAMES[self.obj_type_to_draw.value]}"
        )
        dupe_str = (
            ""
            if len(self.duplicate_rigid_object_cache) == 0
            else f"Num Potential Rigid Dupes : {len(self.duplicate_rigid_object_cache)}\n"
        ) + (
            ""
            if len(self.duplicate_articulated_object_cache) == 0
            else f"Num Potential Articulated Dupes : {len(self.duplicate_articulated_object_cache)}\n"
        )
        disp_str = f"""Edit Mode: {edit_mode_string}
Edit Value: {edit_distance_mode_string}
Scene Is Modified: {self.modified_scene}
Num Sel Objs: {len(self.sel_objs)}{obj_str}{obj_type_disp_str}
{dupe_str}
          """
        return disp_str

    def edit_obj_disp_str(self):
        """
        Specify primary selected object display quantities
        """
        if len(self.sel_objs) == 0:
            return ""
        sel_obj = self.sel_objs[-1]
        if sel_obj.is_articulated:
            tar_str = (
                f"Articulated Object : {sel_obj.handle} with {sel_obj.num_links} links."
            )
        else:
            tar_str = f"Rigid Object : {sel_obj.handle}"
        return f"\nTarget Object is {tar_str}"

    def _clear_sel_objs(self):
        """
        Internal: clear object selection structure(s)
        """
        self._sel_obj_ids.clear()
        self.sel_objs.clear()

    def _add_obj_to_sel(self, obj):
        """
        Internal : add object to selection structure(s)
        """
        self._sel_obj_ids[obj.object_id] = len(self.sel_objs)
        # Add most recently selected object to list of selected objects
        self.sel_objs.append(obj)

    def _remove_obj_from_sel(self, obj):
        """
        Internal : remove object from selection structure(s)
        """
        new_objs: List[Any] = []
        # Rebuild selected object structures without the removed object
        self._sel_obj_ids.clear()
        for old_obj in self.sel_objs:
            if old_obj.object_id != obj.object_id:
                self._sel_obj_ids[old_obj.object_id] = len(new_objs)
                new_objs.append(old_obj)
        self.sel_objs = new_objs

    def set_sel_obj(self, obj):
        """
        Set the selected objects to just be the passed obj
        """
        self._clear_sel_objs()
        if obj is not None:
            self._add_obj_to_sel(obj)

    def toggle_sel_obj(self, obj):
        """
        Remove or add the passed object to the selected objects dict, depending on whether it is present, or not.
        """
        if obj is not None:
            if obj.object_id in self._sel_obj_ids:
                # Remove object from obj selected dict
                self._remove_obj_from_sel(obj)
            else:
                # Add object to selected dict
                self._add_obj_to_sel(obj)

    def sel_obj_list(self, obj_handle_list: List[str]):
        """
        Select all objects whose handles are in passed list
        """
        self._clear_sel_objs()
        sim = self.sim
        for obj_handle in obj_handle_list:
            obj = get_obj_from_handle(sim, obj_handle)
            if obj is not None:
                self._add_obj_to_sel(obj)
            else:
                print(f"Unable to find object with handle : {obj_handle}, so skipping.")

    def set_ao_joint_states(
        self, do_open: bool, selected: bool, agent_name: str = "hab_spot"
    ):
        """
        Set either the selected articulated object states to either fully open or fully closed, or all the articulated object states (except for any robots)
        """
        if selected:
            # Only selected objs if they are articulated
            ao_objs = [ao for ao in self.sel_objs if ao.is_articulated]
        else:
            # all articulated objs that do not contain agent's name
            ao_objs = (
                self.sim.get_articulated_object_manager()
                .get_objects_by_handle_substring(search_str=agent_name, contains=False)
                .values()
            )

        if do_open:
            # Open AOs
            for ao in ao_objs:
                # open the selected receptacle(s)
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

    def _set_scene_dirty(self):
        """
        Set whether the scene is currently modified from saved version or
        not. If there are objects to be deleted or cached transformations
        in the undo stack, this flag should be true.
        """
        # Set scene to be modified if any aos or rigids have been marked
        # for deletion, or any objects have been transformed
        self.modified_scene = (len(self._removed_objs[True]) > 0) or (
            len(self._removed_objs[False]) > 0
        )

        # only check transforms if still false
        if not self.modified_scene:
            # check all object transformations match most recent edit transform
            for obj_id, transform_list in self.obj_transform_edits.items():
                if (obj_id not in self.obj_last_save_transform) or (
                    len(transform_list) == 0
                ):
                    continue
                curr_transform = transform_list[-1][1]
                if curr_transform != self.obj_last_save_transform[obj_id]:
                    self.modified_scene = True

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
        action_str = (
            f"{'Articulated' if obj.is_articulated else 'Rigid'} Object {obj.handle}"
        )
        # If object is marked for deletion, don't allow it to be further moved
        if obj.object_id in self._removed_objs[obj.is_articulated]:
            print(
                f"{action_str} already marked for deletion so cannot be moved. Restore object to change its transformation."
            )
            return navmesh_dirty
        # Move object if transforms exist
        if translation is not None or rotation is not None:
            orig_transform = obj.transformation
            # First time save of original transformation for objects being moved
            if obj.object_id not in self.obj_last_save_transform:
                self.obj_last_save_transform[obj.object_id] = orig_transform
            orig_mt = obj.motion_type
            obj.motion_type = HSim_Phys_MT.KINEMATIC
            if translation is not None:
                obj.translation = obj.translation + translation
                action_str = f"{action_str} translated to {obj.translation};"
            if rotation is not None:
                obj.rotation = rotation * obj.rotation
                action_str = f"{action_str} rotated to {obj.rotation};"
            print(action_str)
            obj.motion_type = orig_mt
            # Save transformation for potential undoing later
            trans_tuple = (orig_transform, obj.transformation, removal)
            self.obj_transform_edits[obj.object_id].append(trans_tuple)
            # Clear entries for redo since we have a new edit
            self.obj_transform_undone_edits[obj.object_id] = []
            # Set whether scene has been modified or not
            self._set_scene_dirty()
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
        for obj in self.sel_objs:
            new_navmesh_dirty = self._move_one_object(
                obj,
                navmesh_dirty,
                translation=translation,
                rotation=rotation,
                removal=removal,
            )
            navmesh_dirty = new_navmesh_dirty or navmesh_dirty
        return navmesh_dirty

    def _remove_obj(self, obj):
        """
        Move and mark the passed object for removal from the scene.
        """
        # move selected object outside of direct render area -20 m below current location
        translation = ObjectEditor.REMOVAL_DISP_VEC
        # ignore navmesh result; removal always recomputes
        self._move_one_object(obj, True, translation=translation, removal=True)
        # record removed object for eventual deletion upon scene save
        self._removed_objs[obj.is_articulated][obj.object_id] = obj

    def _restore_obj(self, obj):
        """
        Restore the passed object from the removal queue
        """
        # move selected object back to where it was before - back 20m up
        translation = -ObjectEditor.REMOVAL_DISP_VEC
        # remove object from deletion record
        self._removed_objs[obj.is_articulated].pop(obj.object_id, None)
        # ignore navmesh result; restoration always recomputes
        self._move_one_object(obj, True, translation=translation, removal=False)

    def remove_sel_objects(self):
        """
        'Removes' all selected objects from the scene by hiding them and putting them in queue to be deleted on
        scene save update. Returns list of the handles of all the objects removed if successful

        Note : removal is not permanent unless scene is saved.
        """
        removed_obj_handles = []
        for obj in self.sel_objs:
            if obj is None:
                continue
            self._remove_obj(obj)
            print(
                f"Moved {obj.handle} out of view and marked for removal. Removal becomes permanent when scene is saved."
            )
            # record handle of removed objects, for return
            removed_obj_handles.append(obj.handle)
        # unselect all objects, since they were all 'removed'
        self._clear_sel_objs()
        # retain all object selected transformations.
        return removed_obj_handles

    def restore_removed_objects(self):
        """
        Undo removals that have not been saved yet via scene instance. Will put object back where it was before marking it for removal
        """
        restored_obj_handles = []
        obj_rem_dict = self._removed_objs[True]
        removed_obj_keys = list(obj_rem_dict.keys())
        for obj_id in removed_obj_keys:
            obj = obj_rem_dict.pop(obj_id, None)
            if obj is not None:
                self._restore_obj(obj)
                restored_obj_handles.append(obj.handle)
        obj_rem_dict = self._removed_objs[False]
        removed_obj_keys = list(obj_rem_dict.keys())
        for obj_id in removed_obj_keys:
            obj = obj_rem_dict.pop(obj_id, None)
            if obj is not None:
                self._restore_obj(obj)
                restored_obj_handles.append(obj.handle)

        # Set whether scene is still considered modified/'dirty'
        self._set_scene_dirty()
        return restored_obj_handles

    def delete_removed_objs(self):
        """
        Delete all the objects in the scene marked for removal. Call before saving a scene instance.
        """
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
        # Set whether scene is still considered modified/'dirty'
        self._set_scene_dirty()

    def _undo_obj_transform_edit(
        self, obj, transform_tuple: tuple[mn.Matrix4, mn.Matrix4, bool]
    ):
        """
        Changes the object's current transformation to the passed, previous transformation (in idx 0).
        Different than a move, only called by undo/redo procedure
        """
        old_transform = transform_tuple[0]
        orig_mt = obj.motion_type
        obj.motion_type = HSim_Phys_MT.KINEMATIC
        obj.transformation = old_transform
        obj.motion_type = orig_mt

    def _redo_single_obj_edit(self, obj):
        """
        Redo edit that has been undone on a single object one step
        """
        obj_id = obj.object_id
        # Verify there are transforms to redo for this object
        if len(self.obj_transform_undone_edits[obj_id]) > 0:
            # Last undo state is last element in transforms list
            # In tuple idxs : 0 : previous transform, 1 : current transform, 2 : whether was a removal op
            # Retrieve and remove last undo
            transform_tuple = self.obj_transform_undone_edits[obj_id].pop()
            if len(self.obj_transform_undone_edits[obj_id]) == 0:
                self.obj_transform_undone_edits.pop(obj_id, None)
            # If this had been a removal that had been undone, redo removal
            remove_str = ""
            if transform_tuple[2]:
                # Restore object to removal queue for eventual deletion upon scene save
                self._removed_objs[obj.is_articulated][obj.object_id] = obj
                remove_str = ", being re-marked for removal,"
            self._undo_obj_transform_edit(obj, transform_tuple)
            # Save transformation tuple for subsequent undoing
            # Swap order of transforms since they were redon, for potential undo
            undo_tuple = (
                transform_tuple[1],
                transform_tuple[0],
                transform_tuple[2],
            )
            self.obj_transform_edits[obj_id].append(undo_tuple)
            print(
                f"REDO : Sel Obj : {obj.handle} : Current object{remove_str} transformation : \n{transform_tuple[1]}\nReplaced by saved transformation : \n{transform_tuple[0]}"
            )

    def redo_sel_edits(self):
        """
        Internal only. Redo edits that have been undone on all currently selected objects one step
        NOTE : does not address scene being dirty or not
        """
        if len(self.sel_objs) == 0:
            return
        # For every object in selected object
        for obj in self.sel_objs:
            self._redo_single_obj_edit(obj)
        # Set whether scene is still considered modified/'dirty'
        self._set_scene_dirty()

    def _undo_single_obj_edit(self, obj):
        """
        Internal only. Undo any edits on the passed object one step, (including removal marks)
        NOTE : does not address scene being dirty or not
        """
        obj_id = obj.object_id
        # Verify there are transforms to undo for this object
        if len(self.obj_transform_edits[obj_id]) > 0:
            # Last edit state is last element in transforms list
            # In tuple idxs : 0 : previous transform, 1 : current transform, 2 : whether was a removal op
            # Retrieve and remove last edit
            transform_tuple = self.obj_transform_edits[obj_id].pop()
            # If all object edits have been removed, also remove entry
            if len(self.obj_transform_edits[obj_id]) == 0:
                self.obj_transform_edits.pop(obj_id, None)
            # If this was a removal, remove object from removal queue
            remove_str = ""
            if transform_tuple[2]:
                # Remove object from removal queue if there - undo removal
                self._removed_objs[obj.is_articulated].pop(obj_id, None)
                remove_str = ", being restored from removal list,"
            self._undo_obj_transform_edit(obj, transform_tuple)
            # Save transformation tuple for redoing
            # Swap order of transforms since they were undone, for potential redo
            redo_tuple = (
                transform_tuple[1],
                transform_tuple[0],
                transform_tuple[2],
            )
            self.obj_transform_undone_edits[obj_id].append(redo_tuple)
            print(
                f"UNDO : Sel Obj : {obj.handle} : Current object{remove_str} transformation : \n{transform_tuple[1]}\nReplaced by saved transformation : \n{transform_tuple[0]}"
            )

    def undo_sel_edits(self):
        """
        Undo the edits that have been performed on all the currently selected objects one step, (including removal marks)
        """
        if len(self.sel_objs) == 0:
            return
        # For every object in selected object
        for obj in self.sel_objs:
            self._undo_single_obj_edit(obj)
        # Set whether scene is still considered modified/'dirty'
        self._set_scene_dirty()

    def select_all_matching_objects(self, only_matches: bool):
        """
        Selects all objects matching currently selected object (or first object selected)
        only_matches : only select objects that match type of first selected object (deselects all others)
        """
        if len(self.sel_objs) == 0:
            return
        # primary object is always at idx -1
        match_obj = self.sel_objs[-1]
        obj_is_articulated = match_obj.is_articulated
        if only_matches:
            # clear out existing objects
            self._clear_sel_objs()

        attr_mgr = (
            self.sim.get_articulated_object_manager()
            if obj_is_articulated
            else self.sim.get_rigid_object_manager()
        )
        match_obj_handle = match_obj.handle.split("_:")[0]
        new_sel_objs_dict = attr_mgr.get_objects_by_handle_substring(
            search_str=match_obj_handle, contains=True
        )
        for obj in new_sel_objs_dict.values():
            self._add_obj_to_sel(obj)
        # reset match_obj as selected object by first unselected and then re-selecting
        self.toggle_sel_obj(match_obj)
        self.toggle_sel_obj(match_obj)

    def match_dim_sel_objects(
        self,
        navmesh_dirty: bool,
        new_val: float,
        axis: mn.Vector3,
    ) -> bool:
        """
        Set all selected objects to have the same specified translation dimension value.
        new_val : new value to set the location of the object
        axis : the dimension's axis to match the value of
        """
        trans_vec = new_val * axis
        for obj in self.sel_objs:
            obj_mod_vec = obj.translation.projected(axis)
            new_navmesh_dirty = self._move_one_object(
                obj,
                navmesh_dirty,
                translation=trans_vec - obj_mod_vec,
                removal=False,
            )
            navmesh_dirty = new_navmesh_dirty or navmesh_dirty
        return navmesh_dirty

    def match_x_dim(self, navmesh_dirty: bool) -> bool:
        """
        All selected objects should match specified target object's x value
        """
        if len(self.sel_objs) == 0:
            return None
        match_val = self.sel_objs[-1].translation.x
        return self.match_dim_sel_objects(navmesh_dirty, match_val, mn.Vector3.x_axis())

    def match_y_dim(self, navmesh_dirty: bool) -> bool:
        """
        All selected objects should match specified target object's y value
        """
        if len(self.sel_objs) == 0:
            return None
        match_val = self.sel_objs[-1].translation.y
        return self.match_dim_sel_objects(navmesh_dirty, match_val, mn.Vector3.y_axis())

    def match_z_dim(self, navmesh_dirty: bool) -> bool:
        """
        All selected objects should match specified target object's z value
        """
        if len(self.sel_objs) == 0:
            return None
        match_val = self.sel_objs[-1].translation.z
        return self.match_dim_sel_objects(navmesh_dirty, match_val, mn.Vector3.z_axis())

    def match_orientation(self, navmesh_dirty: bool) -> bool:
        """
        All selected objects should match specified target object's orientation
        """
        if len(self.sel_objs) == 0:
            return None
        match_rotation = self.sel_objs[-1].rotation
        local_navmesh_dirty = False
        for obj in self.sel_objs:
            obj_mod_rot = match_rotation * obj.rotation.inverted()
            local_navmesh_dirty = self._move_one_object(
                obj,
                navmesh_dirty,
                rotation=obj_mod_rot,
                removal=False,
            )
            navmesh_dirty = navmesh_dirty or local_navmesh_dirty
        return navmesh_dirty

    def edit_left(self, navmesh_dirty: bool) -> bool:
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

    def save_current_scene(self):
        if self.modified_scene:
            # update scene with removals before saving
            self.delete_removed_objs()

            # clear out cache of removed objects by resetting dictionary
            self._removed_objs = defaultdict(dict)
            # Reset all AOs to be 0
            self.set_ao_joint_states(do_open=False, selected=False)
            # Save current scene
            self.sim.save_current_scene_config(overwrite=True)
            # Specify most recent edits for each object that has an undo queue
            self.obj_last_save_transform = {}
            obj_ids = list(self.obj_transform_edits.keys())
            for obj_id in obj_ids:
                transform_list = self.obj_transform_edits[obj_id]
                if len(transform_list) == 0:
                    # if transform list is empty, delete it and skip
                    self.obj_transform_edits.pop(obj_id, None)
                    continue
                self.obj_last_save_transform[obj_id] = transform_list[-1][1]

            # Clear edited flag
            self.modified_scene = False
            #
            print("Saved modified scene instance JSON to original location.")
        else:
            print("Nothing modified in scene so save aborted.")

    def load_from_substring(
        self, navmesh_dirty: bool, obj_substring: str, build_loc: mn.Vector3
    ):
        sim = self.sim
        mm = sim.metadata_mediator
        template_mgr = mm.ao_template_manager
        template_handles = template_mgr.get_template_handles(obj_substring)
        build_ao = False
        print(f"Attempting to find {obj_substring} as an articulated object")
        if len(template_handles) == 1:
            print(f"{obj_substring} found as an AO!")
            # Specific AO template found
            obj_mgr = sim.get_articulated_object_manager()
            base_motion_type = HSim_Phys_MT.DYNAMIC
            build_ao = True
        else:
            print(f"Attempting to find {obj_substring} as a rigid object instead")
            template_mgr = mm.object_template_manager
            template_handles = template_mgr.get_template_handles(obj_substring)
            if len(template_handles) != 1:
                print(
                    f"No distinct Rigid or Articulated Object handle found matching substring: '{obj_substring}'. Aborting"
                )
                return [], navmesh_dirty
            print(f"{obj_substring} found as an RO!")
            # Specific Rigid template found
            obj_mgr = sim.get_rigid_object_manager()
            base_motion_type = HSim_Phys_MT.STATIC

        obj_temp_handle = template_handles[0]
        # Build an object using obj_temp_handle, getting template from attr_mgr and object manager obj_mgr
        temp = template_mgr.get_template_by_handle(obj_temp_handle)

        if build_ao:
            obj_type = "Articulated"
            temp.base_type = "FIXED"
            template_mgr.register_template(temp)
            new_obj = obj_mgr.add_articulated_object_by_template_handle(obj_temp_handle)
        else:
            # If any changes to template, put them here and re-register template
            # template_mgr.register_template(temp)
            obj_type = "Rigid"
            new_obj = obj_mgr.add_object_by_template_handle(obj_temp_handle)

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
            print(
                f"Failed to load/create {obj_type} Object from template named {obj_temp_handle}."
            )
            # creation failing would have its own message
            return [], navmesh_dirty
        return [new_obj], navmesh_dirty

    def build_objects(self, navmesh_dirty: bool, build_loc: mn.Vector3):
        """
        Make a copy of the selected object(s), or load a named item at some distance away
        """
        sim = self.sim
        if len(self.sel_objs) > 0:
            # Copy all selected objects
            res_objs = []
            for obj in self.sel_objs:
                # Duplicate object via object ID
                if obj.is_articulated:
                    # duplicate articulated object
                    new_obj = sim.get_articulated_object_manager().duplicate_articulated_object_by_id(
                        obj.object_id
                    )
                else:
                    # duplicate rigid object
                    new_obj = sim.get_rigid_object_manager().duplicate_object_by_id(
                        obj.object_id
                    )

                if new_obj is not None:
                    # set new object location to be above location of copied object
                    new_obj_translation = mn.Vector3(0.0, 1.0, 0.0)
                    # set new object rotation to match copied object's rotation
                    new_obj_rotation = obj.rotation * new_obj.rotation.inverted()
                    new_obj.motion_type = obj.motion_type
                    # move new object to appropriate location
                    new_navmesh_dirty = self._move_one_object(
                        new_obj,
                        navmesh_dirty,
                        translation=new_obj_translation,
                        rotation=new_obj_rotation,
                    )
                    navmesh_dirty = new_navmesh_dirty or navmesh_dirty
                    res_objs.append(new_obj)
            # duplicated all currently selected objects
            # clear currently set selected objects
            self._clear_sel_objs()
            # Select all new objects
            for new_obj in res_objs:
                # add object to selected objects
                self._add_obj_to_sel(new_obj)
            return res_objs, navmesh_dirty

        else:
            # No objects selected, get user input to load a single object
            obj_substring = input(
                "Load Object or AO. Enter a Rigid Object or AO handle substring, first match will be added:"
            ).strip()

            if len(obj_substring) == 0:
                print("No valid name given. Aborting")
                return [], navmesh_dirty
            return self.load_from_substring(
                navmesh_dirty=navmesh_dirty,
                obj_substring=obj_substring,
                build_loc=build_loc,
            )

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

    def change_draw_box_types(self, toggle: bool):
        # Cycle through types of objects to display with highlight box
        mod_val = -1 if toggle else 1
        self.obj_type_to_draw = ObjectEditor.ObjectTypeToDraw(
            (
                self.obj_type_to_draw.value
                + ObjectEditor.ObjectTypeToDraw.NUM_VALS.value
                + mod_val
            )
            % ObjectEditor.ObjectTypeToDraw.NUM_VALS.value
        )

    def handle_duplicate_objects(
        self, find_objs: bool, remove_dupes: bool, trans_eps: float = 0.1
    ):
        if find_objs:
            from difflib import SequenceMatcher

            def handle_dupe_objs_internal(
                obj_dict: Dict[
                    str,
                    Union[
                        HSim_Phys.ManagedRigidObject,
                        HSim_Phys.ManagedArticulatedObject,
                    ],
                ],
                dupe_type: str,
                trans_eps: float,
                handle_eps: float = 0.65,
            ) -> Dict[Tuple[str, str], mn.Vector3]:
                """
                Check for duplicate objects in the scene by looking for overlapping translations among similarly-named objects
                and return dict of candidates.
                """
                obj_translations: Dict[str, mn.Vector3] = {}
                duplicate_object_cache: Dict[Tuple[str, str], mn.Vector3] = {}

                # build translation dict
                for _obj_handle, obj in obj_dict.items():
                    obj_translations[_obj_handle] = obj.translation

                for obj_handle1, translation1 in obj_translations.items():
                    for obj_handle2, translation2 in obj_translations.items():
                        if obj_handle1 == obj_handle2:
                            continue
                        trans_dist = (translation1 - translation2).length()
                        if trans_dist < trans_eps:
                            if (
                                obj_handle1,
                                obj_handle2,
                            ) in duplicate_object_cache or (
                                obj_handle2,
                                obj_handle1,
                            ) in duplicate_object_cache:
                                continue
                            # calculate handle similarity only if translations are close enough to warrant investigation
                            handle_similarity = SequenceMatcher(
                                None, obj_handle1, obj_handle2
                            ).ratio()
                            print(
                                f" - Possible {dupe_type} object duplicate detected: {obj_handle1} and {obj_handle2} with similarity {handle_similarity}"
                            )
                            if handle_similarity > handle_eps:
                                duplicate_object_cache[
                                    (obj_handle1, obj_handle2)
                                ] = translation1

                return duplicate_object_cache

            # look for duplicate rigids
            self.duplicate_rigid_object_cache = handle_dupe_objs_internal(
                obj_dict=self.sim.get_rigid_object_manager().get_objects_by_handle_substring(),
                dupe_type="Rigid",
                trans_eps=trans_eps,
                handle_eps=0.65,
            )
            # look for duplicate aos
            self.duplicate_articulated_object_cache = handle_dupe_objs_internal(
                obj_dict=self.sim.get_articulated_object_manager().get_objects_by_handle_substring(),
                dupe_type="Articulated",
                trans_eps=trans_eps,
                handle_eps=0.65,
            )
            print(
                f"Rigid object duplicates detected (with high similarity): {len(self.duplicate_rigid_object_cache)}\nArticulated object duplicates detected (with high similarity): {len(self.duplicate_articulated_object_cache)}"
            )
        elif remove_dupes:

            def remove_dupe_objs(
                obj_cache: Dict[Tuple[str, str], mn.Vector3], obj_mgr
            ) -> List[str]:
                """
                Automatically remove one of each detected duplicate pair and return the list of removed object handles.
                NOTE: this does not just mark these objects for removal but actually removes them from the scene.
                """
                removed_list = []
                # automatically remove one of each detected duplicate pair
                for obj_handles in obj_cache:
                    # remove the 2nd of each pair assuming it is most likely added 2nd and therefore the duplicate
                    obj_mgr.remove_object_by_handle(obj_handles[1])
                    removed_list.append(obj_handles[1])
                return removed_list

            # remove rigid duplicate proposals
            rigid_removed_list = remove_dupe_objs(
                obj_cache=self.duplicate_rigid_object_cache,
                obj_mgr=self.sim.get_rigid_object_manager(),
            )
            articulated_removed_list = remove_dupe_objs(
                obj_cache=self.duplicate_articulated_object_cache,
                obj_mgr=self.sim.get_articulated_object_manager(),
            )

            print(
                f"Removed {len(rigid_removed_list)} duplicate rigid objects: {rigid_removed_list}"
            )
            print(
                f"Removed {len(articulated_removed_list)} duplicate articulated objects: {articulated_removed_list}"
            )

    def draw_obj_vis(self, camera_trans: mn.Vector3, debug_line_render):
        # draw selected object frames if any objects are selected and any toggled object settings
        self.draw_selected_objects(debug_line_render=debug_line_render)
        # draw a circle around objects that may be duplicates
        self.draw_duplicate_objs(
            camera_trans=camera_trans, debug_line_render=debug_line_render
        )
        # draw highlight box around specific objects
        self.draw_box_around_objs(debug_line_render=debug_line_render)

    def _draw_selected_obj(self, obj, debug_line_render, box_color):
        """
        Draw a selection box around and axis frame at the origin of a single object
        """
        aabb = get_ao_root_bb(obj) if obj.is_articulated else obj.collision_shape_aabb
        debug_line_render.push_transform(obj.transformation)
        debug_line_render.draw_box(aabb.min, aabb.max, box_color)
        debug_line_render.pop_transform()

    def draw_duplicate_objs(self, camera_trans: mn.Vector3, debug_line_render):
        def draw_duplicate_objs_internal(
            camera_trans: mn.Vector3,
            dupe_obj_cache: Dict[Tuple[str, str], mn.Vector3],
            dupe_color: mn.Color4,
            debug_line_render,
        ):
            if len(dupe_obj_cache) == 0:
                return
            # debug draw duplicate object indicators if available
            for _obj_handles, translation in dupe_obj_cache.items():
                debug_line_render.draw_circle(
                    translation=translation,
                    radius=0.1,
                    color=dupe_color,
                    normal=camera_trans - translation,
                )

        # rigid dupes
        draw_duplicate_objs_internal(
            camera_trans=camera_trans,
            dupe_obj_cache=self.duplicate_rigid_object_cache,
            dupe_color=mn.Color4.yellow(),
            debug_line_render=debug_line_render,
        )
        # articulated dupes
        draw_duplicate_objs_internal(
            camera_trans=camera_trans,
            dupe_obj_cache=self.duplicate_articulated_object_cache,
            dupe_color=mn.Color4.green(),
            debug_line_render=debug_line_render,
        )

    def draw_selected_objects(self, debug_line_render):
        if len(self.sel_objs) == 0:
            return
        obj_list = self.sel_objs
        sel_obj = obj_list[-1]
        if sel_obj.is_alive:
            # Last object selected is target object
            self._draw_selected_obj(
                sel_obj,
                debug_line_render=debug_line_render,
                box_color=mn.Color4.yellow(),
            )
            debug_line_render.draw_axes(sel_obj.translation)

        mag_color = mn.Color4.magenta()
        # draw all but last/target object
        for i in range(len(obj_list) - 1):
            obj = obj_list[i]
            if obj.is_alive:
                self._draw_selected_obj(
                    obj, debug_line_render=debug_line_render, box_color=mag_color
                )
                debug_line_render.draw_axes(obj.translation)

    def draw_box_around_objs(self, debug_line_render, agent_name: str = "hab_spot"):
        """
        Draw a box of an object-type-specific color around every object in the scene (green for AOs and cyan for Rigids)
        """
        if self.obj_type_to_draw.value % 2 == 1:
            # draw aos if 1 or 3
            attr_mgr = self.sim.get_articulated_object_manager()
            # Get all aos excluding the agent if present
            new_sel_objs_dict = attr_mgr.get_objects_by_handle_substring(
                search_str=agent_name, contains=False
            )
            obj_clr = mn.Color4.green()
            for obj in new_sel_objs_dict.values():
                if obj.is_alive:
                    self._draw_selected_obj(
                        obj, debug_line_render=debug_line_render, box_color=obj_clr
                    )

        if self.obj_type_to_draw.value > 1:
            # draw rigids if 2 or 3
            attr_mgr = self.sim.get_rigid_object_manager()
            new_sel_objs_dict = attr_mgr.get_objects_by_handle_substring(search_str="")
            obj_clr = mn.Color4.cyan()
            for obj in new_sel_objs_dict.values():
                if obj.is_alive:
                    self._draw_selected_obj(
                        obj, debug_line_render=debug_line_render, box_color=obj_clr
                    )
