#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


import os
from typing import Callable, List

import magnum as mn
import numpy as np
from habitat.articulated_agents.robots.spot_robot import SpotRobot
from habitat.datasets.rearrange.navmesh_utils import get_largest_island_index
from omegaconf import DictConfig

import habitat_sim
from habitat_sim import physics as HSim_Phys


# Class to instantiate and maneuver spot from a viewer
# DEPENDENT ON HABITAT-LAB -
class SpotAgent:
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
        agent_config = DictConfig({"articulated_agent_urdf": robot_path})
        self.spot: SpotRobot = SpotRobot(agent_config, self.sim, fixed_base=True)
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
            self.largest_island_ix = get_largest_island_index(
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

    def cache_transform_and_remove(self):
        """
        Save spot's current location and remove from scene, for saving scene instance.
        """
        aom = self.sim.get_articulated_object_manager()
        self.spot_rigid_state = self.spot.sim_obj.rigid_state
        aom.remove_object_by_id(self.spot.sim_obj.object_id)

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
