#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from os import path as osp

import numpy as np
import pytest

import examples.settings
import habitat_sim
import habitat_sim.robots.fetch_robot as fetch_robot
from utils import simulate


@pytest.mark.skipif(
    not osp.exists("data/robots/hab_fetch"),
    reason="Test requires Fetch robot URDF and assets.",
)
@pytest.mark.skipif(
    not habitat_sim.built_with_bullet,
    reason="Robot wrapper API requires Bullet physics.",
)
@pytest.mark.parametrize("fixed_base", [True, False])
def test_fetch_robot_wrapper(fixed_base):
    # set this to output test results as video for easy investigation
    produce_debug_video = False
    observations = []
    cfg_settings = examples.settings.default_sim_settings.copy()
    cfg_settings["scene"] = "NONE"
    cfg_settings["enable_physics"] = True

    # loading the physical scene
    hab_cfg = examples.settings.make_cfg(cfg_settings)

    with habitat_sim.Simulator(hab_cfg) as sim:
        obj_template_mgr = sim.get_object_template_manager()
        rigid_obj_mgr = sim.get_rigid_object_manager()

        # setup the camera for debug video (looking at 0,0,0)
        sim.agents[0].scene_node.translation = [0.0, -1.0, 2.0]

        # add a ground plane
        cube_handle = obj_template_mgr.get_template_handles("cubeSolid")[0]
        cube_template_cpy = obj_template_mgr.get_template_by_handle(cube_handle)
        cube_template_cpy.scale = np.array([5.0, 0.2, 5.0])
        obj_template_mgr.register_template(cube_template_cpy)
        ground_plane = rigid_obj_mgr.add_object_by_template_handle(cube_handle)
        ground_plane.translation = [0.0, -0.2, 0.0]
        ground_plane.motion_type = habitat_sim.physics.MotionType.STATIC

        # compute a navmesh on the ground plane
        navmesh_settings = habitat_sim.NavMeshSettings()
        navmesh_settings.set_defaults()
        sim.recompute_navmesh(sim.pathfinder, navmesh_settings, True)
        sim.navmesh_visualization = True

        # add the robot to the world via the wrapper
        robot_path = "data/robots/hab_fetch/robots/hab_fetch.urdf"
        fetch = fetch_robot.FetchRobot(robot_path, sim, fixed_base=fixed_base)
        fetch.reconfigure()
        assert fetch.get_robot_sim_id() == 1  # 0 is the ground plane
        print(fetch.get_link_and_joint_names())
        observations += simulate(sim, 1.0, produce_debug_video)

        # retract the arm
        observations += fetch._interpolate_arm_control(
            [1.2299035787582397, 2.345386505126953],
            [fetch.params.arm_joints[1], fetch.params.arm_joints[3]],
            1,
            30,
            produce_debug_video,
        )

        # ready the arm
        observations += fetch._interpolate_arm_control(
            [-0.45, 0.1],
            [fetch.params.arm_joints[1], fetch.params.arm_joints[3]],
            1,
            30,
            produce_debug_video,
        )

        # setting arm motor positions
        fetch.arm_motor_pos = np.zeros(len(fetch.params.arm_joints))
        observations += simulate(sim, 1.0, produce_debug_video)

        # set base ground position from navmesh
        # NOTE: because the navmesh floats above the collision geometry we should see a pop/settle with dynamics and no fixed base
        target_base_pos = sim.pathfinder.snap_point(fetch.sim_obj.translation)
        fetch.base_pos = target_base_pos
        assert fetch.base_pos == target_base_pos
        observations += simulate(sim, 1.0, produce_debug_video)
        if fixed_base:
            assert np.allclose(fetch.base_pos, target_base_pos)
        else:
            assert not np.allclose(fetch.base_pos, target_base_pos)

        # arm joint queries and setters
        print(f" Arm joint velocities = {fetch.arm_velocity}")
        fetch.arm_joint_pos = np.ones(len(fetch.params.arm_joints))
        fetch.arm_motor_pos = np.ones(len(fetch.params.arm_joints))
        print(f" Arm joint positions (should be ones) = {fetch.arm_joint_pos}")
        print(f" Arm joint limits = {fetch.arm_joint_limits}")
        fetch.arm_motor_pos = fetch.arm_motor_pos
        observations += simulate(sim, 1.0, produce_debug_video)

        # test gripper state
        fetch.open_gripper()
        observations += simulate(sim, 1.0, produce_debug_video)
        assert fetch.is_gripper_open
        assert not fetch.is_gripper_closed
        fetch.close_gripper()
        observations += simulate(sim, 1.0, produce_debug_video)
        assert fetch.is_gripper_closed
        assert not fetch.is_gripper_open

        # halfway open
        fetch.set_gripper_target_state(0.5)
        observations += simulate(sim, 0.5, produce_debug_video)
        assert not fetch.is_gripper_open
        assert not fetch.is_gripper_closed

        # kinematic open/close (checked before simulation)
        fetch.gripper_joint_pos = fetch.params.gripper_open_state
        assert np.allclose(fetch.gripper_joint_pos, fetch.params.gripper_open_state)
        assert fetch.is_gripper_open
        observations += simulate(sim, 0.2, produce_debug_video)
        fetch.gripper_joint_pos = fetch.params.gripper_closed_state
        assert fetch.is_gripper_closed
        observations += simulate(sim, 0.2, produce_debug_video)

        # end effector queries
        print(f" End effector link id = {fetch.ee_link_id}")
        print(f" End effector local offset = {fetch.ee_local_offset}")
        print(f" End effector transform = {fetch.ee_transform}")
        print(
            f" End effector translation (at current state) = {fetch.calculate_ee_forward_kinematics(fetch.sim_obj.joint_positions)}"
        )
        invalid_ef_target = np.array([100.0, 200.0, 300.0])
        print(
            f" Clip end effector target ({invalid_ef_target}) to reach = {fetch.clip_ee_to_workspace(invalid_ef_target)}"
        )

        # produce some test debug video
        if produce_debug_video:
            from habitat_sim.utils import viz_utils as vut

            vut.make_video(
                observations,
                "color_sensor",
                "color",
                "test_fetch_robot_wrapper__fixed_base=" + str(fixed_base),
                open_vid=True,
            )
