#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from os import path as osp

# import magnum as mn
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
def test_fetch_robot_wrapper():
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
        # art_obj_mgr = sim.get_articulated_object_manager()
        rigid_obj_mgr = sim.get_rigid_object_manager()

        # setup the camera for debug video (looking at 0,0,0)
        sim.agents[0].scene_node.translation = [0.0, -1.0, 2.0]

        # add a ground plane
        cube_handle = obj_template_mgr.get_template_handles("cubeSolid")[0]
        cube_template_cpy = obj_template_mgr.get_template_by_handle(cube_handle)
        cube_template_cpy.scale = np.array([5.0, 0.2, 5.0])
        obj_template_mgr.register_template(cube_template_cpy)
        ground_plane = rigid_obj_mgr.add_object_by_template_handle(cube_handle)
        ground_plane.motion_type = habitat_sim.physics.MotionType.KINEMATIC
        ground_plane.translation = [0.0, -0.2, 0.0]

        # add the robot to the world via the wrapper
        robot_path = "data/robots/hab_fetch/robots/hab_fetch.urdf"
        fetch = fetch_robot.FetchRobot(robot_path, sim)
        fetch.reconfigure()
        fetch.update()

        for i in range(fetch._robot.num_links):
            # if fetch._robot.get_link_joint_type(i) != habitat_sim.physics.JointType.Fixed:
            print(
                f"{i} = {fetch._robot.get_link_joint_name(i)} | type = {fetch._robot.get_link_joint_type(i)}"
            )

        observations += simulate(sim, 1.0, produce_debug_video)

        # TODO: this isn't working correctly for Fetch yet
        fetch.reset()

        observations += simulate(sim, 1.0, produce_debug_video)

        # TODO: test other functions
        # fetch.ready_arm()
        # fetch.retract_arm()
        # fetch.set_arm_mtr_pos()
        # fetch.get_arm_vel()
        # fetch.get_arm_pos()
        # fetch.set_arm_pos()
        # fetch.open_gripper()
        # fetch.close_gripper()
        # fetch.is_gripper_open()
        # fetch.set_gripper_state()
        # fetch.get_end_effector_transform()
        # fetch.get_arm_joint_lims()

        # produce some test debug video
        if produce_debug_video:
            from habitat_sim.utils import viz_utils as vut

            vut.make_video(
                observations,
                "color_sensor",
                "color",
                "test_fetch_robot_wrapper",
                open_vid=True,
            )
