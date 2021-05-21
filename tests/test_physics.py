#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import math
import random
from os import path as osp

import magnum as mn
import numpy as np
import pytest
import quaternion

import examples.settings
import habitat_sim
import habitat_sim.physics
from habitat_sim.utils.common import (
    quat_from_angle_axis,
    quat_from_magnum,
    quat_to_magnum,
)


@pytest.mark.skipif(
    not osp.exists("data/scene_datasets/habitat-test-scenes/skokloster-castle.glb")
    or not osp.exists("data/objects/example_objects/"),
    reason="Requires the habitat-test-scenes and habitat test objects",
)
def test_kinematics():
    cfg_settings = examples.settings.default_sim_settings.copy()

    cfg_settings[
        "scene"
    ] = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    # enable the physics simulator: also clears available actions to no-op
    cfg_settings["enable_physics"] = True
    cfg_settings["depth_sensor"] = True

    # test loading the physical scene
    hab_cfg = examples.settings.make_cfg(cfg_settings)
    with habitat_sim.Simulator(hab_cfg) as sim:
        # get the rigid object attributes manager, which manages
        # templates used to create objects
        obj_template_mgr = sim.get_object_template_manager()
        obj_template_mgr.load_configs("data/objects/example_objects/", True)
        assert obj_template_mgr.get_num_templates() > 0
        # get the rigid object manager, which provides direct
        # access to objects
        rigid_obj_mgr = sim.get_rigid_object_manager()

        # test adding an object to the world
        # get handle for object 0, used to test
        obj_handle_list = obj_template_mgr.get_template_handles("cheezit")
        cheezit_box = rigid_obj_mgr.add_object_by_template_handle(obj_handle_list[0])
        assert rigid_obj_mgr.get_num_objects() > 0
        assert (
            len(rigid_obj_mgr.get_object_handles()) == rigid_obj_mgr.get_num_objects()
        )

        # test setting the motion type
        cheezit_box.motion_type = habitat_sim.physics.MotionType.STATIC
        assert cheezit_box.motion_type == habitat_sim.physics.MotionType.STATIC
        cheezit_box.motion_type = habitat_sim.physics.MotionType.KINEMATIC
        assert cheezit_box.motion_type == habitat_sim.physics.MotionType.KINEMATIC

        # test kinematics
        I = np.identity(4)

        # test get and set translation
        cheezit_box.translation = [0.0, 1.0, 0.0]
        assert np.allclose(cheezit_box.translation, np.array([0.0, 1.0, 0.0]))

        # test object SceneNode
        assert np.allclose(
            cheezit_box.translation, cheezit_box.root_scene_node.translation
        )

        # test get and set transform
        cheezit_box.transformation = I
        assert np.allclose(cheezit_box.transformation, I)

        # test get and set rotation
        Q = quat_from_angle_axis(np.pi, np.array([0.0, 1.0, 0.0]))
        expected = np.eye(4)
        expected[0:3, 0:3] = quaternion.as_rotation_matrix(Q)
        cheezit_box.rotation = quat_to_magnum(Q)
        assert np.allclose(cheezit_box.transformation, expected)
        assert np.allclose(quat_from_magnum(cheezit_box.rotation), Q)

        # test object removal
        rigid_obj_mgr.remove_object_by_id(cheezit_box.object_id)

        assert rigid_obj_mgr.get_num_objects() == 0

        obj_handle_list = obj_template_mgr.get_template_handles("cheezit")
        cheezit_box = rigid_obj_mgr.add_object_by_template_handle(obj_handle_list[0])

        prev_time = 0.0
        for _ in range(2):
            # do some kinematics here (todo: translating or rotating instead of absolute)
            cheezit_box.translation = np.random.rand(3)
            T = cheezit_box.transformation  # noqa : F841

            # test getting observation
            sim.step(random.choice(list(hab_cfg.agents[0].action_space.keys())))

            # check that time is increasing in the world
            assert sim.get_world_time() > prev_time
            prev_time = sim.get_world_time()

        rigid_obj_mgr.remove_object_by_id(cheezit_box.object_id)

        # test attaching/dettaching an Agent to/from physics simulation
        agent_node = sim.agents[0].scene_node
        obj_handle_list = obj_template_mgr.get_template_handles("cheezit")
        cheezit_agent = rigid_obj_mgr.add_object_by_template_handle(
            obj_handle_list[0], agent_node
        )

        cheezit_agent.translation = np.random.rand(3)
        assert np.allclose(agent_node.translation, cheezit_agent.translation)
        rigid_obj_mgr.remove_object_by_id(
            cheezit_agent.object_id, delete_object_node=False
        )  # don't delete the agent's node
        assert agent_node.translation

        # test get/set RigidState
        cheezit_box = rigid_obj_mgr.add_object_by_template_handle(obj_handle_list[0])
        targetRigidState = habitat_sim.bindings.RigidState(
            mn.Quaternion(), np.array([1.0, 2.0, 3.0])
        )
        cheezit_box.rigid_state = targetRigidState
        objectRigidState = cheezit_box.rigid_state
        assert np.allclose(objectRigidState.translation, targetRigidState.translation)
        assert objectRigidState.rotation == targetRigidState.rotation


@pytest.mark.skipif(
    not osp.exists("data/scene_datasets/habitat-test-scenes/skokloster-castle.glb")
    or not osp.exists("data/objects/example_objects/"),
    reason="Requires the habitat-test-scenes and habitat test objects",
)
def test_dynamics():
    # This test assumes that default.phys_scene_config.json contains "physics simulator": "bullet".
    # TODO: enable dynamic override of this setting in simulation config structure

    cfg_settings = examples.settings.default_sim_settings.copy()

    cfg_settings[
        "scene"
    ] = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    # enable the physics simulator: also clears available actions to no-op
    cfg_settings["enable_physics"] = True
    cfg_settings["depth_sensor"] = True

    # test loading the physical scene
    hab_cfg = examples.settings.make_cfg(cfg_settings)
    with habitat_sim.Simulator(hab_cfg) as sim:
        # get the rigid object attributes manager, which manages
        # templates used to create objects
        obj_template_mgr = sim.get_object_template_manager()
        obj_template_mgr.load_configs("data/objects/example_objects/", True)
        # make the simulation deterministic (C++ seed is set in reconfigure)
        np.random.seed(cfg_settings["seed"])
        assert obj_template_mgr.get_num_templates() > 0
        # get the rigid object manager, which provides direct
        # access to objects
        rigid_obj_mgr = sim.get_rigid_object_manager()

        # test adding an object to the world
        obj_handle_list = obj_template_mgr.get_template_handles("cheezit")
        cheezit_box1 = rigid_obj_mgr.add_object_by_template_handle(obj_handle_list[0])
        cheezit_box2 = rigid_obj_mgr.add_object_by_template_handle(obj_handle_list[0])

        assert rigid_obj_mgr.get_num_objects() > 0
        assert (
            len(rigid_obj_mgr.get_object_handles()) == rigid_obj_mgr.get_num_objects()
        )

        # place the objects over the table in room
        cheezit_box1.translation = [-0.569043, 2.04804, 13.6156]
        cheezit_box2.translation = [-0.569043, 2.04804, 12.6156]

        # get object MotionType and continue testing if MotionType::DYNAMIC (implies a physics implementation is active)
        if cheezit_box1.motion_type == habitat_sim.physics.MotionType.DYNAMIC:
            object1_init_template = cheezit_box1.creation_attributes
            object1_mass = object1_init_template.mass
            grav = sim.get_gravity()
            previous_object_states = [
                [cheezit_box1.translation, cheezit_box1.rotation],
                [cheezit_box2.translation, cheezit_box2.rotation],
            ]
            prev_time = sim.get_world_time()
            for _ in range(50):
                # force application at a location other than the origin should always cause angular and linear motion
                cheezit_box2.apply_force(np.random.rand(3), np.random.rand(3))

                # TODO: expose object properties (such as mass) to python
                # Counter the force of gravity on the object (it should not translate)
                cheezit_box1.apply_force(-grav * object1_mass, np.zeros(3))

                # apply torque to the "floating" object. It should rotate, but not translate
                cheezit_box1.apply_torque(np.random.rand(3))

                # TODO: test other physics functions

                # test getting observation
                sim.step(random.choice(list(hab_cfg.agents[0].action_space.keys())))

                # check that time is increasing in the world
                assert sim.get_world_time() > prev_time
                prev_time = sim.get_world_time()

                # check the object states
                # 1st object should rotate, but not translate
                assert np.allclose(
                    previous_object_states[0][0], cheezit_box1.translation
                )
                assert previous_object_states[0][1] != cheezit_box1.rotation

                # 2nd object should rotate and translate
                assert not np.allclose(
                    previous_object_states[1][0], cheezit_box2.translation
                )
                assert previous_object_states[1][1] != cheezit_box2.rotation

                previous_object_states = [
                    [cheezit_box1.translation, cheezit_box1.rotation],
                    [cheezit_box2.translation, cheezit_box2.rotation],
                ]

            # test setting DYNAMIC object to KINEMATIC
            cheezit_box2.motion_type = habitat_sim.physics.MotionType.KINEMATIC
            assert cheezit_box2.motion_type == habitat_sim.physics.MotionType.KINEMATIC

            sim.step(random.choice(list(hab_cfg.agents[0].action_space.keys())))

            # 2nd object should no longer rotate or translate
            assert np.allclose(previous_object_states[1][0], cheezit_box2.translation)
            assert previous_object_states[1][1] == cheezit_box2.rotation

            sim.step_physics(0.1)

            # test velocity get/set
            test_lin_vel = np.array([1.0, 0.0, 0.0])
            test_ang_vel = np.array([0.0, 1.0, 0.0])

            # velocity setting for KINEMATIC objects won't be simulated, but will be recorded for bullet internal usage.
            cheezit_box2.linear_velocity = test_lin_vel
            assert cheezit_box2.linear_velocity == test_lin_vel

            cheezit_box2.motion_type = habitat_sim.physics.MotionType.DYNAMIC

            cheezit_box2.linear_velocity = test_lin_vel
            cheezit_box2.angular_velocity = test_ang_vel
            assert cheezit_box2.linear_velocity == test_lin_vel
            assert cheezit_box2.angular_velocity == test_ang_vel

            # test modifying gravity
            new_object_start = np.array([100.0, 0, 0])
            cheezit_box1.translation = new_object_start
            new_grav = np.array([10.0, 0, 0])
            sim.set_gravity(new_grav)
            assert np.allclose(sim.get_gravity(), new_grav)
            assert np.allclose(cheezit_box1.translation, new_object_start)
            sim.step_physics(0.1)
            assert cheezit_box1.translation[0] > new_object_start[0]


def test_velocity_control():
    cfg_settings = examples.settings.default_sim_settings.copy()
    cfg_settings["scene"] = "NONE"
    cfg_settings["enable_physics"] = True
    hab_cfg = examples.settings.make_cfg(cfg_settings)
    with habitat_sim.Simulator(hab_cfg) as sim:
        sim.set_gravity(np.array([0.0, 0.0, 0.0]))
        # get the rigid object attributes manager, which manages
        # templates used to create objects
        obj_template_mgr = sim.get_object_template_manager()
        # get the rigid object manager, which provides direct
        # access to objects
        rigid_obj_mgr = sim.get_rigid_object_manager()

        template_path = osp.abspath("data/test_assets/objects/nested_box")
        template_ids = obj_template_mgr.load_configs(template_path)
        object_template = obj_template_mgr.get_template_by_id(template_ids[0])
        object_template.linear_damping = 0.0
        object_template.angular_damping = 0.0
        obj_template_mgr.register_template(object_template)

        obj_handle = obj_template_mgr.get_template_handle_by_id(template_ids[0])

        for iteration in range(2):
            sim.reset()

            box_object = rigid_obj_mgr.add_object_by_template_handle(obj_handle)
            vel_control = box_object.velocity_control

            if iteration == 0:
                if box_object.motion_type != habitat_sim.physics.MotionType.DYNAMIC:
                    # Non-dynamic simulator in use. Skip 1st pass.
                    rigid_obj_mgr.remove_object_by_id(box_object.object_id)
                    continue
            elif iteration == 1:
                # test KINEMATIC
                box_object.motion_type = habitat_sim.physics.MotionType.KINEMATIC

            # test global velocities
            vel_control.linear_velocity = np.array([1.0, 0.0, 0.0])
            vel_control.angular_velocity = np.array([0.0, 1.0, 0.0])
            vel_control.controlling_lin_vel = True
            vel_control.controlling_ang_vel = True

            while sim.get_world_time() < 1.0:
                # NOTE: stepping close to default timestep to get near-constant velocity control of DYNAMIC bodies.
                sim.step_physics(0.00416)

            ground_truth_pos = sim.get_world_time() * vel_control.linear_velocity
            assert np.allclose(box_object.translation, ground_truth_pos, atol=0.01)
            ground_truth_q = mn.Quaternion([[0, 0.480551, 0], 0.876967])
            angle_error = mn.math.angle(ground_truth_q, box_object.rotation)
            assert angle_error < mn.Rad(0.005)

            sim.reset()

            # test local velocities (turn in a half circle)
            vel_control.lin_vel_is_local = True
            vel_control.ang_vel_is_local = True
            vel_control.linear_velocity = np.array([0, 0, -math.pi])
            vel_control.angular_velocity = np.array([math.pi * 2.0, 0, 0])

            box_object.translation = [0.0, 0.0, 0.0]
            box_object.rotation = mn.Quaternion()

            while sim.get_world_time() < 0.5:
                # NOTE: stepping close to default timestep to get near-constant velocity control of DYNAMIC bodies.
                sim.step_physics(0.008)

            print(sim.get_world_time())

            # NOTE: explicit integration, so expect some error
            ground_truth_q = mn.Quaternion([[1.0, 0.0, 0.0], 0.0])
            print(box_object.translation)
            assert np.allclose(
                box_object.translation, np.array([0, 1.0, 0.0]), atol=0.07
            )
            angle_error = mn.math.angle(ground_truth_q, box_object.rotation)
            assert angle_error < mn.Rad(0.05)

            rigid_obj_mgr.remove_object_by_id(box_object.object_id)


@pytest.mark.skipif(
    not osp.exists("data/scene_datasets/habitat-test-scenes/apartment_1.glb"),
    reason="Requires the habitat-test-scenes",
)
def test_raycast():
    cfg_settings = examples.settings.default_sim_settings.copy()

    # configure some settings in case defaults change
    cfg_settings["scene"] = "data/scene_datasets/habitat-test-scenes/apartment_1.glb"

    # enable the physics simulator
    cfg_settings["enable_physics"] = True

    # loading the physical scene
    hab_cfg = examples.settings.make_cfg(cfg_settings)
    with habitat_sim.Simulator(hab_cfg) as sim:
        # get the rigid object attributes manager, which manages
        # templates used to create objects
        obj_template_mgr = sim.get_object_template_manager()
        # get the rigid object manager, which provides direct
        # access to objects
        rigid_obj_mgr = sim.get_rigid_object_manager()

        if (
            sim.get_physics_simulation_library()
            != habitat_sim.physics.PhysicsSimulationLibrary.NoPhysics
        ):
            # only test this if we have a physics simulator and therefore a collision world
            test_ray_1 = habitat_sim.geo.Ray()
            test_ray_1.direction = mn.Vector3(1.0, 0, 0)
            raycast_results = sim.cast_ray(test_ray_1)
            assert raycast_results.ray.direction == test_ray_1.direction
            assert raycast_results.has_hits()
            assert len(raycast_results.hits) == 1
            assert np.allclose(
                raycast_results.hits[0].point, np.array([6.83063, 0, 0]), atol=0.07
            )
            assert np.allclose(
                raycast_results.hits[0].normal,
                np.array([-0.999587, 0.0222882, -0.0181424]),
                atol=0.07,
            )
            assert abs(raycast_results.hits[0].ray_distance - 6.831) < 0.001
            assert raycast_results.hits[0].object_id == -1

            # add a primitive object to the world and test a ray away from the origin
            cube_prim_handle = obj_template_mgr.get_template_handles("cube")[0]
            cube_obj = rigid_obj_mgr.add_object_by_template_handle(cube_prim_handle)
            cube_obj.translation = [2.0, 0.0, 2.0]

            test_ray_1.origin = np.array([0.0, 0, 2.0])

            raycast_results = sim.cast_ray(test_ray_1)

            assert raycast_results.has_hits()
            assert len(raycast_results.hits) == 4
            assert np.allclose(
                raycast_results.hits[0].point, np.array([1.89048, 0, 2]), atol=0.07
            )
            assert np.allclose(
                raycast_results.hits[0].normal,
                np.array([-0.99774, -0.0475114, -0.0475114]),
                atol=0.07,
            )
            assert abs(raycast_results.hits[0].ray_distance - 1.89) < 0.001
            assert raycast_results.hits[0].object_id == cube_obj.object_id

            # test raycast against a non-collidable object.
            # should not register a hit with the object.
            cube_obj.collidable = False
            raycast_results = sim.cast_ray(test_ray_1)
            assert raycast_results.has_hits()
            assert len(raycast_results.hits) == 3

            # test raycast against a non-collidable stage.
            # should not register any hits.
            sim.set_stage_is_collidable(False)
            raycast_results = sim.cast_ray(test_ray_1)
            assert not raycast_results.has_hits()
