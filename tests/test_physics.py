#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import math
import random
from os import path as osp

import magnum as mn
import numpy as np
import pytest
import quaternion

import habitat_sim
import habitat_sim.bindings
import habitat_sim.physics
import habitat_sim.utils.settings
from habitat_sim.utils.common import (
    quat_from_angle_axis,
    quat_from_magnum,
    quat_to_magnum,
    random_quaternion,
)
from utils import simulate


@pytest.mark.skipif(
    not osp.exists("data/scene_datasets/habitat-test-scenes/skokloster-castle.glb")
    or not osp.exists("data/objects/example_objects/"),
    reason="Requires the habitat-test-scenes and habitat test objects",
)
@pytest.mark.skipif(
    not habitat_sim.bindings.built_with_bullet,
    reason="Bullet physics used for validation.",
)
def test_kinematics():
    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()

    cfg_settings[
        "scene"
    ] = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    # enable the physics simulator: also clears available actions to no-op
    cfg_settings["depth_sensor"] = True

    # test loading the physical scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)
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
            T = cheezit_box.transformation  # noqa: F841

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
def test_kinematics_no_physics():
    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()

    cfg_settings[
        "scene"
    ] = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    # enable the physics simulator: also clears available actions to no-op
    cfg_settings["enable_physics"] = False
    cfg_settings["depth_sensor"] = True

    # test loading the physical scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)
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
            T = cheezit_box.transformation  # noqa: F841

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
@pytest.mark.skipif(
    not habitat_sim.bindings.built_with_bullet,
    reason="Bullet physics used for validation.",
)
def test_dynamics():
    # This test assumes that default.phys_scene_config.json contains "physics simulator": "bullet".
    # TODO: enable dynamic override of this setting in simulation config structure

    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()

    cfg_settings[
        "scene"
    ] = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    # enable the physics simulator: also clears available actions to no-op
    cfg_settings["enable_physics"] = True
    cfg_settings["depth_sensor"] = True

    # test loading the physical scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)
    with habitat_sim.Simulator(hab_cfg) as sim:
        # get the rigid object attributes manager, which manages
        # templates used to create objects
        obj_template_mgr = sim.get_object_template_manager()
        obj_template_mgr.load_configs("data/objects/example_objects/", True)
        # make the simulation deterministic (C++ seed is set in reconfigure)
        np.random.seed(cfg_settings["seed"])  # type: ignore[arg-type]
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

        obj_dict = rigid_obj_mgr.get_objects_by_handle_substring("cheezit")
        assert len(obj_dict) == rigid_obj_mgr.get_num_objects()
        for k, v in obj_dict.items():
            assert k == v.handle
            assert v.is_alive

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

            # test gettings/setting friction
            cheezit_box1.friction_coefficient = 0.123
            assert np.allclose(cheezit_box1.friction_coefficient, 0.123, atol=1e-7)
            cheezit_box1.rolling_friction_coefficient = 0.234
            assert np.allclose(
                cheezit_box1.rolling_friction_coefficient, 0.234, atol=1e-7
            )
            cheezit_box1.spinning_friction_coefficient = 0.345
            assert np.allclose(
                cheezit_box1.spinning_friction_coefficient, 0.345, atol=1e-7
            )


def test_velocity_control():
    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()
    cfg_settings["scene"] = "NONE"
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)
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
        obj_attr_margin = object_template.margin

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
                # verify bullet wrapper is being accessed
                assert np.allclose(box_object.margin, obj_attr_margin)
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
@pytest.mark.skipif(
    not habitat_sim.bindings.built_with_bullet,
    reason="Raycasts require Bullet physics.",
)
def test_raycast():
    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()

    # configure some settings in case defaults change
    cfg_settings["scene"] = "data/scene_datasets/habitat-test-scenes/apartment_1.glb"

    # enable the physics simulator
    cfg_settings["enable_physics"] = True

    # loading the physical scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)
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
            sim.set_stage_is_collidable(True)

            # test non-unit ray direction
            test_ray_1.direction = mn.Vector3(0.5, 0, 0)
            raycast_results = sim.cast_ray(test_ray_1)
            assert raycast_results.has_hits()
            assert len(raycast_results.hits) == 3
            assert (
                raycast_results.hits[0].point
                - (
                    test_ray_1.origin
                    + test_ray_1.direction * raycast_results.hits[0].ray_distance
                )
            ).length() < 0.001


@pytest.mark.skipif(
    not osp.exists("data/scene_datasets/habitat-test-scenes/apartment_1.glb"),
    reason="Requires the habitat-test-scenes",
)
@pytest.mark.skipif(
    not habitat_sim.bindings.built_with_bullet,
    reason="Collision testing requires Bullet physics.",
)
def test_collision_groups():
    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()

    # configure some settings in case defaults change
    cfg_settings["scene"] = "data/scene_datasets/habitat-test-scenes/apartment_1.glb"

    # enable the physics simulator
    cfg_settings["enable_physics"] = True

    # loading the physical scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)

    with habitat_sim.Simulator(hab_cfg) as sim:
        # get the rigid object attributes manager, which manages
        # templates used to create objects
        obj_template_mgr = sim.get_object_template_manager()
        # get the rigid object manager, which provides direct
        # access to objects
        rigid_obj_mgr = sim.get_rigid_object_manager()
        ao_mgr = sim.get_articulated_object_manager()

        if (
            sim.get_physics_simulation_library()
            != habitat_sim.physics.PhysicsSimulationLibrary.NoPhysics
        ):
            cgh = habitat_sim.physics.CollisionGroupHelper
            cg = habitat_sim.physics.CollisionGroups

            # test group naming
            assert cgh.get_group_name(cg.UserGroup1) == "UserGroup1"
            assert cgh.get_group("UserGroup1") == cg.UserGroup1
            cgh.set_group_name(cg.UserGroup1, "my_custom_group_1")
            assert cgh.get_group_name(cg.UserGroup1) == "my_custom_group_1"
            assert cgh.get_group("my_custom_group_1") == cg.UserGroup1
            assert cgh.get_mask_for_group(cg.UserGroup1) == cgh.get_mask_for_group(
                "my_custom_group_1"
            )

            # create a custom group behavior (STATIC and KINEMATIC only)
            new_user_group_1_mask = cg.Static | cg.Kinematic
            cgh.set_mask_for_group(cg.UserGroup1, new_user_group_1_mask)

            cube_prim_handle = obj_template_mgr.get_template_handles("cube")[0]
            cube_obj1 = rigid_obj_mgr.add_object_by_template_handle(cube_prim_handle)
            cube_obj2 = rigid_obj_mgr.add_object_by_template_handle(cube_prim_handle)
            # add a DYNAMIC cube in a contact free state
            cube_obj1.translation = [1.0, 0.0, 4.5]
            assert not cube_obj1.contact_test()
            # add another in contact with the first
            cube_obj2.translation = [1.1, 0.0, 4.6]
            assert cube_obj1.contact_test()
            assert cube_obj2.contact_test()
            # override cube1 collision group to STATIC|KINEMATIC only
            cube_obj1.override_collision_group(cg.UserGroup1)
            assert not cube_obj1.contact_test()
            assert not cube_obj2.contact_test()

            # add an ArticulatedObject in contact with cube 2
            ao = ao_mgr.add_articulated_object_from_urdf(
                filepath="data/test_assets/urdf/amass_male.urdf"
            )
            ao.translation = [1.1, 0.0, 4.6]
            assert ao.contact_test()
            assert cube_obj2.contact_test()
            assert not cube_obj1.contact_test()
            # set to non-collidable and check no contacts
            ao.override_collision_group(cg.Noncollidable)
            assert not ao.contact_test()
            assert not cube_obj2.contact_test()
            assert not cube_obj1.contact_test()

            # override cube2 to a new group and configure custom mask to interact with it
            cgh.set_mask_for_group(cg.UserGroup1, new_user_group_1_mask | cg.UserGroup2)
            # NOTE: changing group settings requires overriding object group again
            cube_obj1.override_collision_group(cg.UserGroup1)
            cube_obj2.override_collision_group(cg.UserGroup2)
            assert cube_obj1.contact_test()
            assert cube_obj2.contact_test()

            # NOTE: trying to set the object's MotionType to its current type won't change the collision group
            cube_obj2.motion_type = habitat_sim.physics.MotionType.DYNAMIC
            assert cube_obj1.contact_test()
            assert cube_obj2.contact_test()
            # NOTE: changing the object's MotionType will override the group
            cube_obj2.motion_type = habitat_sim.physics.MotionType.KINEMATIC
            cube_obj2.motion_type = habitat_sim.physics.MotionType.DYNAMIC
            assert not cube_obj1.contact_test()
            assert not cube_obj2.contact_test()
            # cube 1 is still using the custom group and will interact with KINEMATIC
            cube_obj2.motion_type = habitat_sim.physics.MotionType.KINEMATIC
            assert cube_obj1.contact_test()
            assert cube_obj2.contact_test()

            # test convenience bitwise mask setter
            cgh.set_group_interacts_with(cg.UserGroup1, cg.Kinematic, False)
            assert not cgh.get_mask_for_group(cg.UserGroup1) & cg.Kinematic
            cube_obj1.override_collision_group(cg.UserGroup1)
            assert not cube_obj1.contact_test()
            assert not cube_obj2.contact_test()
            cgh.set_group_interacts_with(cg.UserGroup1, cg.Kinematic, True)
            assert cgh.get_mask_for_group(cg.UserGroup1) & cg.Kinematic
            cube_obj1.override_collision_group(cg.UserGroup1)
            assert cube_obj1.contact_test()
            assert cube_obj2.contact_test()

            # test Noncollidable
            cube_obj2.override_collision_group(cg.Noncollidable)
            assert not cube_obj1.contact_test()
            assert not cube_obj2.contact_test()

            # test Noncollidable vs Noncollidable
            cube_obj1.override_collision_group(cg.Noncollidable)
            assert not cube_obj1.contact_test()
            assert not cube_obj2.contact_test()


def check_articulated_object_root_state(
    articulated_object, target_rigid_state, epsilon=1.0e-4
):
    r"""Checks the root state of the ArticulatedObject with all query methods against a target RigidState.

    :param articulated_object: The ArticulatedObject to check
    :param target_rigid_state: A RigidState object separating translation (vector3) rotation (quaternion)
    :param epsilon: An error threshold for numeric comparisons
    """
    # NOTE: basic transform properties refer to the root state
    # convert target to matrix and check against scene_node transform
    assert np.allclose(
        articulated_object.root_scene_node.transformation,
        mn.Matrix4.from_(
            target_rigid_state.rotation.to_matrix(), target_rigid_state.translation
        ),
        atol=epsilon,
    )
    # convert target to matrix and check against transform
    assert np.allclose(
        articulated_object.transformation,
        mn.Matrix4.from_(
            target_rigid_state.rotation.to_matrix(), target_rigid_state.translation
        ),
        atol=epsilon,
    )
    assert np.allclose(
        articulated_object.translation, target_rigid_state.translation, atol=epsilon
    )
    assert mn.math.angle(
        articulated_object.rotation, target_rigid_state.rotation
    ) < mn.Rad(epsilon)
    # check against object's rigid_state
    assert np.allclose(
        articulated_object.rigid_state.translation,
        target_rigid_state.translation,
        atol=epsilon,
    )
    assert mn.math.angle(
        articulated_object.rigid_state.rotation, target_rigid_state.rotation
    ) < mn.Rad(epsilon)


def getRestPositions(articulated_object):
    r"""Constructs a valid rest pose vector for an ArticulatedObject with all zeros for non-spherical joints which get an identity quaternion instead."""
    rest_pose = np.zeros(len(articulated_object.joint_positions))
    for linkIx in range(articulated_object.num_links):
        if (
            articulated_object.get_link_joint_type(linkIx)
            == habitat_sim.physics.JointType.Spherical
        ):
            rest_pose[articulated_object.get_link_joint_pos_offset(linkIx) + 3] = 1
    return rest_pose


def getRandomPositions(articulated_object):
    r"""Constructs a random pose vector for an ArticulatedObject with unit quaternions for spherical joints."""
    joint_limits = articulated_object.joint_position_limits
    lower_limits = np.maximum(joint_limits[0], -1)
    upper_limits = np.minimum(joint_limits[1], 1)
    rand_pose = np.random.uniform(
        lower_limits, upper_limits, len(articulated_object.joint_positions)
    )
    for linkIx in range(articulated_object.num_links):
        if (
            articulated_object.get_link_joint_type(linkIx)
            == habitat_sim.physics.JointType.Spherical
        ):
            # draw a random quaternion
            rand_quat = random_quaternion()
            rand_pose[
                articulated_object.get_link_joint_pos_offset(linkIx) + 3
            ] = rand_quat.scalar
            rand_pose[
                articulated_object.get_link_joint_pos_offset(
                    linkIx
                ) : articulated_object.get_link_joint_pos_offset(linkIx)
                + 3
            ] = rand_quat.vector

    return rand_pose


@pytest.mark.skipif(
    not habitat_sim.bindings.built_with_bullet,
    reason="ArticulatedObject API requires Bullet physics.",
)
@pytest.mark.skipif(
    not habitat_sim.bindings.built_with_bullet,
    reason="Bullet physics used for validation.",
)
def test_articulated_object_add_remove():
    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()
    cfg_settings["scene"] = "NONE"
    cfg_settings["enable_physics"] = True

    # loading the physical scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)

    with habitat_sim.Simulator(hab_cfg) as sim:
        art_obj_mgr = sim.get_articulated_object_manager()
        robot_file = "data/test_assets/urdf/kuka_iiwa/model_free_base.urdf"

        # parse URDF and add a robot to the world
        robot = art_obj_mgr.add_articulated_object_from_urdf(filepath=robot_file)
        assert robot
        assert robot.is_alive
        assert robot.object_id == 0  # first robot added

        # add a second robot
        robot2 = art_obj_mgr.add_articulated_object_from_urdf(
            filepath=robot_file, global_scale=2.0
        )
        assert robot2
        assert art_obj_mgr.get_num_objects() == 2
        assert robot2.global_scale == 2.0

        # remove a robot and check that it was removed
        art_obj_mgr.remove_object_by_handle(robot.handle)
        assert not robot.is_alive
        assert art_obj_mgr.get_num_objects() == 1
        assert robot2.is_alive

        # add some more
        for _i in range(5):
            art_obj_mgr.add_articulated_object_from_urdf(filepath=robot_file)
        assert art_obj_mgr.get_num_objects() == 6

        # remove another
        art_obj_mgr.remove_object_by_id(robot2.object_id)
        assert not robot2.is_alive
        assert art_obj_mgr.get_num_objects() == 5

        # remove all
        art_obj_mgr.remove_all_objects()
        assert art_obj_mgr.get_num_objects() == 0


@pytest.mark.skipif(
    not habitat_sim.bindings.built_with_bullet,
    reason="ArticulatedObject API requires Bullet physics.",
)
def test_articulated_object_maintain_link_order():
    # test that the maintain_link_order option for urdf import
    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()
    cfg_settings["scene"] = "NONE"
    cfg_settings["enable_physics"] = True

    # loading the physical scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)

    with habitat_sim.Simulator(hab_cfg) as sim:
        art_obj_mgr = sim.get_articulated_object_manager()
        amass_file = "data/test_assets/urdf/amass_male.urdf"

        # parse URDF and add a humanoid to the world
        ao = art_obj_mgr.add_articulated_object_from_urdf(
            filepath=amass_file, maintain_link_order=True
        )
        assert ao
        assert ao.is_alive

        amass_urdf_link_order = [
            "lhip",
            "lknee",
            "lankle",
            "rhip",
            "rknee",
            "rankle",
            "lowerback",
            "upperback",
            "chest",
            "lowerneck",
            "upperneck",
            "lclavicle",
            "lshoulder",
            "lelbow",
            "lwrist",
            "rclavicle",
            "rshoulder",
            "relbow",
            "rwrist",
        ]
        link_names = []
        for link_ix in range(ao.num_links):
            link_names.append(ao.get_link_name(link_ix))
        # check the link ordering against ground truth
        assert amass_urdf_link_order == link_names


@pytest.mark.skipif(
    not habitat_sim.bindings.built_with_bullet,
    reason="ArticulatedObject API requires Bullet physics.",
)
@pytest.mark.parametrize(
    "test_asset",
    [
        "data/test_assets/urdf/kuka_iiwa/model_free_base.urdf",
        "data/test_assets/urdf/fridge/fridge.urdf",
        "data/test_assets/urdf/prim_chain.urdf",
        "data/test_assets/urdf/amass_male.urdf",
    ],
)
def test_articulated_object_kinematics(test_asset):
    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()
    cfg_settings["scene"] = "NONE"
    cfg_settings["enable_physics"] = True

    # loading the physical scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)

    with habitat_sim.Simulator(hab_cfg) as sim:
        art_obj_mgr = sim.get_articulated_object_manager()
        robot_file = test_asset

        # parse URDF and add an ArticulatedObject to the world
        robot = art_obj_mgr.add_articulated_object_from_urdf(filepath=robot_file)
        assert robot.is_alive

        # NOTE: basic transform properties refer to the root state
        # root state should be identity by default
        expected_root_state = habitat_sim.RigidState()
        check_articulated_object_root_state(robot, expected_root_state)
        # set the transformation with various methods
        # translation and rotation properties:
        robot.translation = expected_root_state.translation = mn.Vector3(1.0, 2.0, 3.0)
        check_articulated_object_root_state(robot, expected_root_state)
        robot.rotation = expected_root_state.rotation = mn.Quaternion.rotation(
            mn.Rad(0.5), mn.Vector3(-5.0, 1.0, 9.0).normalized()
        )
        check_articulated_object_root_state(robot, expected_root_state)
        # transform property:
        test_matrix_rotation = mn.Quaternion.rotation(
            mn.Rad(0.75), mn.Vector3(4.0, 3.0, -1.0).normalized()
        )
        test_matrix_translation = mn.Vector3(3.0, 2.0, 1.0)
        transform_test_matrix = mn.Matrix4.from_(
            test_matrix_rotation.to_matrix(), test_matrix_translation
        )
        robot.transformation = transform_test_matrix
        expected_root_state = habitat_sim.RigidState(
            test_matrix_rotation, test_matrix_translation
        )
        # looser epsilon for quat->matrix conversions
        check_articulated_object_root_state(robot, expected_root_state, epsilon=1.0e-3)
        # rigid_state property:
        expected_root_state = habitat_sim.RigidState()
        robot.rigid_state = expected_root_state
        check_articulated_object_root_state(robot, expected_root_state)
        # state modifying functions:
        robot.translate(test_matrix_translation)
        expected_root_state.translation = test_matrix_translation
        check_articulated_object_root_state(robot, expected_root_state)
        # rotate the robot 180 degrees
        robot.rotate(mn.Rad(math.pi), mn.Vector3(0, 1.0, 0.0))
        expected_root_state.rotation = mn.Quaternion.rotation(
            mn.Rad(math.pi), mn.Vector3(0, 1.0, 0.0)
        )
        check_articulated_object_root_state(robot, expected_root_state)
        # not testing local transforms at this point since using the same mechanism

        # object should have some degrees of freedom
        num_dofs = len(robot.joint_forces)
        assert num_dofs > 0
        # default zero joint states
        assert np.allclose(robot.joint_positions, getRestPositions(robot))
        assert np.allclose(robot.joint_velocities, np.zeros(num_dofs))
        assert np.allclose(robot.joint_forces, np.zeros(num_dofs))

        # test joint state get/set
        # generate vectors with num_dofs evenly spaced samples in a range
        target_pose = getRandomPositions(robot)
        # positions
        robot.joint_positions = target_pose
        assert np.allclose(robot.joint_positions, target_pose)
        # velocities
        target_joint_vel = np.linspace(1.1, 2.0, num_dofs)
        robot.joint_velocities = target_joint_vel
        assert np.allclose(robot.joint_velocities, target_joint_vel)
        # forces
        target_joint_forces = np.linspace(2.1, 3.0, num_dofs)
        robot.joint_forces = target_joint_forces
        assert np.allclose(robot.joint_forces, target_joint_forces)
        # absolute, not additive setter
        robot.joint_forces = target_joint_forces
        assert np.allclose(robot.joint_forces, target_joint_forces)
        # test additive method
        robot.add_joint_forces(target_joint_forces)
        assert np.allclose(robot.joint_forces, 2 * target_joint_forces)
        # clear all positions, velocities, forces to zero
        robot.clear_joint_states()
        assert np.allclose(robot.joint_positions, getRestPositions(robot))
        assert np.allclose(robot.joint_velocities, np.zeros(num_dofs))
        assert np.allclose(robot.joint_forces, np.zeros(num_dofs))

        # test joint limits and clamping
        joint_limits = robot.joint_position_limits
        lower_pos_limits = joint_limits[0]
        upper_pos_limits = joint_limits[1]

        # setup joint positions outside of the limit range
        invalid_joint_positions = getRestPositions(robot)
        for pos in range(len(invalid_joint_positions)):
            if not math.isinf(upper_pos_limits[pos]):
                invalid_joint_positions[pos] = upper_pos_limits[pos] + 0.1
        robot.joint_positions = invalid_joint_positions
        # allow these to be set
        assert np.allclose(robot.joint_positions, invalid_joint_positions, atol=1.0e-4)
        # then clamp back into valid range
        robot.clamp_joint_limits()
        assert np.all(robot.joint_positions <= upper_pos_limits)
        assert np.all(robot.joint_positions <= invalid_joint_positions)
        # repeat with lower limits
        invalid_joint_positions = getRestPositions(robot)
        for pos in range(len(invalid_joint_positions)):
            if not math.isinf(lower_pos_limits[pos]):
                invalid_joint_positions[pos] = lower_pos_limits[pos] - 0.1
        robot.joint_positions = invalid_joint_positions
        # allow these to be set
        assert np.allclose(robot.joint_positions, invalid_joint_positions, atol=1.0e-4)
        # then clamp back into valid range
        robot.clamp_joint_limits()
        assert np.all(robot.joint_positions >= lower_pos_limits)

        # test auto-clamping (only occurs during step function BEFORE integration)
        robot.joint_positions = invalid_joint_positions
        assert np.allclose(robot.joint_positions, invalid_joint_positions, atol=1.0e-4)
        # taking a single step should not clamp positions by default
        sim.step_physics(-1)
        assert np.allclose(robot.joint_positions, invalid_joint_positions, atol=1.0e-3)
        assert robot.auto_clamp_joint_limits == False
        robot.auto_clamp_joint_limits = True
        assert robot.auto_clamp_joint_limits == True
        # taking a single step should clamp positions when auto clamp enabled
        sim.step_physics(-1)
        assert np.all(robot.joint_positions >= lower_pos_limits)


@pytest.mark.skipif(
    not osp.exists("data/scene_datasets/habitat-test-scenes/apartment_1.glb"),
    reason="Requires the habitat-test-scenes",
)
@pytest.mark.skipif(
    not habitat_sim.bindings.built_with_bullet,
    reason="ArticulatedObject API requires Bullet physics.",
)
@pytest.mark.parametrize(
    "test_asset",
    [
        "data/test_assets/urdf/kuka_iiwa/model_free_base.urdf",
        "data/test_assets/urdf/fridge/fridge.urdf",
        "data/test_assets/urdf/prim_chain.urdf",
        "data/test_assets/urdf/amass_male.urdf",
    ],
)
def test_articulated_object_dynamics(test_asset):
    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()
    cfg_settings["scene"] = "data/scene_datasets/habitat-test-scenes/apartment_1.glb"
    cfg_settings["enable_physics"] = True

    # loading the physical scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)

    with habitat_sim.Simulator(hab_cfg) as sim:
        art_obj_mgr = sim.get_articulated_object_manager()
        robot_file = test_asset

        # parse URDF and add an ArticulatedObject to the world
        robot = art_obj_mgr.add_articulated_object_from_urdf(filepath=robot_file)
        assert robot.is_alive

        # object should be initialized with dynamics
        assert robot.motion_type == habitat_sim.physics.MotionType.DYNAMIC
        sim.step_physics(0.2)
        assert robot.root_linear_velocity[1] < -1.0
        sim.step_physics(2.8)
        # the robot should fall to the floor under gravity and stop
        assert robot.translation[1] < -0.5
        assert robot.translation[1] > -1.7

        # test linear and angular root velocity
        robot.translation = mn.Vector3(100)
        robot.rotation = mn.Quaternion()
        target_lin_vel = np.linspace(1.1, 2.0, 3)
        target_ang_vel = np.linspace(2.1, 3.0, 3)
        robot.root_linear_velocity = target_lin_vel
        robot.root_angular_velocity = target_ang_vel
        assert np.allclose(robot.root_linear_velocity, target_lin_vel, atol=1.0e-4)
        assert np.allclose(robot.root_angular_velocity, target_ang_vel, atol=1.0e-4)
        # take a single step and expect the velocity to be applied
        current_time = sim.get_world_time()
        sim.step_physics(-1)
        timestep = sim.get_world_time() - current_time
        lin_finite_diff = (robot.translation - mn.Vector3(100)) / timestep
        assert np.allclose(robot.root_linear_velocity, lin_finite_diff, atol=1.0e-3)
        expected_rotation = mn.Quaternion.rotation(
            mn.Rad(np.linalg.norm(target_ang_vel * timestep)),
            target_ang_vel / np.linalg.norm(target_ang_vel),
        )
        angle_error = mn.math.angle(robot.rotation, expected_rotation)
        assert angle_error < mn.Rad(0.0005)

        assert not np.allclose(robot.root_linear_velocity, mn.Vector3(0), atol=0.1)
        assert not np.allclose(robot.root_angular_velocity, mn.Vector3(0), atol=0.1)

        # reset root transform and switch to kinematic
        robot.translation = mn.Vector3(0)
        robot.clear_joint_states()
        robot.motion_type = habitat_sim.physics.MotionType.KINEMATIC
        assert robot.motion_type == habitat_sim.physics.MotionType.KINEMATIC
        sim.step_physics(1.0)
        assert robot.translation == mn.Vector3(0)
        assert robot.root_linear_velocity == mn.Vector3(0)
        assert robot.root_angular_velocity == mn.Vector3(0)
        # set forces and velocity and check no simulation result
        current_positions = robot.joint_positions
        robot.joint_velocities = np.linspace(1.1, 2.0, len(robot.joint_velocities))
        robot.joint_forces = np.linspace(2.1, 3.0, len(robot.joint_forces))
        sim.step_physics(1.0)
        assert np.allclose(robot.joint_positions, current_positions, atol=1.0e-4)
        assert robot.translation == mn.Vector3(0)
        # positions can be manually changed
        target_joint_positions = getRandomPositions(robot)
        robot.joint_positions = target_joint_positions
        assert np.allclose(robot.joint_positions, target_joint_positions, atol=1.0e-4)

        # instance fresh robot with fixed base
        art_obj_mgr.remove_object_by_id(robot.object_id)
        robot = art_obj_mgr.add_articulated_object_from_urdf(
            filepath=robot_file, fixed_base=True
        )
        assert robot.translation == mn.Vector3(0)
        assert robot.motion_type == habitat_sim.physics.MotionType.DYNAMIC
        # perturb the system dynamically
        robot.joint_velocities = np.linspace(5.1, 8.0, len(robot.joint_velocities))
        sim.step_physics(1.0)
        # root should remain fixed
        assert robot.translation == mn.Vector3(0)
        assert robot.root_linear_velocity == mn.Vector3(0)
        assert robot.root_angular_velocity == mn.Vector3(0)
        # positions should be dynamic and perturbed by velocities
        assert not np.allclose(robot.joint_positions, getRestPositions(robot), atol=0.1)

        # instance fresh robot with free base
        art_obj_mgr.remove_object_by_id(robot.object_id)
        robot = art_obj_mgr.add_articulated_object_from_urdf(filepath=robot_file)
        # put object to sleep
        assert robot.can_sleep
        assert robot.awake
        robot.awake = False
        assert not robot.awake
        sim.step_physics(1.0)
        assert not robot.awake
        assert robot.translation == mn.Vector3(0)

        # add a new object to drop onto the first, waking it up
        robot2 = art_obj_mgr.add_articulated_object_from_urdf(filepath=robot_file)
        sim.step_physics(0.5)
        assert robot.awake
        assert robot2.awake


@pytest.mark.skipif(
    not habitat_sim.bindings.built_with_bullet,
    reason="ArticulatedObject API requires Bullet physics.",
)
def test_articulated_object_fixed_base_proxy():
    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()
    cfg_settings["scene"] = "NONE"
    cfg_settings["enable_physics"] = True

    # loading the physical scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)

    with habitat_sim.Simulator(hab_cfg) as sim:
        art_obj_mgr = sim.get_articulated_object_manager()
        rigid_obj_mgr = sim.get_rigid_object_manager()
        obj_template_mgr = sim.get_object_template_manager()

        robot_file = "data/test_assets/urdf/fixed_base_test.urdf"

        # parse URDF and add an ArticulatedObject to the world with fixed base
        robot = art_obj_mgr.add_articulated_object_from_urdf(
            filepath=robot_file, fixed_base=True
        )
        assert robot.is_alive

        # add a test object to the world
        cube_prim_handle = obj_template_mgr.get_template_handles("cube")[0]
        cube_obj = rigid_obj_mgr.add_object_by_template_handle(cube_prim_handle)
        for it in range(2):
            if it == 1:
                # test updating the fixed base position
                robot.translation = [0.2, 0.3, 0.4]
                cube_obj.translation = robot.translation
            # initial position should be intersecting with all components
            assert robot.contact_test()
            # move the Dynamic link out of the way
            robot.joint_positions = [0.3]
            # should still report contact because cube_obj is Dynamic
            assert robot.contact_test()
            # should not report contact once cube_obj is Static because base link is also Static
            cube_obj.motion_type = habitat_sim.physics.MotionType.STATIC
            assert not robot.contact_test()
            cube_obj.motion_type = habitat_sim.physics.MotionType.KINEMATIC
            assert not robot.contact_test()
            robot.joint_positions = [0.0]
            assert robot.contact_test()
            cube_obj.motion_type = habitat_sim.physics.MotionType.DYNAMIC


@pytest.mark.skipif(
    not habitat_sim.bindings.built_with_bullet,
    reason="ArticulatedObject API requires Bullet physics.",
)
def test_articulated_object_damping_joint_motors():
    # test automated creation of joint motors from URDF configured joint damping values
    robot_file = "data/test_assets/urdf/kuka_iiwa/model_free_base.urdf"
    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()
    cfg_settings["scene"] = "NONE"
    cfg_settings["enable_physics"] = True
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)
    with habitat_sim.Simulator(hab_cfg) as sim:
        art_obj_mgr = sim.get_articulated_object_manager()
        # parse URDF and add an ArticulatedObject to the world
        robot = art_obj_mgr.add_articulated_object_from_urdf(filepath=robot_file)
        assert robot.is_alive
        # When URDF joint damping is defined, we generate a set of motors automatically
        # get a map of joint motor ids to starting DoF indices
        existing_joint_motors = robot.existing_joint_motor_ids
        assert len(existing_joint_motors) == 7
        for id_to_dof in existing_joint_motors.items():
            # for this model, should be single dof motors for all dofs
            assert id_to_dof[0] == id_to_dof[1]
            motor_settings = robot.get_joint_motor_settings(id_to_dof[0])
            # note max impulse is taken directly from the configured damping value
            assert motor_settings.max_impulse == 0.5
            assert (
                motor_settings.motor_type
                == habitat_sim.physics.JointMotorType.SingleDof
            )
            # should not be controlling position
            assert motor_settings.position_gain == 0
            # should be attempting to maintain 0 velocity
            assert motor_settings.velocity_gain == 1.0
            assert motor_settings.velocity_target == 0.0


def check_joint_positions(robot, target, single_dof_eps=5.0e-3, quat_eps=0.2):
    positions = robot.joint_positions
    for link_id in robot.get_link_ids():
        start_pos = robot.get_link_joint_pos_offset(link_id)
        joint_type = robot.get_link_joint_type(link_id)
        if joint_type == habitat_sim.physics.JointType.Spherical:
            target_q = mn.Quaternion(
                target[start_pos : start_pos + 3], target[start_pos + 3]
            )
            actual_q = mn.Quaternion(
                positions[start_pos : start_pos + 3], positions[start_pos + 3]
            )
            angle_error = mn.math.angle(target_q, actual_q)
            angle_error2 = mn.math.angle(target_q, -1 * actual_q)
            # negative quaternion represents the same rotation, but gets a different angle error so check both
            assert angle_error < mn.Rad(quat_eps) or angle_error2 < mn.Rad(quat_eps)
        elif joint_type in [
            habitat_sim.physics.JointType.Revolute,
            habitat_sim.physics.JointType.Prismatic,
        ]:
            assert abs(target[start_pos] - positions[start_pos]) < single_dof_eps


@pytest.mark.skipif(
    not habitat_sim.bindings.built_with_bullet,
    reason="ArticulatedObject API requires Bullet physics.",
)
@pytest.mark.parametrize(
    "test_asset",
    [
        "data/test_assets/urdf/kuka_iiwa/model_free_base.urdf",
        "data/test_assets/urdf/fridge/fridge.urdf",
        "data/test_assets/urdf/prim_chain.urdf",
        "data/test_assets/urdf/amass_male.urdf",
    ],
)
def test_articulated_object_joint_motors(test_asset):
    # set this to output test results as video for easy investigation
    produce_debug_video = False
    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()
    cfg_settings["scene"] = "NONE"
    cfg_settings["enable_physics"] = True

    # loading the physical scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)

    with habitat_sim.Simulator(hab_cfg) as sim:
        art_obj_mgr = sim.get_articulated_object_manager()
        robot_file = test_asset

        # add an ArticulatedObject to the world with a fixed base
        robot = art_obj_mgr.add_articulated_object_from_urdf(
            filepath=robot_file, fixed_base=True
        )
        assert robot.is_alive

        # remove any automatically created motors
        existing_motor_ids = robot.existing_joint_motor_ids
        for motor_id in existing_motor_ids:
            robot.remove_joint_motor(motor_id)
        assert len(robot.existing_joint_motor_ids) == 0

        check_joint_positions(robot, getRestPositions(robot))

        # setup the camera for debug video
        sim.agents[0].scene_node.translation = [0.0, -1.5, 2.0]
        observations = []
        target_time = 0.0

        # let the agent drop
        target_time += 0.2
        while sim.get_world_time() < target_time:
            sim.step_physics(1.0 / 60.0)
            if produce_debug_video:
                observations.append(sim.get_sensor_observations())

        # iterate through links and setup joint motors to hold a rest position
        joint_motor_settings = None
        print(f" L({-1}): name = {robot.get_link_name(-1)}")
        for link_id in robot.get_link_ids():
            print(
                f" L({link_id}): name = {robot.get_link_name(link_id)} | joint_name = {robot.get_link_joint_name(link_id)}"
            )
            if (
                robot.get_link_joint_type(link_id)
                == habitat_sim.physics.JointType.Spherical
            ):
                # construct a spherical JointMotorSettings
                joint_motor_settings = habitat_sim.physics.JointMotorSettings(
                    spherical_position_target=mn.Quaternion(),
                    position_gain=1.0,
                    spherical_velocity_target=mn.Vector3(),
                    velocity_gain=0.1,
                    max_impulse=10000.0,
                )
            elif (
                robot.get_link_joint_type(link_id)
                == habitat_sim.physics.JointType.Prismatic
                or robot.get_link_joint_type(link_id)
                == habitat_sim.physics.JointType.Revolute
            ):
                # construct a single dof JointMotorSettings
                joint_motor_settings = habitat_sim.physics.JointMotorSettings(
                    position_target=0.0,
                    position_gain=1.0,
                    velocity_target=0.0,
                    velocity_gain=0.1,
                    max_impulse=10000.0,
                )
            else:
                # planar or fixed joints are not supported
                continue
            # create the motor from its settings
            robot.create_joint_motor(link_id, joint_motor_settings)

        target_time += 6.0
        while sim.get_world_time() < target_time:
            sim.step_physics(1.0 / 60.0)
            if produce_debug_video:
                observations.append(sim.get_sensor_observations())

        # check that at rest position torque is low
        if "amass_male" not in test_asset:
            assert (
                np.abs(
                    np.array(robot.get_joint_motor_torques(sim.get_physics_time_step()))
                )
                < 10
            ).all()

        # validate that rest pose is maintained
        # Note: assume all joints for test assets can be actuated
        target_positions = getRestPositions(robot)
        check_joint_positions(robot, target_positions)

        # check removal and auto-creation
        joint_motor_settings = habitat_sim.physics.JointMotorSettings(
            position_target=0.0,
            position_gain=0.8,
            velocity_target=0.0,
            velocity_gain=0.2,
            max_impulse=10000.0,
        )
        num_motors = len(robot.existing_joint_motor_ids)
        existing_motor_ids = robot.existing_joint_motor_ids
        for motor_id in existing_motor_ids:
            robot.remove_joint_motor(motor_id)
        assert len(robot.existing_joint_motor_ids) == 0

        robot.create_all_motors(joint_motor_settings)
        assert len(robot.existing_joint_motor_ids) == num_motors

        # set new random position targets
        random_position_target = getRandomPositions(robot)
        robot.update_all_motor_targets(random_position_target)

        target_time += 6.0
        while sim.get_world_time() < target_time:
            sim.step_physics(1.0 / 60.0)
            if produce_debug_video:
                observations.append(sim.get_sensor_observations())
        # NOTE: because target is randomly generated, this check can fail probabilistically (try re-running test)
        check_joint_positions(robot, random_position_target)

        # set zero position gains and non-zero velocity target
        new_vel_target = np.ones(len(robot.joint_velocities)) * 0.5
        robot.update_all_motor_targets(new_vel_target, velocities=True)

        for motor_id in robot.existing_joint_motor_ids:
            joint_motor_settings = robot.get_joint_motor_settings(motor_id)
            # first check that velocity target update is reflected in settings
            if (
                joint_motor_settings.motor_type
                == habitat_sim.physics.JointMotorType.SingleDof
            ):
                assert joint_motor_settings.velocity_target == 0.5
            else:
                # spherical
                assert joint_motor_settings.spherical_velocity_target == mn.Vector3(0.5)
            joint_motor_settings.position_gain = 0
            joint_motor_settings.velocity_gain = 1.0
            robot.update_joint_motor(motor_id, joint_motor_settings)

        # to ensure joint has enough distance to achieve target velocity, reset positions
        robot.clear_joint_states()

        firstStep = True
        target_time += 0.5
        while sim.get_world_time() < target_time:
            sim.step_physics(1.0 / 60.0)
            if (
                firstStep and "amass_male" not in test_asset
            ):  # amass_male has spherical joints
                joint_motor_torques = np.abs(
                    np.array(robot.get_joint_motor_torques(sim.get_physics_time_step()))
                )
                assert (joint_motor_torques > 1.0).any()
                assert len(joint_motor_torques) > 0
                print("PYTORQUES: ", joint_motor_torques)
                firstStep = False
            if produce_debug_video:
                observations.append(sim.get_sensor_observations())

        # TODO: spherical joint motor velocities are not working correctly
        if "amass_male" not in test_asset:
            assert np.allclose(new_vel_target, robot.joint_velocities, atol=0.06)

        # produce some test debug video
        if produce_debug_video:
            from habitat_sim.utils import viz_utils as vut

            vut.make_video(
                observations,  # type: ignore[arg-type]
                "color_sensor",
                "color",
                "test_articulated_object_joint_motors__" + test_asset.split("/")[-1],
                open_vid=False,
            )


@pytest.mark.skipif(
    not habitat_sim.bindings.built_with_bullet,
    reason="ArticulatedObject API requires Bullet physics.",
)
def test_rigid_constraints():
    # set this to output test results as video for easy investigation
    produce_debug_video = False
    observations = []
    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()
    cfg_settings["scene"] = "NONE"
    cfg_settings["enable_physics"] = True

    # loading the physical scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)

    with habitat_sim.Simulator(hab_cfg) as sim:
        obj_template_mgr = sim.get_object_template_manager()
        art_obj_mgr = sim.get_articulated_object_manager()
        rigid_obj_mgr = sim.get_rigid_object_manager()

        # setup the camera for debug video (looking at 0,0,0)
        sim.agents[0].scene_node.translation = [0.0, -1.5, 2.0]
        sim.agents[0]._sensors["color_sensor"].specification().clear_color = [
            0.5,
            0.5,
            0.5,
        ]

        # add a visualize reference frame
        cube_prim_viz_handle = obj_template_mgr.get_template_handles("cubeWireframe")[0]
        cube_viz_obj = rigid_obj_mgr.add_object_by_template_handle(cube_prim_viz_handle)
        cube_viz_obj.collidable = False
        cube_viz_obj.motion_type = habitat_sim.physics.MotionType.KINEMATIC
        viz_cube_bb = cube_viz_obj.collision_shape_aabb

        # add a test cube object
        cube_prim_handle = obj_template_mgr.get_template_handles("cubeSolid")[0]
        cube_obj = rigid_obj_mgr.add_object_by_template_handle(cube_prim_handle)

        # ---------------------------
        # test rigid P2P constraints
        # ---------------------------

        # add a constraint to dangle the cube from its corner at the origin
        constraint_settings = habitat_sim.physics.RigidConstraintSettings()
        constraint_settings.object_id_a = cube_obj.object_id
        constraint_settings.pivot_a = cube_obj.collision_shape_aabb.front_top_left
        constraint_id = sim.create_rigid_constraint(constraint_settings)
        assert constraint_id >= 0
        observations += simulate(sim, 1.0, produce_debug_video)
        global_pivot_pos = cube_obj.root_scene_node.transformation.transform_point(
            constraint_settings.pivot_a
        )
        assert np.allclose(global_pivot_pos, np.zeros(3), atol=1.0e-4)

        # move the cube constraint to a corner of the viz region
        constraint_settings.pivot_b = viz_cube_bb.back_bottom_right
        sim.update_rigid_constraint(constraint_id, constraint_settings)
        observations += simulate(sim, 1.0, produce_debug_video)
        global_pivot_pos = cube_obj.root_scene_node.transformation.transform_point(
            constraint_settings.pivot_a
        )
        assert np.allclose(global_pivot_pos, constraint_settings.pivot_b, atol=1.0e-4)

        # switch the cube pivot to the opposite corner
        constraint_settings.pivot_a = cube_obj.collision_shape_aabb.back_bottom_right
        sim.update_rigid_constraint(constraint_id, constraint_settings)
        observations += simulate(sim, 2.0, produce_debug_video)
        global_pivot_pos = cube_obj.root_scene_node.transformation.transform_point(
            constraint_settings.pivot_a
        )
        assert np.allclose(global_pivot_pos, constraint_settings.pivot_b, atol=1.0e-4)

        # add another object and constraint them together
        cube_obj_2 = rigid_obj_mgr.add_object_by_template_handle(cube_prim_handle)
        constraint_settings_2 = habitat_sim.physics.RigidConstraintSettings()
        constraint_settings_2.object_id_a = cube_obj_2.object_id
        constraint_settings_2.object_id_b = cube_obj.object_id
        constraint_settings_2.pivot_a = (
            cube_obj_2.collision_shape_aabb.back_bottom_right
        )
        constraint_settings_2.pivot_b = cube_obj.collision_shape_aabb.front_top_left
        constraint_settings_2.max_impulse = 1000.0
        constraint_id_2 = sim.create_rigid_constraint(constraint_settings_2)
        assert constraint_id_2 >= 0
        observations += simulate(sim, 4.0, produce_debug_video)
        global_pivot_pos = cube_obj.root_scene_node.transformation.transform_point(
            constraint_settings.pivot_a
        )
        assert np.allclose(global_pivot_pos, constraint_settings.pivot_b, atol=1.0e-3)
        # both pivot corners should be at the same global position
        global_connect_a = cube_obj_2.root_scene_node.transformation.transform_point(
            constraint_settings_2.pivot_a
        )
        global_connect_b = cube_obj.root_scene_node.transformation.transform_point(
            constraint_settings_2.pivot_b
        )
        assert np.allclose(global_connect_a, global_connect_b, atol=5.0e-3)

        # weaken the world constraint
        constraint_settings.max_impulse = 0.2
        sim.update_rigid_constraint(constraint_id, constraint_settings)
        observations += simulate(sim, 2.0, produce_debug_video)
        global_pivot_pos = cube_obj.root_scene_node.transformation.transform_point(
            constraint_settings.pivot_a
        )
        assert not np.allclose(
            global_pivot_pos, constraint_settings.pivot_b, atol=1.0e-4
        )

        # check that the queried settings are reflecting updates
        queried_settings = sim.get_rigid_constraint_settings(constraint_id)
        assert queried_settings.object_id_a == constraint_settings.object_id_a
        assert queried_settings.object_id_b == constraint_settings.object_id_b
        assert queried_settings.pivot_a == constraint_settings.pivot_a
        assert queried_settings.pivot_b == constraint_settings.pivot_b
        assert queried_settings.max_impulse == constraint_settings.max_impulse
        queried_settings = sim.get_rigid_constraint_settings(constraint_id_2)
        assert queried_settings.object_id_a == constraint_settings_2.object_id_a
        assert queried_settings.object_id_b == constraint_settings_2.object_id_b
        assert queried_settings.pivot_a == constraint_settings_2.pivot_a
        assert queried_settings.pivot_b == constraint_settings_2.pivot_b
        assert queried_settings.max_impulse == constraint_settings_2.max_impulse

        assert cube_obj.translation[1] > -1.0
        assert cube_obj_2.translation[1] > -1.0
        # remove the constraint
        sim.remove_rigid_constraint(constraint_id)
        sim.remove_rigid_constraint(constraint_id_2)
        observations += simulate(sim, 2.0, produce_debug_video)
        # cubes should fall and separate
        assert cube_obj.translation[1] < -1.0
        assert cube_obj_2.translation[1] < -1.0
        global_connect_a = cube_obj_2.root_scene_node.transformation.transform_point(
            constraint_settings_2.pivot_a
        )
        global_connect_b = cube_obj.root_scene_node.transformation.transform_point(
            constraint_settings_2.pivot_b
        )
        assert not np.allclose(global_connect_a, global_connect_b, atol=0.5)

        # -----------------------------
        # test rigid fixed constraints
        # -----------------------------

        # use the same settings, but change the type
        constraint_settings.constraint_type = (
            habitat_sim.physics.RigidConstraintType.Fixed
        )
        constraint_settings_2.constraint_type = (
            habitat_sim.physics.RigidConstraintType.Fixed
        )
        constraint_id = sim.create_rigid_constraint(constraint_settings)
        constraint_id_2 = sim.create_rigid_constraint(constraint_settings_2)
        observations += simulate(sim, 2.0, produce_debug_video)

        # pivot relationship should be the same
        global_connect_a = cube_obj_2.root_scene_node.transformation.transform_point(
            constraint_settings_2.pivot_a
        )
        global_connect_b = cube_obj.root_scene_node.transformation.transform_point(
            constraint_settings_2.pivot_b
        )
        assert np.allclose(global_connect_a, global_connect_b, atol=5.0e-3)
        global_pivot_pos = cube_obj.root_scene_node.transformation.transform_point(
            constraint_settings.pivot_a
        )
        assert np.allclose(global_pivot_pos, constraint_settings.pivot_b, atol=1.0e-3)
        # default frames lock identity orientation
        assert mn.math.angle(cube_obj.rotation, mn.Quaternion()) < mn.Rad(0.01)
        assert mn.math.angle(cube_obj_2.rotation, mn.Quaternion()) < mn.Rad(0.01)

        # change the global frame rotation
        global_target_frame = mn.Quaternion.rotation(
            mn.Rad(mn.math.pi / 4.0), mn.Vector3(1.0, 0, 0)
        )
        constraint_settings.frame_b = global_target_frame.to_matrix()
        sim.update_rigid_constraint(constraint_id, constraint_settings)

        # counter rotate object frames pi/4
        local_target_frame_1 = mn.Quaternion.rotation(
            mn.Rad(mn.math.pi / 4.0), mn.Vector3(0.0, 1.0, 0)
        )
        local_target_frame_2 = mn.Quaternion.rotation(
            -mn.Rad(mn.math.pi / 4.0), mn.Vector3(0.0, 1.0, 0)
        )
        constraint_settings_2.frame_a = local_target_frame_1.to_matrix()
        constraint_settings_2.frame_b = local_target_frame_2.to_matrix()
        sim.update_rigid_constraint(constraint_id_2, constraint_settings_2)

        observations += simulate(sim, 2.0, produce_debug_video)
        # check global frame change of object 1
        assert mn.math.angle(cube_obj.rotation, global_target_frame) < mn.Rad(0.01)
        # check that relative frames of objects total pi/2
        angle_error = mn.math.angle(cube_obj.rotation, cube_obj_2.rotation) - mn.Rad(
            mn.math.pi / 4.0
        )
        assert abs(float(angle_error)) < 0.01

        # removing objects will clear constraints
        rigid_obj_mgr.remove_object_by_id(cube_obj_2.object_id)

        # object should not be moving or in contact, but should not be allowed to sleep with an active constraint
        assert cube_obj.awake
        simulate(sim, 6.0, False)
        assert cube_obj.awake

        # -----------------------------------------
        # test articulated|rigid mixed constraints
        # -----------------------------------------

        # add a humanoid to the world
        robot_file = "data/test_assets/urdf/amass_male.urdf"
        robot = art_obj_mgr.add_articulated_object_from_urdf(
            filepath=robot_file, fixed_base=False
        )
        assert robot.is_alive
        # need motors to stabalize the humanoid
        joint_motor_settings = habitat_sim.physics.JointMotorSettings()
        joint_motor_settings.position_gain = 0.5
        robot.create_all_motors(joint_motor_settings)
        # for link in range(robot.num_links):
        #    print(robot.get_link_name(link))

        # hang AO from cube with P2P
        constraint_settings_2 = habitat_sim.physics.RigidConstraintSettings()
        constraint_settings_2.object_id_a = robot.object_id
        constraint_settings_2.link_id_a = 9  # lwrist
        constraint_settings_2.object_id_b = cube_obj.object_id
        constraint_settings_2.pivot_b = cube_obj.collision_shape_aabb.front_top_left
        constraint_settings_2.max_impulse = 10000000
        constraint_id_2 = sim.create_rigid_constraint(constraint_settings_2)
        constraint_settings.max_impulse = 10000000
        sim.update_rigid_constraint(constraint_id, constraint_settings)

        observations += simulate(sim, 4.0, produce_debug_video)
        global_connect_a = robot.get_link_scene_node(
            constraint_settings_2.link_id_a
        ).transformation.transform_point(constraint_settings_2.pivot_a)
        global_connect_b = cube_obj.root_scene_node.transformation.transform_point(
            constraint_settings_2.pivot_b
        )
        assert np.allclose(global_connect_a, global_connect_b, atol=0.04)

        # hang AO from cube with Fixed constraint
        sim.remove_rigid_constraint(constraint_id_2)
        constraint_settings_2.constraint_type = (
            habitat_sim.physics.RigidConstraintType.Fixed
        )
        constraint_id_2 = sim.create_rigid_constraint(constraint_settings_2)
        observations += simulate(sim, 4.0, produce_debug_video)

        # check pivots are aligned
        global_connect_a = robot.get_link_scene_node(
            constraint_settings_2.link_id_a
        ).transformation.transform_point(constraint_settings_2.pivot_a)
        global_connect_b = cube_obj.root_scene_node.transformation.transform_point(
            constraint_settings_2.pivot_b
        )
        assert np.allclose(global_connect_a, global_connect_b, atol=0.08)

        # check that relative frames of objects near 0
        assert mn.math.angle(
            cube_obj.rotation,
            robot.get_link_scene_node(constraint_settings_2.link_id_a).rotation,
        ) < mn.Rad(0.1)

        # counter rotate object frames pi/4
        local_target_frame_1 = mn.Quaternion.rotation(
            mn.Rad(mn.math.pi / 4.0), mn.Vector3(0.0, 0, 1.0)
        )
        local_target_frame_2 = mn.Quaternion.rotation(
            -mn.Rad(mn.math.pi / 4.0), mn.Vector3(0.0, 0, 1.0)
        )
        constraint_settings_2.frame_a = local_target_frame_1.to_matrix()
        constraint_settings_2.frame_b = local_target_frame_2.to_matrix()
        sim.update_rigid_constraint(constraint_id_2, constraint_settings_2)

        observations += simulate(sim, 4.0, produce_debug_video)
        # check frames align
        angle_error = mn.math.angle(
            cube_obj.rotation,
            robot.get_link_scene_node(constraint_settings_2.link_id_a).rotation,
        ) - mn.Rad(mn.math.pi / 4.0)
        assert abs(float(angle_error)) < 0.05

        # remove cube and AO should fall
        assert robot.translation[1] > -2
        rigid_obj_mgr.remove_object_by_id(cube_obj.object_id)
        observations += simulate(sim, 1.0, produce_debug_video)
        assert robot.translation[1] < -3

        # hang AO from the world with P2P
        constraint_settings_2.object_id_b = -1
        constraint_settings_2.constraint_type = (
            habitat_sim.physics.RigidConstraintType.PointToPoint
        )
        constraint_id_2 = sim.create_rigid_constraint(constraint_settings_2)
        observations += simulate(sim, 7.0, produce_debug_video)

        # check pivots are aligned
        global_connect_a = robot.get_link_scene_node(
            constraint_settings_2.link_id_a
        ).transformation.transform_point(constraint_settings_2.pivot_a)
        assert np.allclose(global_connect_a, constraint_settings_2.pivot_b, atol=0.04)

        # hang AO from world Fixed
        sim.remove_rigid_constraint(constraint_id_2)
        constraint_settings_2.constraint_type = (
            habitat_sim.physics.RigidConstraintType.Fixed
        )
        # counter rotate object frames pi/4
        local_target_frame_1 = mn.Quaternion.rotation(
            mn.Rad(mn.math.pi / 4.0), mn.Vector3(0, 1.0, 0)
        )
        local_target_frame_2 = mn.Quaternion.rotation(
            -mn.Rad(mn.math.pi / 4.0), mn.Vector3(0, 1.0, 0)
        )
        constraint_settings_2.frame_a = local_target_frame_1.to_matrix()
        constraint_settings_2.frame_b = local_target_frame_2.to_matrix()
        constraint_id_2 = sim.create_rigid_constraint(constraint_settings_2)
        observations += simulate(sim, 5.0, produce_debug_video)

        # check pivots are aligned
        global_connect_a = robot.get_link_scene_node(
            constraint_settings_2.link_id_a
        ).transformation.transform_point(constraint_settings_2.pivot_a)
        assert np.allclose(global_connect_a, constraint_settings_2.pivot_b, atol=0.08)

        # check frames align
        angle_error = mn.math.angle(
            local_target_frame_2,
            robot.get_link_scene_node(constraint_settings_2.link_id_a).rotation,
        ) - mn.Rad(mn.math.pi / 4.0)
        assert abs(float(angle_error)) < 0.2

        # hang the object from its base link
        constraint_settings_2.link_id_a = -1
        sim.remove_rigid_constraint(constraint_id_2)
        constraint_id_2 = sim.create_rigid_constraint(constraint_settings_2)
        observations += simulate(sim, 5.0, produce_debug_video)
        angle_error = mn.math.angle(local_target_frame_2, robot.rotation) - mn.Rad(
            mn.math.pi / 4.0
        )
        # NOTE: This error is a bit high, but constraint is doing its best
        assert abs(float(angle_error)) < 0.4
        # check pivots are aligned
        global_connect_a = robot.get_link_scene_node(
            constraint_settings_2.link_id_a
        ).transformation.transform_point(constraint_settings_2.pivot_a)
        assert np.allclose(robot.translation, constraint_settings_2.pivot_b, atol=0.08)
        sim.remove_rigid_constraint(constraint_id_2)

        # -----------------------------------------
        # test articulated constraints
        # -----------------------------------------

        # AO - AO P2P
        robot.clear_joint_states()
        robot.translation = [0.775, 0.0, 0.0]
        robot.rotation = mn.Quaternion()
        # add a new robot with a fixed base
        robot2 = art_obj_mgr.add_articulated_object_from_urdf(
            filepath=robot_file, fixed_base=True
        )
        robot2.translation = [-0.775, 0.0, 0.0]
        robot2.create_all_motors(joint_motor_settings)

        constraint_settings_2 = habitat_sim.physics.RigidConstraintSettings()
        constraint_settings_2.object_id_a = robot.object_id
        constraint_settings_2.link_id_a = 15  # rwrist
        constraint_settings_2.pivot_a = [-0.04, 0.0, 0.0]
        constraint_settings_2.object_id_b = robot2.object_id
        constraint_settings_2.link_id_b = 9  # lwrist
        constraint_settings_2.pivot_b = [0.04, 0.0, 0.0]
        constraint_settings_2.max_impulse = 10000000
        constraint_id_2 = sim.create_rigid_constraint(constraint_settings_2)

        observations += simulate(sim, 5.0, produce_debug_video)

        # check pivots are aligned
        global_connect_a = robot.get_link_scene_node(
            constraint_settings_2.link_id_a
        ).transformation.transform_point(constraint_settings_2.pivot_a)
        global_connect_b = robot2.get_link_scene_node(
            constraint_settings_2.link_id_b
        ).transformation.transform_point(constraint_settings_2.pivot_b)
        assert np.allclose(global_connect_a, global_connect_b, atol=0.08)

        # switch to fixed constraint
        sim.remove_rigid_constraint(constraint_id_2)
        constraint_settings_2.constraint_type = (
            habitat_sim.physics.RigidConstraintType.Fixed
        )
        constraint_id_2 = sim.create_rigid_constraint(constraint_settings_2)

        observations += simulate(sim, 5.0, produce_debug_video)

        # check pivots are aligned
        global_connect_a = robot.get_link_scene_node(
            constraint_settings_2.link_id_a
        ).transformation.transform_point(constraint_settings_2.pivot_a)
        global_connect_b = robot2.get_link_scene_node(
            constraint_settings_2.link_id_b
        ).transformation.transform_point(constraint_settings_2.pivot_b)
        assert np.allclose(global_connect_a, global_connect_b, atol=0.08)

        # check frames align
        angle_error = mn.math.angle(
            robot.get_link_scene_node(constraint_settings_2.link_id_a).rotation,
            robot2.get_link_scene_node(constraint_settings_2.link_id_b).rotation,
        )
        # NOTE: This error is a bit high, but constraint is doing its best
        assert abs(float(angle_error)) < 0.37

        # produce some test debug video
        if produce_debug_video:
            from habitat_sim.utils import viz_utils as vut

            vut.make_video(
                observations,
                "color_sensor",
                "color",
                "test_rigid_constraints",
                open_vid=True,
            )


@pytest.mark.skipif(
    not osp.exists("data/scene_datasets/habitat-test-scenes/apartment_1.glb"),
    reason="Requires the habitat-test-scenes",
)
@pytest.mark.skipif(
    not habitat_sim.bindings.built_with_bullet,
    reason="ArticulatedObject API requires Bullet physics.",
)
def test_bullet_collision_helper():
    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()
    cfg_settings["scene"] = "data/scene_datasets/habitat-test-scenes/apartment_1.glb"
    cfg_settings["enable_physics"] = True

    # loading the physical scene
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)

    with habitat_sim.Simulator(hab_cfg) as sim:
        obj_template_mgr = sim.get_object_template_manager()
        cube_prim_handle = obj_template_mgr.get_template_handles("cube")[0]
        rigid_obj_mgr = sim.get_rigid_object_manager()
        cube_obj = rigid_obj_mgr.add_object_by_template_handle(cube_prim_handle)
        cube_obj.translation = [2.5, 1.5, 2.5]

        sim.step_physics(0.01)

        assert sim.get_physics_num_active_contact_points() == 0
        assert sim.get_physics_num_active_overlapping_pairs() == 0
        assert (
            sim.get_physics_step_collision_summary()
            == "(no active collision manifolds)\n"
        )

        sim.step_physics(0.25)

        assert sim.get_physics_num_active_contact_points() == 4
        assert sim.get_physics_num_active_overlapping_pairs() == 1
        assert (
            sim.get_physics_step_collision_summary()
            == "[RigidObject, cubeSolid, id 0] vs [Stage, subpart 0], 4 points\n"
        )

        sim.step_physics(3.0)

        assert sim.get_physics_num_active_contact_points() == 0
        assert sim.get_physics_num_active_overlapping_pairs() == 0
        assert (
            sim.get_physics_step_collision_summary()
            == "(no active collision manifolds)\n"
        )

        rigid_obj_mgr.remove_object_by_id(cube_obj.object_id)

        art_obj_mgr = sim.get_articulated_object_manager()
        robot_file = "data/test_assets/urdf/fridge/fridge.urdf"

        # parse URDF and add an ArticulatedObject to the world
        robot = art_obj_mgr.add_articulated_object_from_urdf(filepath=robot_file)
        assert robot.is_alive

        robot.translation = mn.Vector3(2.5, 4.0, 2.5)
        robot.rotation = mn.Quaternion()

        sim.step_physics(0.01)

        assert sim.get_physics_num_active_contact_points() == 0
        assert sim.get_physics_num_active_overlapping_pairs() == 0
        assert (
            sim.get_physics_step_collision_summary()
            == "(no active collision manifolds)\n"
        )

        sim.step_physics(0.75)

        assert sim.get_physics_num_active_contact_points() == 2
        # lots of overlapping pairs due to various fridge links near the stage
        assert sim.get_physics_num_active_overlapping_pairs() == 5
        assert (
            sim.get_physics_step_collision_summary()
            == "[URDF, fridge, link body] vs [Stage, subpart 0], 2 points\n"
        )

        sim.step_physics(3.0)

        assert sim.get_physics_num_active_contact_points() == 0
        assert sim.get_physics_num_active_overlapping_pairs() == 0
        assert (
            sim.get_physics_step_collision_summary()
            == "(no active collision manifolds)\n"
        )
