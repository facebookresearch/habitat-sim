import os.path as osp
import random

import numpy as np
import pytest
import quaternion

import examples.settings
import habitat_sim.physics
from habitat_sim.utils.common import (
    quat_from_angle_axis,
    quat_from_magnum,
    quat_to_magnum,
)


@pytest.mark.skipif(
    not osp.exists("data/scene_datasets/habitat-test-scenes/skokloster-castle.glb")
    or not osp.exists("data/objects/"),
    reason="Requires the habitat-test-scenes and habitat test objects",
)
def test_kinematics(sim):
    cfg_settings = examples.settings.default_sim_settings.copy()

    cfg_settings[
        "scene"
    ] = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    # enable the physics simulator: also clears available actions to no-op
    cfg_settings["enable_physics"] = True
    cfg_settings["depth_sensor"] = True

    # test loading the physical scene
    hab_cfg = examples.settings.make_cfg(cfg_settings)
    sim.reconfigure(hab_cfg)
    assert sim.get_physics_object_library_size() > 0

    # test adding an object to the world
    object_id = sim.add_object(0)
    assert len(sim.get_existing_object_ids()) > 0

    # test setting the motion type

    assert sim.set_object_motion_type(habitat_sim.physics.MotionType.STATIC, object_id)
    assert (
        sim.get_object_motion_type(object_id) == habitat_sim.physics.MotionType.STATIC
    )
    assert sim.set_object_motion_type(
        habitat_sim.physics.MotionType.KINEMATIC, object_id
    )
    assert (
        sim.get_object_motion_type(object_id)
        == habitat_sim.physics.MotionType.KINEMATIC
    )

    # test kinematics
    I = np.identity(4)

    # test get and set translation
    sim.set_translation(np.array([0, 1.0, 0]), object_id)
    assert np.allclose(sim.get_translation(object_id), np.array([0, 1.0, 0]))

    # test get and set transform
    sim.set_transformation(I, object_id)
    assert np.allclose(sim.get_transformation(object_id), I)

    # test get and set rotation
    Q = quat_from_angle_axis(np.pi, np.array([0, 1.0, 0]))
    expected = np.eye(4)
    expected[0:3, 0:3] = quaternion.as_rotation_matrix(Q)
    sim.set_rotation(quat_to_magnum(Q), object_id)
    assert np.allclose(sim.get_transformation(object_id), expected)
    assert np.allclose(quat_from_magnum(sim.get_rotation(object_id)), Q)

    # test object removal
    old_object_id = sim.remove_object(object_id)
    assert len(sim.get_existing_object_ids()) == 0
    assert old_object_id == object_id

    object_id = sim.add_object(0)

    prev_time = 0.0
    for _ in range(2):
        # do some kinematics here (todo: translating or rotating instead of absolute)
        sim.set_translation(np.random.rand(3), object_id)
        T = sim.get_transformation(object_id)

        # test getting observation
        obs = sim.step(random.choice(list(hab_cfg.agents[0].action_space.keys())))

        # check that time is increasing in the world
        assert sim.get_world_time() > prev_time
        prev_time = sim.get_world_time()


@pytest.mark.skipif(
    not osp.exists("data/scene_datasets/habitat-test-scenes/skokloster-castle.glb")
    or not osp.exists("data/objects/"),
    reason="Requires the habitat-test-scenes and habitat test objects",
)
def test_dynamics(sim):
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
    sim.reconfigure(hab_cfg)
    # make the simulation deterministic (C++ seed is set in reconfigure)
    np.random.seed(cfg_settings["seed"])
    assert sim.get_physics_object_library_size() > 0

    # test adding an object to the world
    object_id = sim.add_object(0)
    object2_id = sim.add_object(0)
    assert len(sim.get_existing_object_ids()) > 0

    # place the objects over the table in room
    sim.set_translation(np.array([-0.569043, 2.04804, 13.6156]), object_id)
    sim.set_translation(np.array([-0.569043, 2.04804, 12.6156]), object2_id)

    # get object MotionType and continue testing if MotionType::DYNAMIC (implies a physics implementation is active)
    if sim.get_object_motion_type(object_id) == habitat_sim.physics.MotionType.DYNAMIC:
        previous_object_states = [
            [sim.get_translation(object_id), sim.get_rotation(object_id)],
            [sim.get_translation(object2_id), sim.get_rotation(object2_id)],
        ]
        prev_time = 0.0
        for _ in range(50):
            # force application at a location other than the origin should always cause angular and linear motion
            sim.apply_force(np.random.rand(3), np.random.rand(3), object2_id)

            # TODO: expose object properties (such as mass) to python
            # Counter the force of gravity on the object (it should not translate)
            obj_mass = 0.5
            sim.apply_force(np.array([0, obj_mass * 9.8, 0]), np.zeros(3), object_id)

            # apply torque to the "floating" object. It should rotate, but not translate
            sim.apply_torque(np.random.rand(3), object_id)

            # TODO: test other physics functions

            # test getting observation
            obs = sim.step(random.choice(list(hab_cfg.agents[0].action_space.keys())))

            # check that time is increasing in the world
            assert sim.get_world_time() > prev_time
            prev_time = sim.get_world_time()

            # check the object states
            # 1st object should rotate, but not translate
            assert previous_object_states[0][0] == sim.get_translation(object_id)
            assert previous_object_states[0][1] != sim.get_rotation(object_id)

            # 2nd object should rotate and translate
            assert previous_object_states[1][0] != sim.get_translation(object2_id)
            assert previous_object_states[1][1] != sim.get_rotation(object2_id)

            previous_object_states = [
                [sim.get_translation(object_id), sim.get_rotation(object_id)],
                [sim.get_translation(object2_id), sim.get_rotation(object2_id)],
            ]

        # test setting DYNAMIC object to KINEMATIC
        assert sim.set_object_motion_type(
            habitat_sim.physics.MotionType.KINEMATIC, object2_id
        )
        assert (
            sim.get_object_motion_type(object2_id)
            == habitat_sim.physics.MotionType.KINEMATIC
        )

        obs = sim.step(random.choice(list(hab_cfg.agents[0].action_space.keys())))

        # 2nd object should no longer rotate or translate
        assert previous_object_states[1][0] == sim.get_translation(object2_id)
        assert previous_object_states[1][1] == sim.get_rotation(object2_id)
