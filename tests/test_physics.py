import os.path as osp
import random

import numpy as np
import pytest

import examples.settings
import habitat_sim


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

    hab_cfg = examples.settings.make_cfg(cfg_settings)
    sim.reconfigure(hab_cfg)

    # add an object to the world
    object_id = sim.add_object(0)

    # test kinematics
    sim.set_translation(np.array([0, 1.0, 0]), object_id)
    assert sim.get_translation(object_id) == np.array([0, 1.0, 0])

    # M = np.array([[]])
    # sim.set_transformation(object_id)
    # sim.set_r

    Q = habitat_sim.utils.quat_from_angle_axis(90, np.array([0, 1.0, 0]))
    print(Q)
    sim.set_rotation(Q, object_id)
    print(sim.get_rotation(object_id))
    print(sim.get_transformation(object_id))

    prev_time = 0.0
    for _ in range(2):
        # do kinematics here
        sim.set_translation(np.random.rand(3), object_id)
        T = sim.get_transformation(object_id)

        # get observation
        obs = sim.step(random.choice(list(hab_cfg.agents[0].action_space.keys())))

        # check that time is increasing in the world
        assert sim.get_world_time() > prev_time
        prev_time = sim.get_world_time()
