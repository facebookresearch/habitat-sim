import os.path as osp
import random

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
    # enable the physics simulator
    cfg_settings["enable_physics"] = True
    cfg_settings["depth_sensor"] = True

    hab_cfg = examples.settings.make_cfg(cfg_settings)
    sim.reconfigure(hab_cfg)

    # add an object

    for _ in range(10):
        # do kinematics here
        obs = sim.step(random.choice(list(hab_cfg.agents[0].action_space.keys())))
