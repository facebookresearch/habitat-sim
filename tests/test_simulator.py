import random

import examples.settings
import habitat_sim


def test_no_navmesh_smoke():
    sim_cfg = habitat_sim.SimulatorConfiguration()
    agent_config = habitat_sim.AgentConfiguration()
    # No sensors as we are only testing to see if things work
    # with no navmesh and the navmesh isn't used for any exisitng sensors
    agent_config.sensor_specifications = []

    sim_cfg.scene.id = "data/scene_datasets/habitat-test-scenes/van-gogh-room.glb"
    # Make it try to load a navmesh that doesn't exists
    sim_cfg.scene.filepaths["navmesh"] = "/tmp/dne.navmesh"

    sim = habitat_sim.Simulator(habitat_sim.Configuration(sim_cfg, [agent_config]))

    sim.initialize_agent(0)

    random.seed(0)
    for _ in range(50):
        obs = sim.step(random.choice(list(agent_config.action_space.keys())))
        # Can't collide with no navmesh
        assert not obs["collided"]


def test_empty_scene(sim):
    cfg_settings = examples.settings.default_sim_settings.copy()

    # keyword "NONE" initializes a scene with no scene mesh
    cfg_settings["scene"] = "NONE"
    # test that depth sensor doesn't mind an empty scene
    cfg_settings["depth_sensor"] = True

    hab_cfg = examples.settings.make_cfg(cfg_settings)
    sim.reconfigure(hab_cfg)

    # test that empty frames can be rendered without a scene mesh
    for _ in range(2):
        obs = sim.step(random.choice(list(hab_cfg.agents[0].action_space.keys())))
