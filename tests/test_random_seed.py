import os
import sys

import examples.settings
import habitat_sim

cwd = os.getcwd()
if cwd not in sys.path:
    sys.path.append(cwd)


def test_random_seed():
    # put the path to your test scene here. You MUST define a scene instead of
    # just the default otherwise this will segfault
    test_scene = "./data/scene_datasets/habitat-test-scenes/van-gogh-room.glb"
    cfg_settings = examples.settings.default_sim_settings.copy()

    # keyword "NONE" initializes a scene with no scene mesh
    cfg_settings["scene"] = test_scene
    hab_cfg = examples.settings.make_cfg(cfg_settings)
    sim = habitat_sim.Simulator(hab_cfg)

    # Test that the same seed gives the same point
    sim.seed(1)
    point1 = sim.pathfinder.get_random_navigable_point()
    sim.seed(1)
    point2 = sim.pathfinder.get_random_navigable_point()
    assert all(point1 == point2)

    # Test that different seeds give different points
    sim.seed(2)
    point1 = sim.pathfinder.get_random_navigable_point()
    sim.seed(3)
    point2 = sim.pathfinder.get_random_navigable_point()
    assert any(point1 != point2)

    # Test that the same seed gives different points when sampled twice
    sim.seed(4)
    point1 = sim.pathfinder.get_random_navigable_point()
    point2 = sim.pathfinder.get_random_navigable_point()
    assert any(point1 != point2)
