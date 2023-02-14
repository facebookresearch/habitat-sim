# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import habitat_sim
import habitat_sim.utils.settings


def test_random_seed():
    # reconfigure to ensure pathfinder exists
    cfg_settings = habitat_sim.utils.settings.default_sim_settings.copy()
    cfg_settings[
        "scene"
    ] = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    hab_cfg = habitat_sim.utils.settings.make_cfg(cfg_settings)
    with habitat_sim.Simulator(hab_cfg) as sim:
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
