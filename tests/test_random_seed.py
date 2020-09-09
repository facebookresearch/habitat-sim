import examples.settings


def test_random_seed(sim):
    # reconfigure to ensure pathfinder exists
    cfg_settings = examples.settings.default_sim_settings.copy()
    hab_cfg = examples.settings.make_cfg(cfg_settings)
    sim.reconfigure(hab_cfg)

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
