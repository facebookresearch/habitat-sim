import habitat_sim


def test_config_eq():
    cfg1 = habitat_sim.Configuration(
        habitat_sim.SimulatorConfiguration(), [habitat_sim.AgentConfiguration()]
    )
    cfg2 = habitat_sim.Configuration(
        habitat_sim.SimulatorConfiguration(), [habitat_sim.AgentConfiguration()]
    )

    assert cfg1 == cfg2
