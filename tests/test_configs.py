import numpy as np

import habitat_sim


def test_config_eq():
    cfg1 = habitat_sim.Configuration(
        habitat_sim.SimulatorConfiguration(), [habitat_sim.AgentConfiguration()]
    )
    cfg2 = habitat_sim.Configuration(
        habitat_sim.SimulatorConfiguration(), [habitat_sim.AgentConfiguration()]
    )

    assert cfg1 == cfg2


def test_core_configuration():
    # test bindings for esp::core::Configuration class
    config = habitat_sim.bindings.ConfigurationGroup()
    config.set("test", "test statement")
    assert config.has_value("test")
    assert config.get_string("test") == "test statement"

    config.remove_value("test")
    assert not config.has_value("test")

    config.set("bool", np.array(True))
    assert config.get_bool("bool") == True

    config.set("integer", 3)
    assert config.get("integer") == "3"
    assert config.get_int("integer") == 3

    my_double = 0.77777777777777
    config.set("double", my_double)
    assert config.get_double("double") == my_double
    assert config.get_int("double") == int(my_double)

    # Magnum::Vector3 (float)
    my_vec3 = np.array([1.12345, 2.0, -3.0])
    config.set("vec3", my_vec3)
    assert config.get_vec3("vec3") == my_vec3
    assert config.get_int("vec3") == int(my_vec3[0])

    # test string group
    text_group = ["a", "b", "  c", "12", "0.1", "-=_+.,';:"]
    for text in text_group:
        config.add_string_to_group("text_group", text)

    queried_group = config.get_string_group("text_group")
    for ix, text in enumerate(queried_group):
        assert text == text_group[ix]
