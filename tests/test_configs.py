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


def test_physics_object_attributes():
    object_template = habitat_sim.attributes.ObjectAttributes()

    my_test_vec = np.array([1.0, 2.0, 3.0])
    object_template.com = my_test_vec
    assert object_template.com == my_test_vec
    object_template.margin = 1.123
    assert object_template.margin == 1.123
    object_template.mass = 1.234
    assert object_template.mass == 1.234
    object_template.inertia = my_test_vec
    assert object_template.inertia == my_test_vec
    object_template.scale = my_test_vec
    assert object_template.scale == my_test_vec
    object_template.friction_coefficient = 1.345
    assert object_template.friction_coefficient == 1.345
    object_template.restitution_coefficient = 1.456
    assert object_template.restitution_coefficient == 1.456
    object_template.linear_damping = 1.567
    assert object_template.linear_damping == 1.567
    object_template.angular_damping = 1.678
    assert object_template.angular_damping == 1.678
    object_template.handle = "origin"
    assert object_template.handle == "origin"
    object_template.render_asset_handle = "render_mesh"
    assert object_template.render_asset_handle == "render_mesh"
    object_template.collision_asset_handle = "collision_mesh"
    assert object_template.collision_asset_handle == "collision_mesh"
    object_template.bounding_box_collisions = True
    assert object_template.bounding_box_collisions == True
    object_template.join_collision_meshes = False
    assert object_template.join_collision_meshes == False
    object_template.requires_lighting = False
    assert object_template.requires_lighting == False

    # test that inheritance is correctly configured
    object_template.set("test_key", "test_string")
    assert object_template.get_string("test_key") == "test_string"
