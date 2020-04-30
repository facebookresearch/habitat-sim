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
    physics_object_template = habitat_sim.attributes.PhysicsObjectAttributes()

    my_test_vec = np.array([1.0, 2.0, 3.0])
    physics_object_template.set_com(my_test_vec)
    assert physics_object_template.get_com() == my_test_vec
    physics_object_template.set_margin(1.123)
    assert physics_object_template.get_margin() == 1.123
    physics_object_template.set_mass(1.234)
    assert physics_object_template.get_mass() == 1.234
    physics_object_template.set_inertia(my_test_vec)
    assert physics_object_template.get_inertia() == my_test_vec
    physics_object_template.set_scale(my_test_vec)
    assert physics_object_template.get_scale() == my_test_vec
    physics_object_template.set_friction_coefficient(1.345)
    assert physics_object_template.get_friction_coefficient() == 1.345
    physics_object_template.set_restitution_coefficient(1.456)
    assert physics_object_template.get_restitution_coefficient() == 1.456
    physics_object_template.set_linear_damping(1.567)
    assert physics_object_template.get_linear_damping() == 1.567
    physics_object_template.set_angular_damping(1.678)
    assert physics_object_template.get_angular_damping() == 1.678
    physics_object_template.set_origin_handle("origin")
    assert physics_object_template.get_origin_handle() == "origin"
    physics_object_template.set_render_asset_handle("render_mesh")
    assert physics_object_template.get_render_asset_handle() == "render_mesh"
    physics_object_template.set_collision_asset_handle("collision_mesh")
    assert physics_object_template.get_collision_asset_handle() == "collision_mesh"
    physics_object_template.set_bounding_box_collisions(True)
    assert physics_object_template.get_bounding_box_collisions() == True
    physics_object_template.set_join_collision_meshes(False)
    assert physics_object_template.get_join_collision_meshes() == False
    physics_object_template.set_requires_lighting(False)
    assert physics_object_template.get_requires_lighting() == False

    # test that inheritance is correctly configured
    physics_object_template.set("test_key", "test_string")
    assert physics_object_template.get_string("test_key") == "test_string"
