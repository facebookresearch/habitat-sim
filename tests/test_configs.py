# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import magnum as mn
import numpy as np

import habitat_sim
from habitat_sim.bindings import ConfigStoredType, Configuration


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
    config = Configuration()
    config.set("test", "test statement")
    assert config.has_value("test")
    assert config.get("test") == "test statement"

    config.remove("test")
    assert not config.has_value("test")

    config.set("bool", np.array(True))
    assert config.get("bool") == True

    config.set("integer", 3)
    assert config.get("integer") == 3

    my_float = 0.77777777777777
    config.set("py_float", my_float)
    assert config.get("py_float") == my_float

    # Magnum::Vector2 (float)
    my_vec2 = np.array([1.12345, 2.0])
    config.set("vec2", my_vec2)
    assert config.get("vec2") == my_vec2

    # Magnum::Vector3 (float)
    my_vec3 = np.array([1.12345, 2.0, -3.0])
    config.set("vec3", my_vec3)
    assert config.get("vec3") == my_vec3

    # Magnum::Vector4 (float)
    my_vec4 = np.array([1.12345, 2.0, -3.0, 4.32])
    config.set("vec4", my_vec4)
    assert config.get("vec4") == my_vec4

    # Magnum::Matrix3 (3x3)
    my_mat3x3 = mn.Matrix3((1.1, 2.2, -3.3), (4.4, 5.0, -6.6), (7.7, 8.0, -9.9))
    config.set("mat3x3", my_mat3x3)
    assert config.get("mat3x3") == my_mat3x3

    # Magnum::Quaternion (float)
    my_quat = mn.Quaternion(((1.12345, 2.0, -3.0), 4.0))
    config.set("quat", my_quat)
    assert config.get("quat") == my_quat

    # test subconfig editing and find
    # create subconfig with passed name and edit it directly
    # changes are saved
    subconfig_0 = config.get_subconfig("subconfig_0")
    # create subconfig_1 as child of subconfig_0 and edit directly
    subconfig_1 = subconfig_0.get_subconfig("subconfig_1")
    # etc.
    subconfig_2 = subconfig_1.get_subconfig("subconfig_2")
    subconfig_3 = subconfig_2.get_subconfig("subconfig_3")
    subconfig_4 = subconfig_3.get_subconfig("subconfig_4")
    subconfig_5 = subconfig_4.get_subconfig("subconfig_5")

    # add a string to find to deepest nested subconfig
    string_to_add = "this is a string to find"
    subconfig_5.set("string_to_find", string_to_add)
    assert subconfig_5.get("string_to_find") == string_to_add
    # now search config tree to find "string_to_find"
    breadcrumbs = config.find_value_location("string_to_find")
    # make sure breadcrumbs array returns 7 values, 6 subconfig keys, in order, plus the key we are looking for
    assert len(breadcrumbs) == 7
    assert breadcrumbs[0] == "subconfig_0"
    assert breadcrumbs[1] == "subconfig_1"
    assert breadcrumbs[2] == "subconfig_2"
    assert breadcrumbs[3] == "subconfig_3"
    assert breadcrumbs[4] == "subconfig_4"
    assert breadcrumbs[5] == "subconfig_5"
    assert breadcrumbs[6] == "string_to_find"

    # get value
    # first get to last subconfig
    subconfig_to_check = config
    for i in range(len(breadcrumbs) - 1):
        subconfig_to_check = subconfig_to_check.get_subconfig(breadcrumbs[i])
    # check at final subconfig
    assert subconfig_to_check.has_key_to_type(breadcrumbs[-1], ConfigStoredType.String)
    # check value is as expected
    assert subconfig_to_check.get(breadcrumbs[-1]) == string_to_add

    # add another value to a different subconfig
    subconfig_31 = subconfig_2.get_subconfig("subconfig_31")
    # set value
    vec_to_add = np.array([11.12345, 12.0, -13.0])
    subconfig_31.set("vec3_to_find", vec_to_add)
    assert subconfig_31.get("vec3_to_find") == vec_to_add
    # find breadcrumbs to this value
    breadcrumbs2 = config.find_value_location("vec3_to_find")
    assert len(breadcrumbs2) == 5
    assert breadcrumbs2[0] == "subconfig_0"
    assert breadcrumbs2[1] == "subconfig_1"
    assert breadcrumbs2[2] == "subconfig_2"
    assert breadcrumbs2[3] == "subconfig_31"
    assert breadcrumbs2[4] == "vec3_to_find"

    # get value
    # first get to last subconfig
    subconfig_to_check = config
    for i in range(len(breadcrumbs2) - 1):
        subconfig_to_check = subconfig_to_check.get_subconfig(breadcrumbs2[i])
    # check at final subconfig
    assert subconfig_to_check.has_key_to_type(
        breadcrumbs2[-1], ConfigStoredType.MagnumVec3
    )
    # check value is as expected
    assert subconfig_to_check.get(breadcrumbs2[-1]) == vec_to_add

    # remove nested subconfig
    nested_subconfig = config.remove_subconfig("subconfig_0")
    assert not config.has_subconfig("subconfig_0")
    assert nested_subconfig.has_subconfig("subconfig_1")

    # test subconfig create/modify/save
    # creates a new subconfig_new, puts it in config's subconfigs, and returns a copy
    subconfig_new = config.get_subconfig_copy("subconfig_new")
    subconfig_new.set("test_string", "this is a test string")

    assert subconfig_new.has_key_to_type("test_string", ConfigStoredType.String)
    assert not config.get_subconfig("subconfig_new").has_key_to_type(
        "test_string", ConfigStoredType.String
    )

    # now adde subconfig into config
    config.save_subconfig("subconfig_new", subconfig_new)
    # now nested subconfig should have key
    assert config.get_subconfig("subconfig_new").has_key_to_type(
        "test_string", ConfigStoredType.String
    )


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
    object_template.rolling_friction_coefficient = 1.456
    assert object_template.rolling_friction_coefficient == 1.456
    object_template.spinning_friction_coefficient = 1.567
    assert object_template.spinning_friction_coefficient == 1.567
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
    object_template.force_flat_shading = True
    assert object_template.force_flat_shading == True
