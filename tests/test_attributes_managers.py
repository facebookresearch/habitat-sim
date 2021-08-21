#!/usr/bin/env python3

import magnum as mn

import examples.settings
import habitat_sim


def perform_general_tests(attr_mgr, search_string):
    # get size of template library
    orig_num_templates = attr_mgr.get_num_templates()
    # make sure this is same value as size of template handles list
    assert orig_num_templates > 0

    # search for attributes with expected string
    template_handles = attr_mgr.get_template_handles(search_string)
    # verify handle exists
    assert len(template_handles) > 0

    template_handle = template_handles[0]
    # get id from handle
    template_id = attr_mgr.get_template_id_by_handle(template_handle)
    assert search_string in template_handle

    # verify both handle and id exist in the manager
    assert attr_mgr.get_library_has_handle(template_handle)
    assert attr_mgr.get_library_has_id(template_id)

    # verify that access is the same for ID and handle lookup
    template0 = attr_mgr.get_template_by_handle(template_handle)
    template1 = attr_mgr.get_template_by_id(template_id)

    # verify template 0 and template 1 are copies of the same template
    assert template0.handle == template1.handle
    assert template0.template_id == template1.template_id

    # modify template, register, then verify that
    # retrieved template is not same as previous template
    template0.set("test_key", "template0_test")
    template1.set("test_key", "template1_test")

    # save modified template
    attr_mgr.register_template(template0, template_handle)
    # retrieve registered template
    template2 = attr_mgr.get_template_by_handle(template_handle)

    # verify templates have the same identifiers
    assert template1.handle == template2.handle
    assert template1.template_id == template2.template_id

    # verify the templates hold different data and are not the
    # same object
    assert template1.get_string("test_key") != template2.get_string("test_key")
    # verify 0 and 2 hold same user-set value
    assert template0.get_string("test_key") == template2.get_string("test_key")

    # change retrieved template, verify it is not same object as template0
    template2.set("test_key", "template2_test")

    # verify the templates hold different data and are not the
    # same object
    assert template0.get_string("test_key") != template2.get_string("test_key")

    # add new template with specified handle
    new_template_handle = "new_template_0"

    attr_mgr.register_template(template0, new_template_handle)

    # register new template and verify size is greater than original
    curr_num_templates = attr_mgr.get_num_templates()
    assert curr_num_templates != orig_num_templates

    # lock template
    attr_mgr.set_template_lock(new_template_handle, True)
    # attempt to delete
    template3 = attr_mgr.remove_template_by_handle(new_template_handle)
    # verify that template was not deleted
    assert template3 == None
    # unlock template
    attr_mgr.set_template_lock(new_template_handle, False)

    # remove template that has been unlocked; retrieves removed template
    template3 = attr_mgr.remove_template_by_handle(new_template_handle)
    # verify not NONE
    assert template3 != None
    # verify has expected handle
    assert template3.handle == new_template_handle

    # get new size of library after remove and verify same as original
    curr_num_templates = attr_mgr.get_num_templates()
    assert curr_num_templates == orig_num_templates

    # add many templates with specified handles, then remove them
    new_handle_stub = "new_template_"
    num_to_add = 10
    for i in range(num_to_add):
        new_iter_handle = new_handle_stub + str(i)
        tmplt_id = attr_mgr.register_template(template3, new_iter_handle)
        assert tmplt_id != -1

    # retrieve a dictionary of all templates that were
    # just created, using search on handle stub substring
    new_template_dict = attr_mgr.get_templates_by_handle_substring(new_handle_stub)
    assert len(new_template_dict) == num_to_add
    for k, v in new_template_dict.items():
        assert k == v.handle

    # lock all added templates
    locked_template_handles = attr_mgr.set_lock_by_substring(
        True, new_handle_stub, True
    )
    # verify that the number of added and locked templates are equal
    assert num_to_add == len(locked_template_handles)
    # attempt to remove templates that are locked
    removed_templates = attr_mgr.remove_templates_by_str(new_handle_stub, True)
    # verify that no templates were removed that have the new handle stub
    assert len(removed_templates) == 0
    # unlock all added templates
    unlocked_template_handles = attr_mgr.set_lock_by_substring(
        False, new_handle_stub, True
    )
    # verify that the number of added and unlocked templates are equal
    assert num_to_add == len(unlocked_template_handles)
    # verify lists of names are same
    assert sorted(unlocked_template_handles) == sorted(locked_template_handles)

    # now delete all templates with handle stub
    removed_templates = attr_mgr.remove_templates_by_str(new_handle_stub, True)
    # verify that the number of added and removed templates are equal
    assert num_to_add == len(removed_templates)

    # get new size of library after remove and verify same as original
    curr_num_templates = attr_mgr.get_num_templates()
    assert curr_num_templates == orig_num_templates

    return template0, template3


def perform_add_blank_template_test(attr_mgr, valid_render_handle=None):
    # get size of template library
    orig_num_templates = attr_mgr.get_num_templates()

    # add new blank template
    new_template_handle = "new template"

    # create new default template, do not register it
    new_template0 = attr_mgr.create_new_template(new_template_handle, False)

    # change new template field
    new_template0.set("test_key", "new_template_test")

    # give new template valid render asset handle, otherwise registration might fail
    if valid_render_handle is not None:
        new_template0.render_asset_handle = valid_render_handle

    # add new template
    attr_mgr.register_template(new_template0, new_template_handle)

    # get new size of library after remove and verify not same as original
    curr_num_templates = attr_mgr.get_num_templates()
    assert curr_num_templates > orig_num_templates

    # verify new template added properly
    new_template1 = attr_mgr.get_template_by_handle(new_template_handle)

    # verify template 0 and template 1 are copies of the same template
    assert new_template0.handle == new_template1.handle
    assert new_template0.template_id == new_template1.template_id
    assert new_template0.get_string("test_key") == new_template1.get_string("test_key")

    # remove newly added default template
    new_template2 = attr_mgr.remove_template_by_handle(new_template_handle)

    # verify added template was one removed
    assert new_template0.handle == new_template2.handle
    assert new_template0.template_id == new_template2.template_id
    assert new_template0.get_string("test_key") == new_template2.get_string("test_key")

    # test addition of user-configurations and verify values

    # create new template, do not register it
    new_template_usr = attr_mgr.create_new_template(new_template_handle, False)
    # give new template valid render asset handle, otherwise registration might fail
    if valid_render_handle is not None:
        new_template_usr.render_asset_handle = valid_render_handle

    usr_template_handle = "new_usr_cfg_handle"
    new_template_usr.handle = usr_template_handle

    # get user configs and set key
    new_template_usr.set_user_config_val("my_custom_key0", "my_custom_string")
    assert (
        new_template_usr.get_user_config_string("my_custom_key0") == "my_custom_string"
    )

    new_template_usr.set_user_config_val("my_custom_key1", True)
    assert new_template_usr.get_user_config_bool("my_custom_key1") == True
    new_template_usr.set_user_config_val("my_custom_key2", 10)
    assert new_template_usr.get_user_config_int("my_custom_key2") == 10
    new_template_usr.set_user_config_val("my_custom_key3", 5.8)
    assert new_template_usr.get_user_config_double("my_custom_key3") == 5.8
    new_template_usr.set_user_config_val("my_custom_key4", mn.Vector3(1.0, -2.8, 3.0))
    assert new_template_usr.get_user_config_vec3("my_custom_key4") == mn.Vector3(
        1.0, -2.8, 3.0
    )

    quat_val = mn.Quaternion.rotation(mn.Deg(-115), mn.Vector3.y_axis())
    new_template_usr.set_user_config_val("my_custom_key5", quat_val)

    assert new_template_usr.num_user_configs == 6

    # add new template - should use template-specified name as handle
    usr_tmplt_ID = attr_mgr.register_template(new_template_usr, "")
    assert usr_tmplt_ID != -1
    #

    reg_template_usr = attr_mgr.get_template_by_handle(usr_template_handle)
    assert reg_template_usr != None
    assert reg_template_usr.num_user_configs == new_template_usr.num_user_configs

    assert (
        reg_template_usr.get_user_config_string("my_custom_key0") == "my_custom_string"
    )
    assert reg_template_usr.get_user_config_bool("my_custom_key1") == True
    assert reg_template_usr.get_user_config_int("my_custom_key2") == 10
    assert reg_template_usr.get_user_config_double("my_custom_key3") == 5.8
    assert reg_template_usr.get_user_config_vec3("my_custom_key4") == mn.Vector3(
        1.0, -2.8, 3.0
    )
    assert reg_template_usr.get_user_config_quat("my_custom_key5") == quat_val

    rmv_template_usr = attr_mgr.remove_template_by_handle(usr_template_handle)
    assert rmv_template_usr != None

    assert new_template_usr.handle == rmv_template_usr.handle
    assert new_template_usr.template_id == rmv_template_usr.template_id
    # get new size of library after remove and verify same as original
    curr_num_templates = attr_mgr.get_num_templates()
    assert curr_num_templates == orig_num_templates


def test_physics_attributes_managers():
    cfg_settings = examples.settings.default_sim_settings.copy()
    cfg_settings["scene"] = "data/scene_datasets/habitat-test-scenes/van-gogh-room.glb"
    cfg_settings["enable_physics"] = True
    hab_cfg = examples.settings.make_cfg(cfg_settings)
    with habitat_sim.Simulator(hab_cfg) as sim:
        # get attribute managers
        phys_attr_mgr = sim.get_physics_template_manager()

        # perform general tests for this attributes manager
        template0, _ = perform_general_tests(
            phys_attr_mgr, cfg_settings["physics_config_file"]
        )

        # verify that physics template matches expected values in file
        assert template0.timestep == 0.008
        assert template0.simulator == "bullet"

        # verify creating new template
        perform_add_blank_template_test(phys_attr_mgr)


def test_stage_attributes_managers():
    cfg_settings = examples.settings.default_sim_settings.copy()
    cfg_settings["scene"] = "data/scene_datasets/habitat-test-scenes/van-gogh-room.glb"
    cfg_settings["enable_physics"] = True
    hab_cfg = examples.settings.make_cfg(cfg_settings)
    with habitat_sim.Simulator(hab_cfg) as sim:
        stage_name = cfg_settings["scene"]

        # get attribute managers
        stage_mgr = sim.get_stage_template_manager()

        # perform general tests for this attributes manager
        template0, _ = perform_general_tests(stage_mgr, stage_name)

        # verify gravity in template is as expected
        assert template0.gravity == mn.Vector3(0.0, -9.8, 0.0)

        # verify creating new template
        perform_add_blank_template_test(stage_mgr, template0.render_asset_handle)


def test_object_attributes_managers():
    cfg_settings = examples.settings.default_sim_settings.copy()
    cfg_settings["scene"] = "data/scene_datasets/habitat-test-scenes/van-gogh-room.glb"
    cfg_settings["enable_physics"] = True
    hab_cfg = examples.settings.make_cfg(cfg_settings)
    with habitat_sim.Simulator(hab_cfg) as sim:
        # get object attribute managers
        obj_mgr = sim.get_object_template_manager()

        # get object template random handle
        rand_obj_handle = obj_mgr.get_random_template_handle()

        # perform general tests for object attribute manager
        template0, _ = perform_general_tests(obj_mgr, rand_obj_handle)

        # verify creating new template
        perform_add_blank_template_test(obj_mgr, template0.render_asset_handle)


def perform_asset_attrib_mgr_tests(
    attr_mgr, default_attribs, ctor_mod_field, legalVal, illegalVal
):
    # get size of template library
    orig_num_templates = attr_mgr.get_num_templates()
    # make sure this is same value as size of template handles list
    assert orig_num_templates > 0

    # get old handle - based on how default_attribs was constructed
    old_handle = default_attribs.handle

    # verify that default_attribs is valid
    assert default_attribs.is_valid_template

    # modify field values that impact primitive construction, if exist
    if "none" != ctor_mod_field:
        default_attribs.set(ctor_mod_field, illegalVal)
        # verify that this is now an illegal template
        assert not (default_attribs.is_valid_template)

    # modify to hold legal value
    default_attribs.set(ctor_mod_field, legalVal)

    # verify that default_attribs is valid
    assert default_attribs.is_valid_template

    # build new handle reflected in modified template.
    # This is only necessary because we are bypassing setters
    default_attribs.build_handle()

    # verify that new handle is different from old handle
    new_handle = default_attribs.handle
    assert new_handle != old_handle

    # register new template
    attr_mgr.register_template(default_attribs)

    # verify that both templates now exist in library
    assert attr_mgr.get_library_has_handle(new_handle)
    assert attr_mgr.get_library_has_handle(old_handle)

    # get new and old templates
    old_template = attr_mgr.get_template_by_handle(old_handle)
    new_template = attr_mgr.get_template_by_handle(new_handle)

    # verify they do not hold the same values in the important fields
    assert old_template.get_int(ctor_mod_field) != new_template.get_int(ctor_mod_field)
    # verify we have more templates than when we started
    assert orig_num_templates != attr_mgr.get_num_templates()

    # delete new template
    deleted_template = attr_mgr.remove_template_by_handle(new_handle)
    assert deleted_template != None

    # verify we are back where we started
    assert orig_num_templates == attr_mgr.get_num_templates()


def test_asset_attributes_managers():
    cfg_settings = examples.settings.default_sim_settings.copy()
    cfg_settings["scene"] = "data/scene_datasets/habitat-test-scenes/van-gogh-room.glb"
    cfg_settings["enable_physics"] = True
    hab_cfg = examples.settings.make_cfg(cfg_settings)
    with habitat_sim.Simulator(hab_cfg) as sim:
        # legal and illegal vals for primitives based on wireframe or solid
        legal_mod_val_wf = 64
        illegal_mod_val_wf = 25
        legal_mod_val_solid = 5
        illegal_mod_val_solid = 0

        # get object attribute managers
        attr_mgr = sim.get_asset_template_manager()

        # capsule
        print("Test Capsule attributes construction, modification, saving and removal.")
        dflt_solid_attribs = attr_mgr.get_default_capsule_template(False)
        perform_asset_attrib_mgr_tests(
            attr_mgr,
            dflt_solid_attribs,
            "segments",
            legal_mod_val_solid,
            illegal_mod_val_solid,
        )
        dflt_wf_attribs = attr_mgr.get_default_capsule_template(True)
        perform_asset_attrib_mgr_tests(
            attr_mgr, dflt_wf_attribs, "segments", legal_mod_val_wf, illegal_mod_val_wf
        )

        # cone
        print("Test Cone attributes construction, modification, saving and removal.")
        dflt_solid_attribs = attr_mgr.get_default_cone_template(False)
        perform_asset_attrib_mgr_tests(
            attr_mgr,
            dflt_solid_attribs,
            "segments",
            legal_mod_val_solid,
            illegal_mod_val_solid,
        )
        dflt_wf_attribs = attr_mgr.get_default_cone_template(True)
        perform_asset_attrib_mgr_tests(
            attr_mgr, dflt_wf_attribs, "segments", legal_mod_val_wf, illegal_mod_val_wf
        )

        # cylinder
        print(
            "Test Cylinder attributes construction, modification, saving and removal."
        )
        dflt_solid_attribs = attr_mgr.get_default_cylinder_template(False)
        perform_asset_attrib_mgr_tests(
            attr_mgr, dflt_solid_attribs, "segments", 5, illegal_mod_val_solid
        )
        dflt_wf_attribs = attr_mgr.get_default_cylinder_template(True)
        perform_asset_attrib_mgr_tests(
            attr_mgr, dflt_wf_attribs, "segments", legal_mod_val_wf, illegal_mod_val_wf
        )

        # UVSphere
        print(
            "Test UVSphere attributes construction, modification, saving and removal."
        )
        dflt_solid_attribs = attr_mgr.get_default_UVsphere_template(False)
        perform_asset_attrib_mgr_tests(
            attr_mgr, dflt_solid_attribs, "segments", 5, illegal_mod_val_solid
        )
        dflt_wf_attribs = attr_mgr.get_default_UVsphere_template(True)
        perform_asset_attrib_mgr_tests(
            attr_mgr, dflt_wf_attribs, "segments", legal_mod_val_wf, illegal_mod_val_wf
        )
