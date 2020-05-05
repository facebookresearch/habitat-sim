import magnum as mn

from habitat_sim.gfx import PhongMaterialInfo


def test_material_api(sim):

    assert sim.get_physics_object_library_size()

    object_id = sim.add_object(0)

    render_asset_handle = sim.get_object_template(object_id).get_render_asset_handle()

    num_materials = sim.get_num_render_asset_materials(render_asset_handle)
    assert num_materials
    material_index = num_materials - 1

    new_material = PhongMaterialInfo()
    new_material.shininess = 1.234
    new_material.diffuse_color = mn.Color4(0.1, 0.2, 0.3, 0.4)

    existing_material = sim.get_render_asset_material(
        render_asset_handle, material_index
    )
    assert new_material != existing_material

    sim.set_render_asset_material(render_asset_handle, material_index, new_material)

    updated_material = sim.get_render_asset_material(
        render_asset_handle, material_index
    )
    assert updated_material == new_material

    new_material.shininess = 5.678

    # There's currently no easy way to test that this worked. We call it just to check
    # that it executes without error.
    sim.override_object_render_asset_material(object_id, material_index, new_material)

    # todo: test error-handling, e.g., out-of-range material index
