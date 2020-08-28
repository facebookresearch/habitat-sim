from os import path as osp

import magnum as mn
import pytest

import examples.settings
from habitat_sim.gfx import PhongMaterialInfo


@pytest.mark.skipif(
    not osp.exists("data/scene_datasets/habitat-test-scenes/skokloster-castle.glb")
    or not osp.exists("data/objects/"),
    reason="Requires the habitat-test-scenes and habitat test objects",
)

# change the material for a render asset (mesh)
def test_render_asset_material_api(sim):
    cfg_settings = examples.settings.default_sim_settings.copy()

    cfg_settings[
        "scene"
    ] = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    cfg_settings["enable_physics"] = True

    hab_cfg = examples.settings.make_cfg(cfg_settings)
    sim.reconfigure(hab_cfg)
    obj_mgr = sim.get_object_template_manager()

    assert obj_mgr.get_num_templates() > 0

    # add an object
    object_id = sim.add_object(0)

    render_asset_handle = sim.get_object_initialization_template(
        object_id
    ).render_asset_handle

    num_materials = sim.get_num_render_asset_materials(render_asset_handle)
    assert num_materials
    material_index = num_materials - 1

    # create a new material
    new_material = PhongMaterialInfo()
    new_material.shininess = 1.234
    new_material.diffuse_color = mn.Color4(0.1, 0.2, 0.3, 0.4)

    existing_material = sim.get_render_asset_material(
        render_asset_handle, material_index
    )
    assert new_material != existing_material

    # change one of the render asset's materials
    sim.set_render_asset_material(render_asset_handle, material_index, new_material)

    updated_material = sim.get_render_asset_material(
        render_asset_handle, material_index
    )
    assert updated_material == new_material
