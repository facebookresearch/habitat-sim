# ---
# jupyter:
#   accelerator: GPU
#   jupytext:
#     cell_metadata_filter: -all
#     formats: nb_python//py:percent,colabs//ipynb
#     notebook_metadata_filter: all
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.5.2
#   kernelspec:
#     display_name: Python 3
#     name: python3
# ---

# %%
# !curl -L https://raw.githubusercontent.com/facebookresearch/habitat-sim/master/examples/colab_utils/colab_install.sh | NIGHTLY=true bash -s

# %%
# %cd /content/habitat-sim
import math
import os
import random
import sys

import git
import magnum as mn
import numpy as np
from matplotlib import pyplot as plt

import habitat_sim
from habitat_sim.gfx import PhongMaterialInfo
from habitat_sim.utils.common import quat_from_angle_axis

if "google.colab" in sys.modules:
    os.environ["IMAGEIO_FFMPEG_EXE"] = "/usr/bin/ffmpeg"

repo = git.Repo(".", search_parent_directories=True)
dir_path = repo.working_tree_dir
data_path = os.path.join(dir_path, "data")


def place_agent(sim):
    # place our agent in the scene
    agent_state = habitat_sim.AgentState()
    agent_state.position = [5.0, 0.0, 1.0]
    agent_state.rotation = quat_from_angle_axis(
        math.radians(70), np.array([0, 1.0, 0])
    ) * quat_from_angle_axis(math.radians(-20), np.array([1.0, 0, 0]))
    agent = sim.initialize_agent(0, agent_state)
    return agent.scene_node.transformation_matrix()


def make_configuration():
    # simulator configuration
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene.id = "data/scene_datasets/habitat-test-scenes/van-gogh-room.glb"
    backend_cfg.enable_physics = True

    # agent configuration
    sensor_cfg = habitat_sim.SensorSpec()
    sensor_cfg.resolution = [1080, 960]
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = [sensor_cfg]

    return habitat_sim.Configuration(backend_cfg, [agent_cfg])


def show_obs(sim):
    data = sim.get_sensor_observations()["rgba_camera"]
    plt.figure(figsize=(12, 12))
    plt.imshow(data, interpolation="nearest")
    plt.axis("off")
    plt.show(block=False)
    plt.pause(1)


if __name__ == "__main__":

# %%
    # set up the scene and agent (camera position)

    cfg = make_configuration()
    sim = habitat_sim.Simulator(cfg)
    place_agent(sim)

# %%
    # place a sphere in the scene
    obj_templates_mgr = sim.get_object_template_manager()

    sphere_template_id = obj_templates_mgr.load_object_configs(
        str(os.path.join(data_path, "test_assets/objects/sphere"))
    )[0]

    sphere_ids = []
    sphere_ids.append(sim.add_object(sphere_template_id))
    sim.set_translation([3.7, 0.23, 0.0], sphere_ids[0])

    show_obs(sim)

# %%
    # change the material for the sphere render asset

    # could also use sim.get_object_template(sphere_template_id) instead of
    # get_object_initialization_template
    render_asset_handle = sim.get_object_initialization_template(
        sphere_ids[0]
    ).render_asset_handle
    num_materials = sim.get_num_render_asset_materials(render_asset_handle)
    assert num_materials == 1
    material_index = 0

    material = sim.get_render_asset_material(render_asset_handle, material_index)

    # make the specular highlights less shiny (more matte)
    material.shininess *= 0.1

    # reduce the specular highlights (make them darker)
    material.specular_color *= 0.3

    # set diffuse to dark green
    material.diffuse_color = mn.Color4(r=0.2, g=0.4, b=0.2)

    # change material for all existing and future uses of this render asset, including
    # the existing sphere.
    sim.set_render_asset_material(render_asset_handle, material_index, material)

    show_obs(sim)

# %%
    # create more spheres. They will also use the modified material.

    for x in [3.0, 2.3, 1.6, 0.9]:
        id = sim.add_object(sphere_template_id)
        sim.set_translation([x, 0.23, 0.0], id)
        sphere_ids.append(id)

    show_obs(sim)

# %%
    # override_object_render_asset_material API is still pending. I leave this
    # code for reference.
    if False:
        # randomize material properties of individual spheres
        random.seed(5)
        for id in sphere_ids:
            randomized_material = PhongMaterialInfo(material)
            randomized_material.specular_color *= random.uniform(0, 2)
            randomized_material.diffuse_color = mn.Color4(
                r=material.diffuse_color.r + random.uniform(-0.1, 0.1),
                g=material.diffuse_color.g + random.uniform(-0.1, 0.1),
                b=material.diffuse_color.b + random.uniform(-0.1, 0.1),
            )
            sim.override_object_render_asset_material(
                id, material_index, randomized_material
            )

        show_obs(sim)
# %%
    # place another test object, a torus stack. We don't modify any materials yet.
    engine_template_id = obj_templates_mgr.load_object_configs(
        str(os.path.join(data_path, "test_assets/objects/torus_stack"))
    )[0]
    engine_id = sim.add_object(engine_template_id)
    sim.set_rotation(
        mn.Quaternion.rotation(mn.Deg(-50), mn.Vector3.y_axis()), engine_id
    )
    sim.set_translation([2.2, 0.47, 1.15], engine_id)

    show_obs(sim)
# %%
    # create a new material for use with the torus stack: purple with a
    # green specular highlight.
    new_material = PhongMaterialInfo(
        ambient_color=mn.Color4(0.15, 0.0, 0.15, 1),
        diffuse_color=mn.Color4(0.15, 0.0, 0.15, 1),
        specular_color=mn.Color4(0, 1, 0, 1),
        shininess=20.0,
    )

    render_asset_handle = sim.get_object_initialization_template(
        engine_id
    ).render_asset_handle
    num_materials = sim.get_num_render_asset_materials(render_asset_handle)

    # This model has several materials. We iterate over them and change only
    # "torus3_material".
    found_count = 0
    for i in range(num_materials):
        material = sim.get_render_asset_material(render_asset_handle, i)
        if material.import_name.find("torus3_material") != -1:
            found_count += 1
            sim.set_render_asset_material(render_asset_handle, i, new_material)
    assert found_count

    show_obs(sim)
