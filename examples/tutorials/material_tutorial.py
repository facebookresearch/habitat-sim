# [setup]
import math
import os

import magnum as mn
import numpy as np
from matplotlib import pyplot as plt

import habitat_sim
from habitat_sim.gfx import LightInfo, LightPositionModel
from habitat_sim.utils.common import quat_from_angle_axis, quat_to_magnum

dir_path = os.path.dirname(os.path.realpath(__file__))
data_path = os.path.join(dir_path, "../../data")
output_path = os.path.join(dir_path, "material_tutorial_output/")

save_index = 0


def show_img(data, save):
    plt.figure(figsize=(12, 12))
    plt.imshow(data, interpolation="nearest")
    plt.axis("off")
    plt.show(block=False)
    if save:
        global save_index
        plt.savefig(
            output_path + str(save_index) + ".jpg",
            bbox_inches="tight",
            pad_inches=0,
            quality=50,
        )
        save_index += 1
    plt.pause(1)


def get_obs(sim, show, save):
    obs = sim.get_sensor_observations()["rgba_camera"]
    if show:
        show_img(obs, save)
    return obs


def remove_all_objects(sim):
    for id in sim.get_existing_object_ids():
        sim.remove_object(id)


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


# [/setup]

# This is wrapped such that it can be added to a unit test
def main(show_imgs=True, save_imgs=False):
    if save_imgs:
        if not os.path.exists(output_path):
            os.mkdir(output_path)

    cfg = make_configuration()
    sim = habitat_sim.Simulator(cfg)
    agent_transform = place_agent(sim)

    # [example 1]

    # load some object templates from configuration files
    sphere_template_id = sim.load_object_configs(
        str(os.path.join(data_path, "test_assets/objects/sphere"))
    )[0]
    engine_template_id = sim.load_object_configs(
        str(os.path.join(data_path, "test_assets/objects/2CylinderEngine"))
    )[0]

    id_1 = sim.add_object(sphere_template_id)
    sim.set_translation([3.7, 0.23, 0.03], id_1)

    get_obs(sim, show_imgs, save_imgs)

    # [/example 1]

    # [example 2]

    # could also use sim.get_object_template(sphere_template_id) instead of
    # get_object_initialization_template
    render_asset_handle = sim.get_object_initialization_template(
        id_1
    ).get_render_asset_handle()
    num_materials = sim.get_num_render_asset_materials(render_asset_handle)
    assert num_materials == 1
    material_index = 0

    material = sim.get_render_asset_material(render_asset_handle, material_index)

    # make the specular highlights less shiny (more matte)
    material.shininess *= 0.1

    # reduce the specular highlights (make them darker)
    material.specular_color = mn.Color4(
        material.specular_color.r * 0.3,
        material.specular_color.g * 0.3,
        material.specular_color.b * 0.3,
    )

    # set diffuse color to blue
    material.diffuse_color = mn.Color4(0, 0, 1, 1)

    # change material for all existing and future uses of this render asset, including
    # object id_1
    sim.set_render_asset_material(render_asset_handle, material_index, material)

    get_obs(sim, show_imgs, save_imgs)

    # [/example 2]

    # [example 3]

    # Create a second sphere. It will also use the modified material.
    id_2 = sim.add_object(sphere_template_id)
    sim.set_translation([2.7, 0.23, 0.03], id_2)

    get_obs(sim, show_imgs, save_imgs)

    # [/example 3]

    # [example 4]

    # set diffuse color to green
    material.diffuse_color = mn.Color4(0.0, 0.5, 0.0, 1)

    # override the material for sphere id_2 only. Sphere id_1 will be unaffected.
    sim.override_object_render_asset_material(id_2, material_index, material)

    get_obs(sim, show_imgs, save_imgs)

    # [/example 4]

    # [example 5]

    # Add a model of an engine. We don't modify any materials yet.
    id_3 = sim.add_object(engine_template_id)
    sim.set_rotation(mn.Quaternion.rotation(mn.Deg(-50), mn.Vector3.y_axis()), id_3)
    sim.set_translation([2.2, 0.47, 1.15], id_3)

    get_obs(sim, show_imgs, save_imgs)

    # [/example 5]

    # [example 6]

    render_asset_handle = sim.get_object_initialization_template(
        id_3
    ).get_render_asset_handle()
    num_materials = sim.get_num_render_asset_materials(render_asset_handle)

    # This complex model has many materials, including several that contain the
    # substring "Material_23". We iterate over all materials, check the import_name,
    # and update all matches for Material_23.
    found_count = 0
    for i in range(num_materials):
        material = sim.get_render_asset_material(render_asset_handle, i)
        if material.import_name.find("Material_23") != -1:
            found_count += 1
            # black with a matte orange/gold specular highlight
            material.diffuse_color = mn.Color4(0.0, 0.0, 0.0, 1)
            material.shininess = 20
            material.specular_color = mn.Color4(1, 0.7, 0, 1)
            sim.set_render_asset_material(render_asset_handle, i, material)
    assert found_count

    get_obs(sim, show_imgs, save_imgs)

    # [/example 6]

    remove_all_objects(sim)


if __name__ == "__main__":
    main(show_imgs=True, save_imgs=True)
