# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# [setup]
import math
import os

import magnum as mn
import numpy as np
from matplotlib import pyplot as plt

import habitat_sim
from habitat_sim.gfx import LightInfo, LightPositionModel
from habitat_sim.utils.common import quat_from_angle_axis

dir_path = os.path.dirname(os.path.realpath(__file__))
data_path = os.path.join(dir_path, "../../data")
output_path = os.path.join(dir_path, "lighting_tutorial_output/")

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
    backend_cfg.scene_id = "data/scene_datasets/habitat-test-scenes/van-gogh-room.glb"

    # agent configuration
    sensor_cfg = habitat_sim.CameraSensorSpec()
    sensor_cfg.resolution = [1080, 960]
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = [sensor_cfg]

    return habitat_sim.Configuration(backend_cfg, [agent_cfg])


# [/setup]


# This is wrapped such that it can be added to a unit test
def main(show_imgs=True, save_imgs=False):
    if save_imgs and not os.path.exists(output_path):
        os.mkdir(output_path)

    # [default scene lighting]

    # create the simulator and render flat shaded scene
    cfg = make_configuration()
    sim = habitat_sim.Simulator(cfg)
    agent_transform = place_agent(sim)
    get_obs(sim, show_imgs, save_imgs)

    # [scene swap shader]

    # close the simulator and re-initialize with DEFAULT_LIGHTING_KEY:
    sim.close()
    cfg = make_configuration()
    cfg.sim_cfg.scene_light_setup = habitat_sim.gfx.DEFAULT_LIGHTING_KEY
    cfg.sim_cfg.override_scene_light_defaults = True
    sim = habitat_sim.Simulator(cfg)
    agent_transform = place_agent(sim)
    get_obs(sim, show_imgs, save_imgs)

    # create and register new light setup:
    my_scene_lighting_setup = [
        LightInfo(vector=[0.0, 2.0, 0.6, 1.0], model=LightPositionModel.Global)
    ]
    sim.set_light_setup(my_scene_lighting_setup, "my_scene_lighting")

    # reconfigure with custom key:
    new_cfg = make_configuration()
    new_cfg.sim_cfg.scene_light_setup = "my_scene_lighting"
    new_cfg.sim_cfg.override_scene_light_defaults = True
    sim.reconfigure(new_cfg)
    agent_transform = place_agent(sim)
    get_obs(sim, show_imgs, save_imgs)

    # [/scene]

    # reset to default scene shading
    sim.close()
    cfg = make_configuration()
    sim = habitat_sim.Simulator(cfg)
    agent_transform = place_agent(sim)  # noqa: F841

    # [example 2]

    # get the rigid object attributes manager, which manages
    # templates used to create objects
    obj_template_mgr = sim.get_object_template_manager()
    # get the rigid object manager, which provides direct
    # access to objects
    rigid_obj_mgr = sim.get_rigid_object_manager()

    # load some object templates from configuration files
    sphere_template_id = obj_template_mgr.load_configs(
        str(os.path.join(data_path, "test_assets/objects/sphere"))
    )[0]
    chair_template_id = obj_template_mgr.load_configs(
        str(os.path.join(data_path, "test_assets/objects/chair"))
    )[0]

    # create a sphere and place it at a desired location
    obj_1 = rigid_obj_mgr.add_object_by_template_id(sphere_template_id)
    obj_1.translation = [3.2, 0.23, 0.03]

    get_obs(sim, show_imgs, save_imgs)

    # [/example 2]

    # [example 3]

    # create a custom light setup
    my_default_lighting = [
        LightInfo(vector=[-2.0, -2.0, -1.0, 0.0], model=LightPositionModel.Camera)
    ]
    # overwrite the default DEFAULT_LIGHTING_KEY light setup
    sim.set_light_setup(my_default_lighting)

    get_obs(sim, show_imgs, save_imgs)

    # [/example 3]

    # [example 4]

    # create a chair and place it at a location with a specified orientation
    obj_2 = rigid_obj_mgr.add_object_by_template_id(chair_template_id)
    obj_2.rotation = mn.Quaternion.rotation(mn.Deg(-115), mn.Vector3.y_axis())
    obj_2.translation = [3.06, 0.47, 1.15]

    get_obs(sim, show_imgs, save_imgs)

    # [/example 4]

    # [example 5]
    light_setup_2 = [
        LightInfo(
            vector=[2.0, 1.5, 5.0, 1.0],
            color=[0.0, 100.0, 100.0],
            model=LightPositionModel.Global,
        )
    ]
    sim.set_light_setup(light_setup_2, "my_custom_lighting")

    # [/example 5]

    rigid_obj_mgr.remove_all_objects()

    # [example 6]

    # create and place 2 chairs with custom light setups
    chair_1 = rigid_obj_mgr.add_object_by_template_id(
        chair_template_id, light_setup_key="my_custom_lighting"
    )
    chair_1.rotation = mn.Quaternion.rotation(mn.Deg(-115), mn.Vector3.y_axis())
    chair_1.translation = [3.06, 0.47, 1.15]

    chair_2 = rigid_obj_mgr.add_object_by_template_id(
        chair_template_id, light_setup_key="my_custom_lighting"
    )
    chair_2.rotation = mn.Quaternion.rotation(mn.Deg(50), mn.Vector3.y_axis())
    chair_2.translation = [3.45927, 0.47, -0.624958]

    get_obs(sim, show_imgs, save_imgs)

    # [/example 6]

    # [example 7]
    existing_light_setup = sim.get_light_setup("my_custom_lighting")

    # create a new setup with an additional light
    new_light_setup = existing_light_setup + [
        LightInfo(
            vector=[0.0, 0.0, -1.0, 0.0],
            color=[1.6, 1.6, 1.4],
            model=LightPositionModel.Camera,
        )
    ]

    # register the old setup under a new name
    sim.set_light_setup(existing_light_setup, "my_old_custom_lighting")

    # [/example 7]

    # [example 8]

    # register the new setup overwriting the old one
    sim.set_light_setup(new_light_setup, "my_custom_lighting")

    get_obs(sim, show_imgs, save_imgs)

    # [/example 8]

    # [example 9]
    chair_1.set_light_setup(habitat_sim.gfx.DEFAULT_LIGHTING_KEY)

    get_obs(sim, show_imgs, save_imgs)

    # [/example 9]


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--no-show-images", dest="show_images", action="store_false")
    parser.add_argument("--no-save-images", dest="save_images", action="store_false")
    parser.set_defaults(show_images=True, save_images=True)
    args = parser.parse_args()
    main(show_imgs=args.show_images, save_imgs=args.save_images)
