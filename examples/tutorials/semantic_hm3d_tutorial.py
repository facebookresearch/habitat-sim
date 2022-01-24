import math
import os

import magnum as mn
import numpy as np
from matplotlib import pyplot as plt
from PIL import Image

import habitat_sim
from habitat_sim.utils.common import quat_from_angle_axis

dir_path = os.path.dirname(os.path.realpath(__file__))
output_path = os.path.join(dir_path, "semantic_hm3d_tutorial_output/")

save_index = 0


def save_img_pil(data):
    global save_index
    rgb_image = Image.fromarray(np.uint8(data))
    filepath = output_path + str(save_index) + ".png"
    rgb_image.save(filepath)
    print("saved {}".format(filepath))
    save_index += 1

def show_img(data, save):
    save_img_pil(data[0])
    save_img_pil(data[1])


def get_obs(sim, show, save):
    # render sensor ouputs and optionally show them
    # perf todo don't call get_sensor_observations() twice; this draws twice!
    observations = sim.get_sensor_observations()
    rgb_obs = observations["rgba_camera"]
    semantic_obs = observations["semantic_camera"]
    if show:
        show_img((rgb_obs, semantic_obs), save)


def place_agent(sim):
    # place our agent in the scene
    agent_state = habitat_sim.AgentState()
    agent_state.position = [5.0, 0.0, 1.0]
    agent_state.rotation = quat_from_angle_axis(
        math.radians(70), np.array([0, 1.0, 0])
    ) * quat_from_angle_axis(math.radians(-20), np.array([1.0, 0, 0]))
    agent = sim.initialize_agent(0, agent_state)
    return agent.scene_node.transformation_matrix()


def make_configuration(scene_file, hm3d_data_path):
    # simulator configuration
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene_id = scene_file
    backend_cfg.scene_dataset_config_file = hm3d_data_path + "/hm3d_annotated_basis.scene_dataset_config.json"
    backend_cfg.enable_physics = True

    # sensor configurations
    # Note: all sensors must have the same resolution
    # setup rgb and semantic sensors
    camera_resolution = [1080, 960]
    sensor_specs = []

    rgba_camera_spec = habitat_sim.CameraSensorSpec()
    rgba_camera_spec.uuid = "rgba_camera"
    rgba_camera_spec.sensor_type = habitat_sim.SensorType.COLOR
    rgba_camera_spec.resolution = camera_resolution
    rgba_camera_spec.position = [0.0, 1.5, 0.0]  # ::: fix y to be 0 later
    rgba_camera_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
    sensor_specs.append(rgba_camera_spec)

    semantic_camera_spec = habitat_sim.CameraSensorSpec()
    semantic_camera_spec.uuid = "semantic_camera"
    semantic_camera_spec.sensor_type = habitat_sim.SensorType.SEMANTIC
    semantic_camera_spec.resolution = camera_resolution
    semantic_camera_spec.position = [0.0, 1.5, 0.0]
    semantic_camera_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
    sensor_specs.append(semantic_camera_spec)

    # agent configuration
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs

    return habitat_sim.Configuration(backend_cfg, [agent_cfg])


# This is wrapped such that it can be added to a unit test
def main(hm3d_data_path=None, show_imgs=True, save_imgs=False):
    if save_imgs and not os.path.exists(output_path):
        os.mkdir(output_path)

    sim = None

    test_scenes = [
        hm3d_data_path + "/hm3d-minival-habitat/00800-TEEsavR23oF/TEEsavR23oF.basis.glb",
    ]

    for scene in test_scenes:
        # reconfigure/construct the simulator with a new scene asset
        cfg = make_configuration(scene_file=scene, hm3d_data_path=hm3d_data_path)
        if sim:
            sim.reconfigure(cfg)
        else:
            sim = habitat_sim.Simulator(cfg)
        place_agent(sim)  # noqa: F841

        agent_node = sim.get_agent(0).body.object

        # some hard-coded positions for scene TEEsavR23oF; see also place_agent
        positions = [
            [2.0, 0.0, -2.0], [0.0, 0.0, -3.0], [-4.0, 0.0, -1.0], [-7.0, 0.0, -1.0],
            [0.0, 3.0, -6.0], [-4.0, 3.0, -2.0]
        ]

        for agent_pos in positions:
            agent_node.translation = agent_pos
            get_obs(sim, show_imgs, save_imgs)
        
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--hm3d-data-path", required=True)
    parser.add_argument("--no-save-images", dest="save_images", action="store_false")
    parser.set_defaults(show_images=True, save_images=True)
    args = parser.parse_args()
    main(hm3d_data_path=args.hm3d_data_path, show_imgs=args.show_images, save_imgs=args.save_images)
