# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# [setup]
import math
import os

import magnum as mn
import numpy as np
from PIL import Image

import habitat_sim
from habitat_sim.utils.common import quat_from_angle_axis

dir_path = os.path.dirname(os.path.realpath(__file__))
data_path = os.path.join(dir_path, "../../data")
output_path = os.path.join(dir_path, "semantic_object_tutorial_output/")

save_index = 0


def apply_colormap(image):
    # Normalize the uint32 image to the range [0, 255]
    normalized = (image - image.min()) / (image.max() - image.min()) * 255
    normalized = normalized.astype(np.uint8)

    # Create a color map
    colormap = np.zeros((256, 3), dtype=np.uint8)
    for i in range(256):
        colormap[i] = [i, 255 - i, (i // 2) % 256]

    # Apply the colormap
    colorized = colormap[normalized]
    return Image.fromarray(colorized)


def show_img(data, save):
    # Convert numpy arrays to PIL Images
    img1 = Image.fromarray(data[0])

    # Color-code the semantic map (uint32) to visualize the semantic IDs
    img2 = apply_colormap(data[1])

    # Create a new image with twice the width to place images side-by-side
    total_width = img1.width + img2.width
    max_height = max(img1.height, img2.height)
    new_img = Image.new("RGB", (total_width, max_height))

    # Paste the images side-by-side
    new_img.paste(img1, (0, 0))
    new_img.paste(img2, (img1.width, 0))

    # Display the combined image
    new_img.show()

    if save:
        global save_index
        new_img.save(output_path + str(save_index) + ".jpg", quality=50)
        save_index += 1


def get_obs(sim, show, save):
    # render sensor outputs and optionally show them
    rgb_obs = sim.get_sensor_observations()["rgba_camera"]
    semantic_obs = sim.get_sensor_observations()["semantic_camera"]
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


def make_configuration(scene_file):
    # simulator configuration
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene_id = scene_file

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


# [/setup]


# This is wrapped such that it can be added to a unit test
def main(show_imgs=True, save_imgs=False):
    if save_imgs and not os.path.exists(output_path):
        os.mkdir(output_path)

    # [semantic id]

    # create the simulator and render flat shaded scene
    cfg = make_configuration(scene_file="NONE")
    sim = habitat_sim.Simulator(cfg)

    test_scenes = [
        "data/scene_datasets/habitat-test-scenes/apartment_1.glb",
        "data/scene_datasets/habitat-test-scenes/van-gogh-room.glb",
    ]

    for scene in test_scenes:
        # reconfigure the simulator with a new scene asset
        cfg = make_configuration(scene_file=scene)
        sim.reconfigure(cfg)
        agent_transform = place_agent(sim)  # noqa: F841
        get_obs(sim, show_imgs, save_imgs)

        # get the physics object attributes manager
        obj_templates_mgr = sim.get_object_template_manager()
        # get the rigid object manager, which provides direct
        # access to objects
        rigid_obj_mgr = sim.get_rigid_object_manager()

        # load some chair object template from configuration file
        chair_template_id = obj_templates_mgr.load_configs(
            str(os.path.join(data_path, "test_assets/objects/chair"))
        )[0]

        # add 2 chairs with default semanticId == 0 and arrange them
        chairs = []
        chairs.append(rigid_obj_mgr.add_object_by_template_id(chair_template_id))
        chairs.append(rigid_obj_mgr.add_object_by_template_id(chair_template_id))

        chairs[0].rotation = mn.Quaternion.rotation(mn.Deg(-115), mn.Vector3.y_axis())

        chairs[0].translation = [2.0, 0.47, 0.9]
        chairs[1].translation = [2.9, 0.47, 0.0]

        get_obs(sim, show_imgs, save_imgs)

        # set the semanticId for both chairs
        chairs[0].semantic_id = 2
        chairs[1].semantic_id = 2

        get_obs(sim, show_imgs, save_imgs)

        # set the semanticId for one chair
        chairs[1].semantic_id = 1
        get_obs(sim, show_imgs, save_imgs)

        # add a box with default semanticId configured in the template
        box_template = habitat_sim.attributes.ObjectAttributes()
        box_template.render_asset_handle = str(
            os.path.join(data_path, "test_assets/objects/transform_box.glb")
        )

        box_template.scale = np.array([0.2, 0.2, 0.2])
        # set the default semantic id for this object template
        box_template.semantic_id = 10

        box_template_id = obj_templates_mgr.register_template(box_template, "box")

        box_obj = rigid_obj_mgr.add_object_by_template_id(box_template_id)
        box_obj.translation = [3.5, 0.47, 0.9]
        box_obj.rotation = mn.Quaternion.rotation(mn.Deg(-30), mn.Vector3.y_axis())

        get_obs(sim, show_imgs, save_imgs)

        # set semantic id for specific SceneNode components of the box object
        box_visual_nodes = box_obj.visual_scene_nodes
        box_visual_nodes[6].semantic_id = 3
        box_visual_nodes[7].semantic_id = 4
        get_obs(sim, show_imgs, save_imgs)

    # [/semantic id]


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--no-show-images", dest="show_images", action="store_false")
    parser.add_argument("--no-save-images", dest="save_images", action="store_false")
    parser.set_defaults(show_images=True, save_images=True)
    args = parser.parse_args()
    main(show_imgs=args.show_images, save_imgs=args.save_images)
