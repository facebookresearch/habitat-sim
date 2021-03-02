#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import magnum as mn
import numpy as np
from PIL import Image

import habitat_sim
import habitat_sim.agent
from habitat_sim.agent.agent import AgentConfiguration
from habitat_sim.utils import gfx_replay_utils


def save_rgb_image(rgb_obs, filepath):

    # plt.figure(figsize=(12, 12))
    # plt.imshow(data, interpolation="nearest")
    # plt.axis("off")
    # # plt.show(block=False)
    # plt.savefig(filepath, bbox_inches="tight", pad_inches=0, format="png")

    colors = []
    for row in rgb_obs:
        for rgba in row:
            colors.extend([rgba[0], rgba[1], rgba[2]])

    resolution_x = len(rgb_obs[0])
    resolution_y = len(rgb_obs)

    colors = bytes(colors)
    img = Image.frombytes("RGB", (resolution_x, resolution_y), colors)
    img.save(filepath)


def save_depth_image(depth_obs, filepath):

    depth_img = Image.fromarray((depth_obs / 10 * 255).astype(np.uint8), mode="L")
    depth_img.save(filepath)


def write_replay_file(glb_filepath, camera_matrix_per_observation, output_filepath):

    assert len(camera_matrix_per_observation) > 0

    glb_json_template = (
        '{"keyframes": [{"loads": [{"type": 3,"filepath": "'
        + "glb_filepath"
        + '","frame": {"up": [0.0,0.0,1.0],"front": [0.0,1.0,0.0],"origin": [0.0,0.0,0.0]}, "virtualUnitToMeters": 1.0,"requiresLighting": false,"splitInstanceMesh": true}],"creations": [{"instanceKey": 0,"creation": {"filepath": "'
        + "glb_filepath"
        + '","isStatic": true,"isRGBD": true,"isSemantic": true,"lightSetupKey": "no_lights"}}]}'
    )

    glb_json = glb_json_template.replace("glb_filepath", glb_filepath)
    replay_json = glb_json

    for mat_as_list in camera_matrix_per_observation:

        camera_json_template = (
            ',{"userTransforms": [{"name": "camera", "transform": {"translation": ['
            + "translation_x,"
            + "translation_y,"
            + "translation_z"
            + '], "rotation": ['
            + "rotation_x,"
            + "rotation_y,"
            + "rotation_z,"
            + "rotation_w"
            + "]}}]}"
        )

        # assumptions for a 4x4 matrix representing a rotation (3x3 matrix ) and translation (Vector3)
        assert len(mat_as_list) == 16
        assert mat_as_list[3] == 0.0
        assert mat_as_list[7] == 0.0
        assert mat_as_list[11] == 0.0
        assert mat_as_list[15] == 1.0

        # convert 3x3 rotation matrix to quaternion
        rotation = mn.Quaternion.from_matrix(
            mn.Matrix3(
                mn.Vector3(mat_as_list[0], mat_as_list[1], mat_as_list[2]),
                mn.Vector3(mat_as_list[4], mat_as_list[5], mat_as_list[6]),
                mn.Vector3(mat_as_list[8], mat_as_list[9], mat_as_list[10]),
            )
        )

        translation = mn.Vector3(mat_as_list[12], mat_as_list[13], mat_as_list[14])

        camera_json = camera_json_template
        camera_json = camera_json.replace("translation_x", str(translation[0]))
        camera_json = camera_json.replace("translation_y", str(translation[1]))
        camera_json = camera_json.replace("translation_z", str(translation[2]))
        camera_json = camera_json.replace("rotation_x", str(rotation.vector[0]))
        camera_json = camera_json.replace("rotation_y", str(rotation.vector[1]))
        camera_json = camera_json.replace("rotation_z", str(rotation.vector[2]))
        camera_json = camera_json.replace("rotation_w", str(rotation.scalar))

        replay_json += camera_json

    replay_json += "]}"

    with open(output_filepath, "w") as text_file:
        text_file.write(replay_json)


def create_sim(
    resolution_x=1024,
    resolution_y=768,
    hfov=90,
    do_depth=False,
    texture_downsample_factor=0,
):

    agent_cfg = AgentConfiguration()

    rgbd_sensor_cfg = habitat_sim.SensorSpec()
    rgbd_sensor_cfg.uuid = "rgba_camera"
    rgbd_sensor_cfg.resolution = [resolution_y, resolution_x]
    assert resolution_y >= 1 and resolution_x >= 1
    assert hfov > 0 and hfov < 180.0
    params = habitat_sim.MapStringString()
    params["hfov"] = str(hfov)
    params["near"] = "0.01"  # matches default in Sensor.h
    params["far"] = "1000"  # matches default in Sensor.h
    rgbd_sensor_cfg.parameters = params
    agent_cfg.sensor_specifications = [rgbd_sensor_cfg]

    if do_depth:
        depth_sensor_cfg = habitat_sim.SensorSpec()
        depth_sensor_cfg.uuid = "depth_sensor"
        depth_sensor_cfg.sensor_type = habitat_sim.SensorType.DEPTH
        depth_sensor_cfg.resolution = rgbd_sensor_cfg.resolution
        depth_sensor_cfg.parameters = params
        agent_cfg.sensor_specifications.append(depth_sensor_cfg)

    playback_cfg = habitat_sim.Configuration(
        gfx_replay_utils.make_backend_configuration_for_playback(
            need_separate_semantic_scene_graph=False
        ),
        [agent_cfg],
    )

    playback_cfg.sim_cfg.texture_downsample_factor = texture_downsample_factor

    sim = habitat_sim.Simulator(playback_cfg)
    agent_state = habitat_sim.AgentState()
    sim.initialize_agent(0, agent_state)
    return sim


def render_observations_from_replay(
    sim, replay_filepath, output_base_filepath, do_depth
):

    agent_node = sim.get_agent(0).body.object
    sensor_node = sim._sensors["rgba_camera"]._sensor_object.object
    if do_depth:
        depth_sensor_node = sim._sensors["depth_sensor"]._sensor_object.object

    # We place a dummy agent at the origin and then transform the sensor using the "camera" user transform stored in the replay.
    agent_node.translation = [0.0, 0.0, 0.0]
    agent_node.rotation = mn.Quaternion()

    player = sim.gfx_replay_manager.read_keyframes_from_file(replay_filepath)
    assert player

    num_renders = 0
    for index in range(player.get_num_keyframes()):
        player.set_keyframe_index(index)
        user_transform_pair = player.get_user_transform("camera")
        if user_transform_pair:
            (sensor_node.translation, sensor_node.rotation) = user_transform_pair
            if do_depth:
                depth_sensor_node.translation = sensor_node.translation
                depth_sensor_node.rotation = sensor_node.rotation
            observation = sim.get_sensor_observations()

            save_rgb_image(
                observation["rgba_camera"],
                output_base_filepath + "." + str(num_renders) + ".png",
            )

            if do_depth:
                save_depth_image(
                    observation["depth_sensor"],
                    output_base_filepath + "." + str(num_renders) + ".depth.png",
                )

            num_renders += 1

    if num_renders == 0:
        print("Error: missing 'camera' user transform in replay")
        quit()


def demo():
    # The goal of these utils is to help a user produce renders (images) given a user-specified model, camera intrinsics and camera transforms.
    #
    # Steps shown below:
    # 1. Write a Habitat replay file that stores (a) a reference to our desired model to be rendered, and (b) a camera transform (position and rotation) for each render.
    # 2. Initialize the simulator for offline rendering, specifying camera intrinsics.
    # 3. Load the replay and produce renders.
    #
    # The use of a Habitat replay file is slightly overkill, but it'll help us extend to more complex scenes (multiple models) and other renderers (Unity, Blender) in the future.

    # scene_filepath = "/data/projects/matterport/v1/tasks/mp3d_habitat/mp3d/2t7WUuJeko7/2t7WUuJeko7.glb"
    mp3d_scene_name = "Z6MFQCViBuw"
    scene_filepath = (
        "/data/projects/matterport/v1/tasks/mp3d_habitat/mp3d/"
        + mp3d_scene_name
        + "/"
        + mp3d_scene_name
        + ".glb"
    )
    resolution_x = 600
    resolution_y = 600
    hfov = 90
    rotation_quat = mn.Quaternion(
        mn.Vector3(-0.965926, 1.58481e-17, -0.258819), -5.91459e-17
    )
    translation = [8.38665, 1.442450000000001, -13.7832]
    # 0 = default
    # 1 = 2x downsample (e.g. 256x256 gets downsampled to 128x128)
    # 2 = 4x downsample
    texture_downsample_factor = 4

    output_image_name = mp3d_scene_name + "_downsample" + str(texture_downsample_factor)

    do_depth = False  # also write grayscale depth image?

    # list of camera transform matrices (camera position/rotation), one per render
    camera_matrices = []

    rotation_mat33 = rotation_quat.to_matrix()
    mat44_as_list = [
        rotation_mat33[0][0],
        rotation_mat33[0][1],
        rotation_mat33[0][2],
        0,
        rotation_mat33[1][0],
        rotation_mat33[1][1],
        rotation_mat33[1][2],
        0,
        rotation_mat33[2][0],
        rotation_mat33[2][1],
        rotation_mat33[2][2],
        0,
        translation[0],
        translation[1],
        translation[2],
        1.0,
    ]
    camera_matrices.append(mat44_as_list)

    replay_filepath = "temp.replay.json"

    # write a Habitat replay file. This format stores a reference to our model plus
    # camera transforms (stored as Vector3 translation plus Quaternion rotation).
    write_replay_file(
        # "/data/projects/habitat-sim3/data/scene_datasets/habitat-test-scenes/apartment_1.glb",
        scene_filepath,
        # "/data/projects/p-viz-plan/orp/start_data/frl_apartment_stage_pvizplan_full.glb",
        # "/data/projects/habitat-sim3/data/test_assets/objects/sphere.glb",
        camera_matrices,
        replay_filepath,
    )

    # camera intrinsics: resolution and horizontal FOV (aspect ratio and vertical FOV
    # are derived from resolution)
    sim = create_sim(
        resolution_x=resolution_x,
        resolution_y=resolution_y,
        hfov=hfov,
        do_depth=do_depth,
        texture_downsample_factor=texture_downsample_factor,
    )

    render_observations_from_replay(sim, replay_filepath, output_image_name, do_depth)


if __name__ == "__main__":
    demo()
