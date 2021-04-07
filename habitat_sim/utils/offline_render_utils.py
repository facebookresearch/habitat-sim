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
from habitat_sim.gfx import LightInfo, LightPositionModel
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


def save_semantic_image(semantic_obs, filepath):

    semantic_img = Image.fromarray(semantic_obs.astype(np.uint8), mode="L")
    semantic_img.save(filepath)


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


def create_sim_with_stage(
    stage_filepath,
    resolution_x=1024,
    resolution_y=768,
    hfov=90,
    do_depth=False,
    do_semantic=True,
    texture_downsample_factor=0,
):

    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id = stage_filepath

    return create_sim(
        sim_cfg,
        resolution_x=resolution_x,
        resolution_y=resolution_y,
        hfov=hfov,
        do_depth=do_depth,
        do_semantic=do_semantic,
        texture_downsample_factor=texture_downsample_factor,
    )


def create_sim_for_replay_playback(
    resolution_x=1024,
    resolution_y=768,
    hfov=90,
    do_depth=False,
    texture_downsample_factor=0,
):

    sim_cfg = gfx_replay_utils.make_backend_configuration_for_playback(
        need_separate_semantic_scene_graph=False
    )

    return create_sim(
        sim_cfg,
        resolution_x=resolution_x,
        resolution_y=resolution_y,
        hfov=hfov,
        do_depth=do_depth,
        do_semantic=False,
        texture_downsample_factor=texture_downsample_factor,
    )


def create_sim(
    sim_cfg,
    resolution_x=1024,
    resolution_y=768,
    hfov=90,
    do_depth=False,
    do_semantic=False,
    texture_downsample_factor=0,
):
    agent_cfg = AgentConfiguration()

    # rgbd_sensor_cfg = habitat_sim.SensorSpec()
    # rgbd_sensor_cfg.uuid = "rgba_camera_1stperson"
    # rgbd_sensor_cfg.resolution = [resolution_y, resolution_x]
    # rgbd_sensor_cfg.sensor_type = habitat_sim.SensorType.COLOR

    # sensor_cfg = habitat_sim.CameraSensorSpec()
    # sensor_cfg.resolution = [resolution_y, resolution_x]

    rgbd_sensor_cfg = habitat_sim.CameraSensorSpec()
    rgbd_sensor_cfg.uuid = "rgba_camera"
    rgbd_sensor_cfg.sensor_type = habitat_sim.SensorType.COLOR
    rgbd_sensor_cfg.resolution = [resolution_y, resolution_x]
    rgbd_sensor_cfg.sensor_subtype = habitat_sim.SensorSubType.PINHOLE

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

    if do_semantic:
        semantic_camera_spec = habitat_sim.CameraSensorSpec()
        semantic_camera_spec.uuid = "semantic_camera"
        semantic_camera_spec.sensor_type = habitat_sim.SensorType.SEMANTIC
        semantic_camera_spec.resolution = rgbd_sensor_cfg.resolution
        semantic_camera_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
        agent_cfg.sensor_specifications.append(semantic_camera_spec)

    playback_cfg = habitat_sim.Configuration(sim_cfg, [agent_cfg])

    if hasattr(playback_cfg.sim_cfg, "texture_downsample_factor"):
        playback_cfg.sim_cfg.texture_downsample_factor = texture_downsample_factor
    else:
        if texture_downsample_factor != 0:
            print(
                "error: Your Habitat-sim build doesn't support texture_downsample_factor. Did you build from source? Are you on branch github.com/eundersander/habitat-sim/tree/eundersander/offline_render_utils?"
            )

    sim = habitat_sim.Simulator(playback_cfg)
    agent_state = habitat_sim.AgentState()
    sim.initialize_agent(0, agent_state)
    return sim


def render_observations_from_camera_transform_pairs(
    sim, camera_transform_pairs, output_base_filepath, do_depth, do_semantic
):

    agent_node = sim.get_agent(0).body.object
    sensor_node = sim._sensors["rgba_camera"]._sensor_object.object
    if do_depth:
        depth_sensor_node = sim._sensors["depth_sensor"]._sensor_object.object
    if do_semantic:
        semantic_sensor_node = sim._sensors["semantic_camera"]._sensor_object.object

    # We place a dummy agent at the origin and then later transform the sensor
    # using the desired camera transform.
    agent_node.translation = [0.0, 0.0, 0.0]
    agent_node.rotation = mn.Quaternion()

    num_renders = 0
    for transform_pair in camera_transform_pairs:
        (sensor_node.translation, sensor_node.rotation) = transform_pair
        if do_depth:
            depth_sensor_node.translation = sensor_node.translation
            depth_sensor_node.rotation = sensor_node.rotation
        if do_semantic:
            semantic_sensor_node.translation = sensor_node.translation
            semantic_sensor_node.rotation = sensor_node.rotation
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

        if do_semantic:
            save_semantic_image(
                observation["semantic_camera"],
                output_base_filepath + "." + str(num_renders) + ".semantic.png",
            )


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


# This is sample code to load a stage and object into Habitat and render RGB
# and semantic images.
def demo_using_stage_and_object():

    # the stage (background). As an example, I've downloaded the habitat version
    # of the MP3D scene dataset to my local machine and named one scene here.
    # See https://github.com/facebookresearch/habitat-lab#data.
    stage_filepath = "/home/eundersander/projects/matterport/v1/tasks/mp3d_habitat/mp3d/17DRP5sb8fy/17DRP5sb8fy.glb"

    # the object. In addition to a 3D model (glb or obj), you need an
    # object_config.json file, which should look like this:
    # {
    #    "render_asset": "ycb/002_master_chef_can/google_16k/textured.obj",
    #    "requires_lighting": true
    #    "scale": [1, 1, 1]
    # }
    # For this example, I've used a YCB object "002_master_chef_can". YCB objects
    # can be downloaded here: https://www.ycbbenchmarks.com/object-models/.

    # the directory where all your object_config.json files are located
    objects_directory = "/home/eundersander/projects/ycb/objects"

    # the specific object to render for this image
    object_filepath = objects_directory + "/002_master_chef_can.object_config.json"

    # position and rotation for the object in the scene
    object_translation = [2.05658, 0.072447, -1.65367]
    object_rotation_quat = mn.Quaternion(mn.Vector3(0, 0, 0), 1)

    # object semantic id (integer). This value will appear at the object's pixels in
    # the semantic image output. See also examples/tutorials/semantic_id_tutorial.py.
    object_semantic_id = 255

    # position and rotation for the camera
    camera_translation = [2.66093, 1.57245, -3.36188]
    camera_rotation_quat = mn.Quaternion(
        mn.Vector3(-0.0153043, 0.977763, 0.194489), 0.0769398
    )

    # light setup. For each light, choose a position/direction and color. See
    # also examples/tutorials/lighting_tutorial.py.
    lighting_setup = [
        LightInfo(
            vector=[1.0, 1.0, 0.0, 0.0],
            color=[2.75, 2.75, 2.75],
            model=LightPositionModel.GLOBAL,
        ),
        LightInfo(
            vector=[-0.5, 0.0, -1.0, 0.0],
            color=[2.75, 2.75, 2.75],
            model=LightPositionModel.GLOBAL,
        ),
    ]

    # output image properties
    resolution_x = 600
    resolution_y = 600
    hfov = 90
    do_depth = False

    # Look for output.0.png and output.0.semantic.png. See also
    # render_observations_from_camera_transform_pairs
    output_base_filepath = "./output"

    # Semantic output. With some python post-processing, you can extract the
    # object's 2D bbox. See also object_semantic_id above.
    do_semantic = True

    do_visualize_bbox = True  # for debugging

    # initialize the sim, sensors, and stage
    sim = create_sim_with_stage(
        stage_filepath,
        resolution_x=resolution_x,
        resolution_y=resolution_y,
        hfov=hfov,
        do_depth=do_depth,
        do_semantic=do_semantic,
    )

    # apply desired lighting
    sim.set_light_setup(lighting_setup)

    # add the object to the scene at the desired pose
    sim.get_object_template_manager().load_configs(objects_directory)
    obj_id = sim.add_object_by_handle(object_filepath)
    assert obj_id != -1
    sim.set_translation(object_translation, obj_id)
    sim.set_rotation(object_rotation_quat, obj_id)
    sim.set_object_semantic_id(object_semantic_id, obj_id)

    # print the bbox size of the 3D model
    bb = sim.get_object_scene_node(obj_id).cumulative_bb
    print(
        "bbbox size (x,y,z): [{:.2f}, {:.2f}, {:.2f}]".format(
            bb.size_x(), bb.size_y(), bb.size_z()
        )
    )

    if do_visualize_bbox:
        sim.set_object_bb_draw(True, obj_id)

    # produce images from the desired camera pose
    camera_transform_pairs = [(camera_translation, camera_rotation_quat)]

    render_observations_from_camera_transform_pairs(
        sim, camera_transform_pairs, output_base_filepath, do_depth, do_semantic
    )


# Whereas demo_using_stage_and_object loads a stage and physics objects, this
# sample code uses gfx-replay to directly load and pose a 3D model.
def demo_using_replay():
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
        "/home/eundersander/projects/matterport/v1/tasks/mp3d_habitat/mp3d/"
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
        scene_filepath,
        camera_matrices,
        replay_filepath,
    )

    # camera intrinsics: resolution and horizontal FOV (aspect ratio and vertical FOV
    # are derived from resolution)
    sim = create_sim_for_replay_playback(
        resolution_x=resolution_x,
        resolution_y=resolution_y,
        hfov=hfov,
        do_depth=do_depth,
        texture_downsample_factor=texture_downsample_factor,
    )

    render_observations_from_replay(sim, replay_filepath, output_image_name, do_depth)


if __name__ == "__main__":
    demo_using_stage_and_object()
