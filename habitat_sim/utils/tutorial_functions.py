#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os

import cv2
import numpy as np

import habitat_sim


"""
Functions commonly used in tutorials

"""


def make_configuration(test_scene, enable_physics, sensors_config_dict):
    """
    Build simulator, sensor and agent configurations

    Parameters
    ----------
    test_scene : string
        Name of file in habitat-test-scenes directory describing scene

    enable_physics : boolean
        Whether physics is enabled or not.

    sensors_config_dict : dictionary
         Dictionary keyed by sensor name that contains sensor specifications :
        "sensor_type" : habitat_sim.SensorType enum
            Type of sensor (SensorType.COLOR,SensorType.DEPTH,etc)
        "camera_resolution" : list of ints
            X, Y resolution of sensor image.
        "position" : list of 3 values
            Position of sensor relative to agent.
        "orientation" : list of 3 values
            Orientation of sensor relative to agent (no roll).

    Returns
    -------
    cfg : habitat_sim.Configuration
        Configuration required to instance necessary Habitat components.

    """
    # simulator configuration
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene.id = "data/scene_datasets/habitat-test-scenes/" + test_scene
    backend_cfg.enable_physics = enable_physics

    # sensor configurations
    sensor_specs = []
    for sensor_uuid, sensor_params in sensors_config_dict.items():
        sensor_spec = habitat_sim.SensorSpec()
        sensor_spec.uuid = sensor_uuid
        sensor_spec.sensor_type = sensor_params["sensor_type"]
        sensor_spec.resolution = sensor_params["resolution"]
        sensor_spec.position = sensor_params["position"]
        sensor_spec.orientation = sensor_params["orientation"]
        sensor_specs.append(sensor_spec)

    # agent configurations
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs

    return habitat_sim.Configuration(backend_cfg, [agent_cfg])


def make_video_cv2(
    observations,
    primary_pov,
    output_path,
    file_name,
    camera_res=[540, 720],
    open_vid=True,
    multi_obs=False,
    pov_list=[],
):
    """
    Build a video of passed observations array, with embedded images if desired

    Parameters
    ----------
    observations : list
        List of observations from which the video should be constructed

    primary_pov : string
        Primary camera name in observations to be used for image.

    output_path : string
        location where to save resultant .mp4 video.

    file_name : string
        File name to use to save resultant .mp4 video.

    camera_res : list, optional
        Height by Width of camera for image. Default is [540, 720]

    open_vid : boolean, optional
        Whether to open video upon creation.  Default is True

    multi_obs : boolean, optional
        Whether to embed images from POVs listed in pov_list.
        Defaults to False

    pov_list : List of tuples, optional
        If present, each tuple holds :
            idx 0 is camera name in observations
            idx 1 is boolean whether is depth canera or not.
        List of strings denoting specific camera types and POVs to consume
        from observations list.  Ignored if multi_obs is False

    Returns
    -------
    None.

    """
    videodims = (camera_res[1], camera_res[0])
    fourcc = cv2.VideoWriter_fourcc("m", "p", "4", "v")
    video_file_name = output_path + file_name + ".mp4"
    video = cv2.VideoWriter(video_file_name, fourcc, 60, videodims)
    thumb_size = (int(videodims[0] / 5), int(videodims[1] / 5))
    outline_frame = np.ones((thumb_size[1] + 2, thumb_size[0] + 2, 3), np.uint8) * 150

    if multi_obs and len(pov_list) > 1:
        for ob in observations:
            embed_image_data_list = [
                (ob[pov_list[i][0]], "depth" in pov_list[i][1].lower())
                for i in range(0, len(pov_list))
            ]

            res_image = build_multi_obs_image(
                ob[primary_pov], embed_image_data_list, thumb_size, outline_frame,
            )
            # write the desired image to video
            video.write(res_image)
    else:
        for ob in observations:
            res_image = ob[primary_pov][..., 0:3][..., ::-1]

            # write the desired image to video
            video.write(res_image)
    video.release()
    if open_vid:
        launch_vid(video_file_name)


def launch_vid(video_file_name):
    """
    Platform-independent (mac/linux) method to programmatically launch video at passed file location

    Parameters
    ----------
    video_file_name : string
        Name of video file to launch

    Returns
    -------
    None.

    """
    from sys import platform

    if "linux" in platform:
        # linux
        import subprocess

        subprocess.Popen("xdg-open " + video_file_name, shell=True).wait()
    elif "darwin" in platform:
        # OS X
        import subprocess

        subprocess.Popen("open " + video_file_name, shell=True).wait()


def build_multi_obs_image(
    base_camera_img, embed_image_data_list, thumb_size, outline_frame
):
    """
    Build an image containing multiple sub-images

    Parameters
    ----------
    base_camera_img : 2d numpy array
        Base image to insert sub images into.
    embed_image_data_list : list of tuples
        List of tuples where 
            idx 0 is numpy array of image data 
            idx 1 is boolean whether is depth data or not.
    thumb_size : tuple of ints
        X and Y dimensions of inset images
    outline_frame : numpy array
        Max X and Y dims of frame surrounding inset image

    Returns
    -------
    result_image : 2d numpy array
        Resultant compound image

    """

    result_image = base_camera_img[..., 0:3][..., ::-1]
    x_offset = 50
    y_offset = 50
    y_bound = 10
    for embed_image, embed_image_is_depth in embed_image_data_list:

        if embed_image_is_depth:
            bgr_embed_image = clip_depth_image(embed_image)
        else:
            bgr_embed_image = embed_image[..., 0:3][..., ::-1]

        resized_bgr_embed_image = cv2.resize(
            bgr_embed_image, thumb_size, interpolation=cv2.INTER_AREA
        )

        result_image[
            y_offset - 1 : y_offset + outline_frame.shape[0] - 1,
            x_offset - 1 : x_offset + outline_frame.shape[1] - 1,
        ] = outline_frame
        result_image[
            y_offset : y_offset + resized_bgr_embed_image.shape[0],
            x_offset : x_offset + resized_bgr_embed_image.shape[1],
        ] = resized_bgr_embed_image
        # move along y axis
        y_offset = y_offset + y_bound + thumb_size[1]

    return result_image


def clip_depth_image(depth_image, clip_max=10.0):
    """
    Manually normalize depth into [0, 1] so that images are always consistent

    Parameters
    ----------
    depth_image : 2d numpy array
        Depth image data

    Returns
    -------
    bgr_d_im : 2d numpy array
        Clipped depth image data

    """
    d_im = np.clip(depth_image, 0, clip_max)
    d_im /= clip_max
    bgr_d_im = cv2.cvtColor((d_im * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    return bgr_d_im


def get_obs(sim, camera, show=True, save=False, output_path="", file_name=""):
    """
    Get an agent observation for a specified camera.

    Parameters
    ----------
    sim : Simulator, 
        Reference to the habitat simulator object

    camera : string
        Camera name to get observation from.

    show : boolean, optional
        Whether to display an image of the observation. 
        The default is True

    save : boolean optional
        Whether to save image to path specified.  
        The default is False

    output_path : string, optional
        Where to save the image. The default is "".

    file_name : string, optional
        File name to use to save the .jpg image.  
        Default is "".

    Returns
    -------
    obs : list
        2d numpy array of ints or floats representing observation.

    """
    obs = sim.get_sensor_observations()[camera]
    if show:
        show_img(obs, save=save, output_path=output_path, file_name=file_name)
    return obs


def show_img(data, fig_size=(12, 12), save=False, output_path="", file_name=""):
    """
    Display an image of the passed data

    Parameters
    ----------
    data : list
        Array-like structure of image data.  
        See matplotlib.pyplot.imshow for more.

    fig_size : (float, float), optional, default: (12, 12)
        width, height in inches if resultant image.

    save : boolean optional
        Whether to save image to path and file name 
        specified. Default is False

    output_path : string, optional
        Where to save the .jpg image. The default is "".

    file_name : string, optional
        File name to use to save the .jpg image.  
        Default is "".

    Returns
    -------
    None.

    """
    from matplotlib import pyplot as plt

    plt.figure(figsize=fig_size)
    plt.imshow(data, interpolation="nearest")
    plt.axis("off")
    plt.show(block=False)
    if save:
        plt.savefig(
            output_path + file_name + ".jpg",
            bbox_inches="tight",
            pad_inches=0,
            quality=50,
        )
    plt.pause(1)


def place_agent(sim, pos=[0.0, 0.0, 0.0], rot=np.quaternion(-1, 0, 0, 0)):
    """    
    Places our agent in the scene using specified position and orientation parameters

    Parameters
    ----------
    sim : Simulator, 
        Reference to the habitat simulator object
    pos : List, optional
        Location in scene to place agent. The default is [0.0,0.0,0.0].
    rot : numpy quaternion, optional
        Orientation of agent in scene.  The default is np.quaternion(-1, 0, 0, 0)

    Returns
    -------
    agent's scene_node transformation matrix

    """
    agent_state = habitat_sim.AgentState()
    agent_state.position = pos
    agent_state.rotation = rot
    agent = sim.initialize_agent(0, agent_state)
    return agent.scene_node.transformation_matrix()


def remove_all_objects(sim):
    """
    Removes all objects from simulation world

    Parameters
    ----------
    sim : Simulator, 
        Reference to the habitat simulator object

    Returns
    -------
    None.

    """

    for id in sim.get_existing_object_ids():
        sim.remove_object(id)


def simulate(sim, dt=1.0, get_frames=True):
    """
    Forward simulate world physics by requested duration, 
    returning observations if requested


    Parameters
    ----------
    sim : Simulator, 
        Reference to the habitat simulator object

    dt : float, optional, default
        Duration of simulation in seconds, Defaults to 1.0

    get_frames : boolean, optional, default
        Whether to return observations from specified sensors 

    Returns
    -------
    observations : list
        list of 2d numpy array of ints or floats representing observation 
        per sensor per step.
    """

    # simulate dt seconds at 60Hz to the nearest fixed timestep
    print("Simulating " + str(dt) + " world seconds.")
    observations = []
    start_time = sim.get_world_time()
    while sim.get_world_time() < start_time + dt:
        sim.step_physics(1.0 / 60.0)
        if get_frames:
            observations.append(sim.get_sensor_observations())

    return observations
