#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

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


def get_obs(sim, camera, show=True, save=False, output_path=""):
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

    Returns
    -------
    obs : list
        2d numpy array of ints or floats representing observation.

    """
    obs = sim.get_sensor_observations()[camera]
    if show:
        show_img(obs, save, output_path)
    return obs


def show_img(data, save=False, output_path=""):
    """
    Display an image of the passed data

    Parameters
    ----------
    data : list
        Array-like structure of image data.  
        See matplotlib.pyplot.imshow for more.
    save : boolean optional
        Whether to save image to path specified
    output_path : string, optional
        DESCRIPTION. The default is "".

    Returns
    -------
    None.

    """
    from matplotlib import pyplot as plt

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
    TYPE
        DESCRIPTION.

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
