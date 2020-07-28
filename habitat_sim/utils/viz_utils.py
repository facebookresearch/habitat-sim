import base64
import io
import os
import subprocess
import sys

import cv2
import numpy as np

import habitat_sim

if "google.colab" in sys.modules:
    os.environ["IMAGEIO_FFMPEG_EXE"] = "/usr/bin/ffmpeg"

import imageio
from tqdm.auto import tqdm


def is_notebook():
    """This utility function detects if the code is running in a notebook
    """
    try:
        get_ipython = sys.modules["IPython"].get_ipython
        if "IPKernelApp" not in get_ipython().config:  # pragma: no cover
            raise ImportError("console")
        if "VSCODE_PID" in os.environ:  # pragma: no cover
            raise ImportError("vscode")
    except:
        return False
    else:
        return True


def make_cfg(settings):
    """
    Build simulator, sensor and agent configurations

    Parameters
    ----------
    settings : dictionary containing pertinent setup information : 
        "scene" : full path/filename to test scene
        "enable_physics" : whether
        "camera_list" : list of tuples
            Each tuple value holds :
              idx 0 : name of camera
              idx 1 : whether camera is depth, semantic or color
              idx 2 : whether camera is 1st person or 3rd person
        "height" : camera height (same for all cameras)
        "width" : camera width (same for all cameras)
        "1st_POV_Loc" : location of 1st person camera (x,y,z)
        "1st_POV_Orient" : rotation of 1st person camera around x,y,z axis
        "3rd_POV_Loc" : location of 3rd person camera (x,y,z)
        "3rd_POV_Orient" : rotation of 3rd person camera around x,y,z axis

    Returns
    -------
    cfg : habitat_sim.Configuration
        Configuration required to instance necessary Habitat components.

    """
    # simulator configuration
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene.id = settings["scene"]
    assert os.path.exists(backend_cfg.scene.id)
    backend_cfg.enable_physics = settings["enable_physics"]

    # sensor configurations
    camera_list = settings["camera_list"]
    # sensor configurations
    # Note: all sensors must have the same resolution
    sensors_config_dict = {}
    for camera in camera_list:
        camera_name = camera[0]
        temp_dict = {}
        camera_type = camera[1].lower()
        # check if depth, semantic, or color
        if "depth" in camera_type:
            temp_dict["sensor_type"] = habitat_sim.SensorType.DEPTH
        elif "semantic" in camera_type:
            temp_dict["sensor_type"] = habitat_sim.SensorType.SEMANTIC
        else:
            temp_dict["sensor_type"] = habitat_sim.SensorType.COLOR

        # resolution should be height x width
        temp_dict["resolution"] = [settings["height"], settings["width"]]
        # check if 1st or 3rd person camera
        if ("firstperson" in camera[2].lower()) or ("1st" in camera[2].lower()):
            temp_dict["position"] = settings["1st_POV_Loc"]  # [0.0, 0.6, 0.0]
            # [0.0, 0.0, 0.0]
            temp_dict["orientation"] = settings["1st_POV_Orient"]
        else:
            temp_dict["position"] = settings["3rd_POV_Loc"]  # [0.0, 1.0, 0.3]
            # [-45, 0.0, 0.0]
            temp_dict["orientation"] = settings["3rd_POV_Orient"]
        sensors_config_dict[camera_name] = temp_dict

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
    video_file = output_path + file_name + ".mp4"
    print("Encoding the video: %s " % video_file)
    writer = get_fast_video_writer(video_file, fps=60)
    thumb_size = (int(videodims[0] / 5), int(videodims[1] / 5))
    outline_frame = np.ones((thumb_size[1] + 2, thumb_size[0] + 2, 3), np.uint8) * 150

    if multi_obs and len(pov_list) > 1:
        embed_image_list = []
        for i in range(0, len(pov_list)):
            # don't place primary POV in embedded image list
            if pov_list[i][0].lower() in primary_pov.lower():
                continue
            embed_image_list.append((pov_list[i][0], "depth" in pov_list[i][1].lower()))

        for ob in observations:
            # build list of embedded POV images and whether or not they are depth
            embed_image_data_list = [
                (ob[embed_image_list[i][0]], embed_image_list[i][1])
                for i in range(0, len(embed_image_list))
            ]

            res_image = build_multi_obs_image(
                ob[primary_pov], embed_image_data_list, thumb_size, outline_frame,
            )
            if res_image.shape[:2] != videodims:
                res_image = cv2.resize(
                    res_image, videodims, interpolation=cv2.INTER_AREA
                )

            # write the desired image to video
            writer.append_data(res_image)
    else:
        for ob in observations:
            res_image = cv2.cvtColor(ob[primary_pov], cv2.COLOR_RGBA2RGB)
            if res_image.shape[:2] != videodims:
                res_image = cv2.resize(
                    res_image, videodims, interpolation=cv2.INTER_AREA
                )
            # write the desired image to video
            writer.append_data(res_image)
    writer.close()
    if open_vid:
        print("Displaying video")
        display_video(video_file)


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
    # Main Image
    result_image = cv2.cvtColor(base_camera_img, cv2.COLOR_RGBA2RGB)

    x_offset = 50
    y_offset = 50
    y_bound = 10
    for embed_image, embed_image_is_depth in embed_image_data_list:

        if embed_image_is_depth:
            rgb_embed_image = clip_depth_image(embed_image)
        else:
            rgb_embed_image = cv2.cvtColor(embed_image, cv2.COLOR_RGBA2RGB)

        resized_rgb_embed_image = cv2.resize(
            rgb_embed_image, thumb_size, interpolation=cv2.INTER_AREA
        )

        result_image[
            y_offset - 1 : y_offset + outline_frame.shape[0] - 1,
            x_offset - 1 : x_offset + outline_frame.shape[1] - 1,
        ] = outline_frame
        result_image[
            y_offset : y_offset + resized_rgb_embed_image.shape[0],
            x_offset : x_offset + resized_rgb_embed_image.shape[1],
        ] = resized_rgb_embed_image
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
    rgb_d_im : 2d numpy array
        Clipped depth image data

    """
    d_im = np.clip(depth_image, 0, clip_max)
    d_im /= clip_max
    rgb_d_im = cv2.cvtColor((d_im * 255).astype(np.uint8), cv2.COLOR_GRAY2RGB)
    return rgb_d_im


def get_fast_video_writer(video_file: str, fps: int = 60):
    if (
        "google.colab" in sys.modules
        and os.path.splitext(video_file)[-1] == ".mp4"
        and os.environ.get("IMAGEIO_FFMPEG_EXE") == "/usr/bin/ffmpeg"
    ):
        # USE GPU Accelerated Hardware Encoding
        writer = imageio.get_writer(
            video_file,
            fps=fps,
            codec="h264_nvenc",
            mode="I",
            bitrate="1000k",
            format="FFMPEG",
            ffmpeg_log_level="info",
            output_params=["-minrate", "500k", "-maxrate", "5000k"],
        )
    else:
        # Use software encoding
        writer = imageio.get_writer(video_file, fps=fps)
    return writer


def save_video(video_file: str, frames, fps: int = 60):
    """Saves the video using imageio. Will try to use GPU hardware encoding on
    Google Colab for faster video encoding. Will also display a progressbar.

    :param video_file: the file name of where to save the video
    :param frames: the actual frame objects to save
    :param fps: the fps of the video (default 60)
    """
    writer = get_fast_video_writer(video_file, fps=fps)
    for ob in tqdm(frames, desc="Encoding video:%s" % video_file):
        writer.append_data(ob)
    writer.close()


def display_video(video_file: str, height: int = 400):
    """Displays a video both locally and in a notebook. Will display the video
    as an HTML5 video if in a notebook, otherwise it opens the video file using
    the default system viewer.

    :param video_file: the filename of the video to display
    :param height: the height to display the video in a notebook.
    """
    # Check if in notebook
    if is_notebook():
        from IPython import display as ipythondisplay
        from IPython.display import HTML

        ext = os.path.splitext(video_file)[-1][1:]
        video = io.open(video_file, "r+b").read()
        ipythondisplay.display(
            HTML(
                data="""<video alt="test" autoplay
          loop controls style="height: {2}px;">
          <source src="data:video/{1}';base64,{0}" type="video/{1}" />
          </video>""".format(
                    base64.b64encode(video).decode("ascii"), ext, height
                )
            )
        )
    else:
        if sys.platform == "win32":
            os.startfile(video_file)
        else:
            opener = "open" if sys.platform == "darwin" else "xdg-open"
            subprocess.call([opener, video_file])


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
