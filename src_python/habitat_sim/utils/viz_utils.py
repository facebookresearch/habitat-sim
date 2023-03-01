# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import base64
import io
import os
import subprocess
import sys
from functools import partial

if "google.colab" in sys.modules:
    os.environ["IMAGEIO_FFMPEG_EXE"] = "/usr/bin/ffmpeg"

import random
from typing import Any, Dict, List, Optional, Tuple

import imageio
import numpy as np
from PIL import Image
from tqdm.auto import tqdm

from habitat_sim.utils.common import d3_40_colors_hex, d3_40_colors_rgb


def is_notebook() -> bool:
    """This utility function detects if the code is running in a notebook"""
    try:
        get_ipython = sys.modules["IPython"].get_ipython  # type: ignore[attr-defined]
        if "IPKernelApp" not in get_ipython().config:  # pragma: no cover
            raise ImportError("console")
        if "VSCODE_PID" in os.environ:  # pragma: no cover
            raise ImportError("vscode")
    except (AttributeError, ImportError, KeyError):
        return False
    else:
        return True


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
            format="FFMPEG",  # type: ignore[arg-type]
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
            os.startfile(video_file)  # type: ignore[attr-defined]
        else:
            opener = "open" if sys.platform == "darwin" else "xdg-open"
            subprocess.call([opener, video_file])


def observation_to_image(
    observation_image: np.ndarray,
    observation_type: str,
    depth_clip: Optional[float] = 10.0,
):
    """Generate an rgb image from a sensor observation. Supported types are: "color", "depth", "semantic"

    :param observation_image: Raw observation image from sensor output.
    :param observation_type: Observation type ("color", "depth", "semantic" supported)
    :param depth_clip: Defines default depth clip normalization for all depth images.

    :return: PIL Image object or None if fails.
    """
    rgb_image = None
    if observation_type == "color":
        rgb_image = Image.fromarray(np.uint8(observation_image))
    elif observation_type == "depth":
        rgb_image = Image.fromarray(
            depth_to_rgb(observation_image, clip_max=depth_clip)
        )
    elif observation_type == "semantic":
        rgb_image = semantic_to_rgb(observation_image)
    else:
        print(
            "semantic_to_rgb : Failed, unsupported observation type: "
            + observation_type
        )
    return rgb_image


def border_frames_from_overlay(
    overlay_settings, observation_to_image=observation_to_image
):
    border_frames = []
    if overlay_settings is not None:
        for overlay in overlay_settings:
            border_image = np.zeros(
                (
                    overlay["dims"][1] + overlay["border"] * 2,
                    overlay["dims"][0] + overlay["border"] * 2,
                    3,
                ),
                np.uint8,
            )
            border_color = np.ones(3) * 150
            if "border_color" in overlay:
                border_color = np.asarray(overlay["border_color"])
            border_image[:, :] = border_color
            border_frames.append(observation_to_image(border_image, "color"))
    return border_frames


def make_video_frame(
    ob,
    primary_obs: str,
    primary_obs_type: str,
    video_dims,
    overlay_settings=None,
    observation_to_image=observation_to_image,
):
    image_frame = observation_to_image(ob[primary_obs], primary_obs_type)
    if image_frame is None:
        raise RuntimeError(
            "make_video_new : Aborting, primary image processing failed."
        )

    # build the border frames for the overlays and validate settings
    border_frames = border_frames_from_overlay(
        overlay_settings, observation_to_image=observation_to_image
    )

    # overlay images from provided settings
    if overlay_settings is not None:
        for ov_ix, overlay in enumerate(overlay_settings):
            overlay_rgb_img = observation_to_image(ob[overlay["obs"]], overlay["type"])
            if overlay_rgb_img is None:
                raise RuntimeError(
                    'make_video_new : Aborting, overlay image processing failed on "'
                    + overlay["obs"]
                    + '".'
                )
            overlay_rgb_img = overlay_rgb_img.resize(overlay["dims"])
            image_frame.paste(
                border_frames[ov_ix],
                box=(
                    overlay["pos"][0] - overlay["border"],
                    overlay["pos"][1] - overlay["border"],
                ),
            )
            image_frame.paste(overlay_rgb_img, box=overlay["pos"])

    if video_dims is not None:
        image_frame = image_frame.resize(video_dims)
    return image_frame


def make_video(
    observations: List[np.ndarray],
    primary_obs: str,
    primary_obs_type: str,
    video_file: str,
    fps: int = 60,
    open_vid: bool = True,
    video_dims: Optional[Tuple[int]] = None,
    overlay_settings: Optional[List[Dict[str, Any]]] = None,
    depth_clip: Optional[float] = 10.0,
    observation_to_image=observation_to_image,
):
    """Build a video from a passed observations array, with some images optionally overlayed.
    :param observations: List of observations from which the video should be constructed.
    :param primary_obs: Sensor name in observations to be used for primary video images.
    :param primary_obs_type: Primary image observation type ("color", "depth", "semantic" supported).
    :param video_file: File to save resultant .mp4 video.
    :param fps: Desired video frames per second.
    :param open_vid: Whether or not to open video upon creation.
    :param video_dims: Height by Width of video if different than observation dimensions. Applied after overlays.
    :param overlay_settings: List of settings Dicts, optional.
    :param depth_clip: Defines default depth clip normalization for all depth images.
    :param observation_to_image: Allows overriding the observation_to_image function
    With **overlay_settings** dicts specifying per-entry: \n
        "type": observation type ("color", "depth", "semantic" supported)\n
        "dims": overlay dimensions (Tuple : (width, height))\n
        "pos": overlay position (top left) (Tuple : (width, height))\n
        "border": overlay image border thickness (int)\n
        "border_color": overlay image border color [0-255] (3d: array, list, or tuple). Defaults to gray [150]\n
        "obs": observation key (string)\n
    """
    if not video_file.endswith(".mp4"):
        video_file = video_file + ".mp4"
    print("Encoding the video: %s " % video_file)
    writer = get_fast_video_writer(video_file, fps=fps)
    observation_to_image = partial(observation_to_image, depth_clip=depth_clip)

    for ob in observations:
        # primary image processing
        image_frame = make_video_frame(
            ob,
            primary_obs,
            primary_obs_type,
            video_dims,
            overlay_settings=overlay_settings,
            observation_to_image=observation_to_image,
        )

        # write the desired image to video
        writer.append_data(np.array(image_frame))

    writer.close()
    if open_vid:
        display_video(video_file)


def depth_to_rgb(depth_image: np.ndarray, clip_max: float = 10.0) -> np.ndarray:
    """Normalize depth image into [0, 1] and convert to grayscale rgb

    :param depth_image: Raw depth observation image from sensor output.
    :param clip_max: Max depth distance for clipping and normalization.

    :return: Clipped grayscale depth image data.
    """
    d_im = np.clip(depth_image, 0, clip_max)
    d_im /= clip_max
    rgb_d_im = (d_im * 255).astype(np.uint8)
    return np.asarray(rgb_d_im)


def semantic_to_rgb(semantic_image: np.ndarray) -> np.ndarray:
    """Map semantic ids to colors and genereate an rgb image

    :param semantic_image: Raw semantic observation image from sensor output.

    :return: rgb semantic image data.
    """
    semantic_image_rgb = Image.new(
        "P", (semantic_image.shape[1], semantic_image.shape[0])
    )
    semantic_image_rgb.putpalette(d3_40_colors_rgb.flatten())
    semantic_image_rgb.putdata((semantic_image.flatten() % 40).astype(np.uint8))
    semantic_image_rgb = semantic_image_rgb.convert("RGBA")
    return semantic_image_rgb


def get_island_colored_map(island_top_down_map_data: np.ndarray):
    """
    Get the topdown map for a scene with island colors.

    :param island_top_down_map_data: The island index map data from Pathfinder.get_topdown_island_view()

    :return: rgb Image of islands at the desired slice.
    """

    white = int("0xffffff", base=16)
    island_map = Image.new("RGB", island_top_down_map_data.shape, color=white)
    pixels = island_map.load()
    extra_colors: List[int] = []
    r = lambda: random.randint(0, 255)
    for x in range(island_top_down_map_data.shape[0]):
        for y in range(island_top_down_map_data.shape[1]):
            if island_top_down_map_data[x, y] >= 0:
                color_index = island_top_down_map_data[x, y]
                if color_index < len(d3_40_colors_hex):
                    # fixed colors from a selected list
                    # NOTE: PIL Image origin is top left, so invert y
                    pixels[x, -y] = int(d3_40_colors_hex[color_index], base=16)
                else:
                    random_color_index = color_index - len(d3_40_colors_hex)
                    # pick random colors once fixed colors are overflowed
                    while random_color_index >= len(extra_colors):
                        new_color = int(("0x%02X%02X%02X" % (r(), r(), r())), base=16)
                        if new_color not in extra_colors:
                            extra_colors.append(new_color)
                    pixels[x, -y] = extra_colors[random_color_index]
    return island_map
