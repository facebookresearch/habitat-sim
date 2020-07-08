import base64
import io
import os
import sys

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


def save_video(video_file: str, frames, fps: int = 60):
    """Saves the video using imageio. Will try to use GPU hardware encoding on
    Google Colab for faster video encoding. Will also display a progressbar.

    :param video_file: the file name of where to save the video
    :param frames: the actual frame objects to save
    :param fps: the fps of the video (default 60)
    """
    if "google.colab" in sys.modules and os.splitext(video_file)[-1] == ".mp4":
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
    for ob in tqdm(observations, desc="Encoding video:%s" % video_file):
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
        from IPython.display import HTML
        from IPython import display as ipythondisplay

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
        # Opens the video in the system default viewer.
        os.startfile(video_file)
