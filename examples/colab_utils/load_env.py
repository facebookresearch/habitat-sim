import os
import sys

##Sets up environment on Google Colab
if "google.colab" in sys.modules:
    conda_path = "/usr/local/lib/python3.6/site-packages/"
    user_path = "/root/.local/lib/python3.6/site-packages/"
    sys.path.insert(0, conda_path)
    sys.path.insert(0, user_path)
    os.environ["IMAGEIO_FFMPEG_EXE"] = "/usr/bin/ffmpeg"
