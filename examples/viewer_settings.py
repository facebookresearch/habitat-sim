# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from habitat_sim.utils.settings import default_sim_settings
from habitat_sim.utils.settings import make_cfg as _make_cfg

viewer_settings = {
    # the maximum number of chars displayable in the app window
    # using the magnum text module. These chars are used to
    # display the CPU/GPU usage data
    "max_display_text_chars": 256,
    # how much to displace window text relative to the center of the
    # app window. E.g if you want the display text in the top left of
    # the app window, you will displace the text:
    #
    # window width * -sim_settings["text_delta_from_center"] in the x axis,
    #
    # and:
    #
    # window height * sim_settings["text_delta_from_center"] in the y axis,
    #
    # as the text position defaults to the middle of the app window)
    "text_delta_from_center": 0.49,
    # font size of the magnum in-window display text that displays
    # CPU and GPU usage info
    "display_font_size": 16.0,
    "display_font": "TrueTypeFont",
    "font_relative_path": "../data/fonts/ProggyClean.ttf",
    "fps": 60,
    "debug_bullet_draw": False,
    "debug_contact_draw": False,
    "cached_urdf": "",
    # number of frames to track for app data and CPU/GPU usage
    "num_frames_to_track": 60,
    "simulating": True,
    "silent": False,  # do not print log info (default: OFF)
}
default_sim_settings.update(viewer_settings)


def make_cfg(settings):
    return _make_cfg(settings)
