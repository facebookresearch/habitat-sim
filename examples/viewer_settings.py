# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from habitat_sim.utils.settings import default_sim_settings
from habitat_sim.utils.settings import make_cfg as _make_cfg

viewer_settings = {
    "sim_test_rotations": [
        (0, (1, 0, 0)),
        (90, (1, 0, 0)),
        (180, (1, 0, 0)),
        (270, (1, 0, 0)),
        (-90, (0, 0, 1)),
        (90, (0, 0, 1)),
    ],
    "display_text": {
        "max_display_text_chars": 256,
        "text_delta_from_center": 0.49,
        "display_font_size": 16.0,
        "font_path": "../data/fonts/ProggyClean.ttf",
        "render_frames_to_track": 30,
    },
    "bbox_rgb": [1.0, 0.8, 1.0],
    "silent": False,  # do not print log info (default: OFF)
}
default_sim_settings.update(viewer_settings)


def make_cfg(settings):
    return _make_cfg(settings)
