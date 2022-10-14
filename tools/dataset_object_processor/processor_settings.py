# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from habitat_sim.utils.settings import default_sim_settings
from habitat_sim.utils.settings import make_cfg as _make_cfg

dataset_processor_settings = {
    "frustum_culling": True,
    "memory_data_headers": [
        "Template Handle",
        "Approx. Ram Delta",
        "Mesh Count",
        "Mesh Index Data Size",
        "Mesh Vertex Data Size",
        "Total Mesh Data Size",
        "Image Mip Map Count",
        "Image Data Size",
        "CPU Memory Used",
    ],
    "sim_time_headers": [
        "Sim Time Ratio",
    ],
    "render_time_headers": [
        "Render Time Ratio",
    ],
    "physics_data_headers": [
        "Idle After...",
        "Stable Start Rotations",
        "Translation Drift",
        "Rotation Drift",
    ],
}
default_sim_settings.update(dataset_processor_settings)


def make_cfg(settings):
    return _make_cfg(settings)
