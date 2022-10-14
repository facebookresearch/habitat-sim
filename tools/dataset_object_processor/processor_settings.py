# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from habitat_sim.utils.settings import default_sim_settings
from habitat_sim.utils.settings import make_cfg as _make_cfg

dataset_processor_settings = {
    "frustum_culling": True,
    "memory_data_headers": [
        "Object Template File",
        "Approx. Ram Use",
        "Render Asset File",
        "Collision Asset File",
        "Mesh Count",
        "Mesh Index Data Size",
        "Mesh Vertex Data Size",
        "Total Mesh Data Size",
        "Image Mip Map Count",
        "Image Data Size",
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
    # TODO: figure out what these metrics mean exactly
    # "mem_delta_order" is either -1 or 1. 1 means the delta is
    # calculated as (end_start - start_state), whereas -1 means
    # (start_state - end_state). E.g. Data "used" should be higher
    # after loading, so mem_delta_order == 1, but data free should
    # be higher before loading, so mem_delta_order == -1
    "mem_delta_order": {
        "available": -1,
        "percent": 1,
        "used": 1,
        "free": -1,
        "active": 1,
        "inactive": 1,
    },
}
default_sim_settings.update(dataset_processor_settings)


def make_cfg(settings):
    return _make_cfg(settings)
