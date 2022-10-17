# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from habitat_sim.utils.settings import default_sim_settings
from habitat_sim.utils.settings import make_cfg as _make_cfg

# TODO: maybe move this to a separate file as they are not "settings"
# per se
dataset_processor_settings = {
    # Column titles that are always displayed in CSV file
    "always_displayed": [
        "Object Template File",
        "Approx. Ram Use",
    ],
    # Column titles of CSV file when memory stats are requested
    "memory_data_headers": [
        "Render Asset File",
        "Collision Asset File",
        "Mesh Count",
        "Mesh Index Data Size",
        "Mesh Vertex Data Size",
        "Total Mesh Data Size",
        "Image Mip Map Count",
        "Image Data Size",
    ],
    # Column titles of CSV file when rendering stats are requested
    "render_time_headers": [
        "Render Time Ratio",
    ],
    # Column titles of CSV file when physics stats are requested
    "physics_data_headers": [
        "Sim Time Ratio",
        "Awake Durations\nfor Rotations ->",
        "Angle: 0,\nAxis (1, 0, 0)",
        "Angle: 90,\nAxis (1, 0, 0)",
        "Angle: 180,\nAxis (1, 0, 0)",
        "Angle: 270,\nAxis (1, 0, 0)",
        "Angle: -90,\nAxis (0, 0, 1)",
        "Angle: 90,\nAxis (0, 0, 1)",
        "Stable Start Rotations",
        "Translation Drift",
        "Translation Drift",
        "Translation Drift",
        "Translation Drift",
        "Translation Drift",
        "Translation Drift",
        "Rotation Drift",
        "Rotation Drift",
        "Rotation Drift",
        "Rotation Drift",
        "Rotation Drift",
        "Rotation Drift",
    ],
    # we must test rigid objects in 6 different orientations, each corresponding
    # to a face of an imaginary cube bounding the object. Each rotation is of
    # the form:
    # (angle in degrees, (axis.x, axis.y, axis.z))
    "sim_test_rotations": [
        (0, (1, 0, 0)),
        (90, (1, 0, 0)),
        (180, (1, 0, 0)),
        (270, (1, 0, 0)),
        (-90, (0, 0, 1)),
        (90, (0, 0, 1)),
    ],
    # TODO: make sure these are right
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
        "active": -1,
        "inactive": 1,
    },
    "bbox_rgb": [1.0, 0.8, 1.0],
}
default_sim_settings.update(dataset_processor_settings)


def make_cfg(settings):
    return _make_cfg(settings)
