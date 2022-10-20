# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from habitat_sim.utils.settings import default_sim_settings
from habitat_sim.utils.settings import make_cfg as _make_cfg

dataset_processor_settings = {
    "scene": "data/test_assets/scenes/simple_room.glb",
    "data_to_collect": {
        "memory_data": True,
        "render_time_ratio": True,
        "physics_data": True,
    },
    "index_start_obj": 0,
    "num_objects": "all",
    "memory_vars": {
        "units": "KB",
        "max_mesh_num": 5,
        "max_mesh_size": 2400,
        "max_image_num": 32,
        "max_image_size": 12000,
    },
    "mem_metrics_to_use": ["available", "used", "free"],
    "physics_vars": {
        "fps": 60,
        "max_wait_time": 5,
        "time_units": "sec",
        "distance_units": "m",
        "angle_units": "deg",
    },
    "video_vars": {
        "tasks": {
            "show_bbox": True,
            "show_collision_asset": True,
            "show_bullet_collision_mesh": True,
        },
        "revolution_dur": 4.0,
    },
    "outputs": {"csv": True, "video": True},
    "output_paths": {
        "csv": "tools/dataset_object_processor/dataset_csv_files/",
        "video": "tools/dataset_object_processor/dataset_video_recordings/",
        "output_file_prefix": "",
    },
    "default_transforms": {
        "default_agent_pos": [0.25, 2.00, 1.73],
        "default_agent_rot": {"angle": 0.0, "axis": [1.0, 0.0, 0.0]},
        "default_obj_pos": [0.25, 3.00, -0.23],
        "default_obj_rot": {"angle": 0.0, "axis": [1.0, 0.0, 0.0]},
    },
    "headless": True,
    "enable_physics": True,
    "sensor_height": 1.0,
    "silent": True,
    "debug_print": False,
    # object_config.json file name
    "object_name": [
        "object template file",
    ],
    # Column titles of CSV file when memory stats are requested
    "memory_data_headers": [
        "MEMORY --->",
        "approx. RAM use",
        "render asset file",
        "collision asset file",
        "mesh count",
        "index data size",
        "vertex data size",
        "total mesh data size",
        "mip map count",
        "image data size",
    ],
    # Column titles of CSV file when rendering stats are requested
    "render_time_headers": [
        "RENDERING --->",
        "Render Time Ratio",
    ],
    # Column titles of CSV file when physics stats are requested
    "physics_data_headers": [
        "PHYSICS --->",
        "wait times\nfor rotations",
        "angle: 0,\naxis (1, 0, 0)",
        "angle: 90,\naxis (1, 0, 0)",
        "angle: 180,\naxis (1, 0, 0)",
        "angle: 90,\naxis (-1, 0, 0)",
        "angle: 90,\naxis (0, 0, -1)",
        "angle: 90,\naxis (0, 0, 1)",
        "position\ndeltas",
        "pose 1",
        "pose 2",
        "pose 3",
        "pose 4",
        "pose 5",
        "pose 6",
        "rotation\ndeltas",
        "pose 1",
        "pose 2",
        "pose 3",
        "pose 4",
        "pose 5",
        "pose 6",
        "(sim time)/dt",
        "pose 1",
        "pose 2",
        "pose 3",
        "pose 4",
        "pose 5",
        "pose 6",
    ],
    # we must test rigid objects in 6 different orientations, each corresponding
    # to a face of an imaginary cube bounding the object. Each rotation is of
    # the form:
    # (angle in degrees, (axis.x, axis.y, axis.z))
    "sim_test_rotations": [
        (0, (1, 0, 0)),
        (90, (1, 0, 0)),
        (180, (1, 0, 0)),
        (90, (-1, 0, 0)),
        (90, (0, 0, -1)),
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
