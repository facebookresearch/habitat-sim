# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from habitat_sim.utils.settings import default_sim_settings

qa_scene_settings = {
    # These can be overridden in a qa_scene_config.json file in the
    # "./tools/qa_scenes/configs" directory
    # standard SimulatorConfiguration settings
    "scene_dataset_config_file": "replica_cad/replicaCAD.scene_dataset_config.json",
    "color_sensor": True,
    "semantic_sensor": True,
    "depth_sensor": True,
    "width": 256,
    "height": 256,
    "enable_physics": True,
    # set the desired scenes
    "start_scene_index": 0,
    "end_scene_index": -1,  # -1 indicates end of list
    "process_scene_instances": True,  # includes all dataset scene instances
    "process_stages": True,  # includes all dataset stages
    "output_file_prefix": "default",
    "generate_navmesh": True,
    "navmesh_include_static": True,
    "save_navmesh_island_maps": True,  # if true, saves a set of navmesh topdown island maps for all floors (detected heuristically)
    "render_sensor_obs": True,
    "make_video": True,
    "show_video": True,
    "run_collision_test": True,
    "run_asset_sleep_test": True,
    "silent": False,  # do not print log info
    # primarily for debugging. When "run_viewer" is True, it won't output results as
    # files, but rather, you can look at the scene and press "c" to run the collision
    # test and watch it
    "run_viewer": False,
    "scene": "apt_0",
    "agent_pos": [-1.6971087, 0.11937292, -1.0771935],
    "agent_rot": [0.0, 0.0, 1.0, 0.0],  # (angle, axis.x, axis.y, axis.z)
    # for output json storing detailed scene information. "render_test_max_time"
    # can be overridden in a qa_scene_config.json file in the
    # "./tools/qa_scenes/configs" directory. Don't override the data in
    # "render_test_json_entry", as it is used as a template for json construction
    "render_test_max_time": 0.005,
    "render_test_json_entry": {
        "cardinal_dir_render_times": [0.0, 0.0, 0.0, 0.0],
        "overhead_render_time": 0.0,
        "min_render_time": 0.0,
        "max_render_time": 0.0,
        "avg_render_time": 0.0,
        "exceeds_time_threshold": False,
    },
    # for output json storing detailed scene information. "collision_test_cell_size"
    # and "collision_test_max_time_threshold" can be overridden in a qa_scene_config.json file in
    # the "./tools/qa_scenes/configs" directory. Don't override the data in
    # "collision_test_json_entry", as it is used as a template for json construction
    "collision_test_save_detail_json": True,
    "collision_test_cell_size": 1.0,
    "collision_test_max_time_threshold": 0.005,
    "collision_test_json_entry": {
        "collision_test_num": 0,
        "ijk_indices": [0, 0, 0],
        "pos": [0.0, 0.0, 0.0],
        "test_time": 0.0,
        "exceeds_time_threshold": False,
    },
    # settling test
    "sleep_test_steps_per_sec": 60,
    "asset_sleep_test_duration_seconds": 10.0,
    "asleep_test_json_entry": {
        "asset_handle": "",
        "test_time": 0.0,
        "successfully_sleeps": False,
    },
    # TODO: may not need all of these
    "enable_gfx_replay_save": False,
    "frustum_culling": True,
    "stage_requires_lighting": True,
    "fps": 60.0,
}
default_sim_settings.update(qa_scene_settings)
