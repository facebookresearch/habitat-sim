# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from habitat_sim.utils.settings import default_sim_settings, make_cfg  # noqa: F401

example_settings = {
    "save_png": False,  # save the pngs to disk (default: OFF)
    "print_semantic_scene": False,
    "print_semantic_mask_stats": False,
    "compute_shortest_path": False,
    "compute_action_shortest_path": False,
    "scene": "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb",
    "test_scene_data_url": "http://dl.fbaipublicfiles.com/habitat/habitat-test-scenes.zip",
    "goal_position": [5.047, 0.199, 11.145],
    "enable_physics": False,
    "enable_gfx_replay_save": False,
    "physics_config_file": "./data/default.physics_config.json",
    "num_objects": 10,
    "test_object_index": 0,
    "frustum_culling": True,
}
default_sim_settings.update(example_settings)
