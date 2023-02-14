# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import habitat_sim
from habitat_sim.utils.settings import default_sim_settings
from habitat_sim.utils.settings import make_cfg as _make_cfg

example_settings = {
    "max_frames": 1000,
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
    "num_objects": 10,
    "test_object_index": 0,
    "frustum_culling": True,
    "silent": False,  # do not print log info (default: OFF)
}
default_sim_settings.update(example_settings)


def make_cfg(settings):
    # Start with the built-in configuration, then modify as
    # appropriate.
    cfg = _make_cfg(settings)
    sim_cfg = cfg.sim_cfg
    agent_cfg = cfg.agents[0]

    if not settings["silent"]:
        print("sim_cfg.physics_config_file = " + sim_cfg.physics_config_file)

    # override action space to no-op to test physics
    if sim_cfg.enable_physics:
        agent_cfg.action_space = {
            "move_forward": habitat_sim.agent.ActionSpec(
                "move_forward", habitat_sim.agent.ActuationSpec(amount=0.0)
            )
        }

    return cfg
