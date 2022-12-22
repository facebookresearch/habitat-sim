# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import habitat_sim
from habitat_sim.utils.settings import default_sim_settings
from habitat_sim.utils.settings import make_cfg as _make_cfg

qa_scene_settings = {
    "scene": "./data/scene_datasets/replica_cad/configs/scenes/apt_1.scene_instance.json",
    "scene_dataset_config_file": "scene_datasets/replica_cad/replicaCAD.scene_dataset_config.json",
    "make_csv": True,
    "make_video": True,
    "show_video": True,
    "run_viewer": False,
    "output_file_prefix": "default",
    "enable_physics": True,
    "enable_gfx_replay_save": False,
    "frustum_culling": True,
    "stage_requires_lighting": True,
    "silent": False,  # do not print log info (default: OFF)
}
default_sim_settings.update(qa_scene_settings)


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
