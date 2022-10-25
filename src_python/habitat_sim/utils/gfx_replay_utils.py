# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import magnum as mn

import habitat_sim


def add_node_user_transform(sim, node, name):
    translation = node.absolute_translation
    rot = mn.Quaternion(mn.Quaterniond(node.rotation))
    while True:
        node = node.parent
        if not node:
            break
        try:
            rot = node.rotation * rot
        except AttributeError:
            # scene root has no rotation attribute
            break
    sim.gfx_replay_manager.add_user_transform_to_keyframe(name, translation, rot)


def make_backend_configuration_for_playback(
    need_separate_semantic_scene_graph=False,
):
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene_id = "NONE"  # see Asset.h EMPTY_SCENE
    backend_cfg.force_separate_semantic_scene_graph = need_separate_semantic_scene_graph

    return backend_cfg
