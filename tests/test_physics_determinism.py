#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import json
from os import path as osp

import magnum as mn
import numpy as np
import pytest

import examples.settings
import habitat_sim
import habitat_sim.physics


def get_all_object_states(sim):
    object_ids = sim.get_existing_object_ids()

    states = []
    for object_id in object_ids:
        states.append(
            {
                "object_id": object_id,
                "translation": np.array(sim.get_translation(object_id)).tolist(),
            }
        )
    return states


def get_ground_truth():
    file = open("src/esp/bindings_js/tests/physics_determinism.json", "r")
    print("olad")
    return json.loads(file.read())


def write_ground_truth_states(states):
    with open("src/esp/bindings_js/tests/physics_determinism.json", "w") as f:
        f.write(json.dumps(states))


@pytest.mark.skipif(
    not osp.exists("data/scene_datasets/habitat-test-scenes/skokloster-castle.glb")
    or not osp.exists("data/test_assets/objects/"),
    reason="Requires the habitat-test-scenes and habitat test objects",
)
def test_physics_determinism():
    cfg_settings = examples.settings.default_sim_settings.copy()

    cfg_settings[
        "scene"
    ] = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    # enable the physics simulator: also clears available actions to no-op
    cfg_settings["enable_physics"] = True
    cfg_settings["depth_sensor"] = True

    # test loading the physical scene
    hab_cfg = examples.settings.make_cfg(cfg_settings)
    with habitat_sim.Simulator(hab_cfg) as sim:
        sim.reset()
        obj_mgr = sim.get_object_template_manager()

        assert obj_mgr.get_num_templates() > 0

        # test adding an object to the world
        # get handle for object 0, used to test
        obj_handle_list = obj_mgr.get_template_handles("sphere")
        sphere_object_id = sim.add_object_by_handle(obj_handle_list[0])
        sphere_position = mn.Vector3(
            [-0.9517786502838135, 2.167676642537117, 11.343990325927734]
        )

        sim.set_object_motion_type(
            habitat_sim.physics.MotionType.DYNAMIC, sphere_object_id
        )

        sim.set_translation(sphere_position, sphere_object_id)

        obj_handle_list = obj_mgr.get_template_handles("chair")
        chair_object_id = sim.add_object_by_handle(obj_handle_list[0])

        chair_position = mn.Vector3(
            [-0.9517786502838135, 1.57676642537117, 11.343990325927734]
        )

        sim.set_object_motion_type(
            habitat_sim.physics.MotionType.DYNAMIC, chair_object_id
        )

        sim.set_translation(chair_position, chair_object_id)

        len(sim.get_existing_object_ids()) == 3

        states = []
        world_time = sim.get_world_time()
        step_count = 1
        states.append({"world_time": world_time, "step_count": 0, "object_states": []})
        while world_time < 3.0:
            sim.step_world()
            world_time = sim.get_world_time()
            states.append(
                {
                    "world_time": world_time,
                    "step_count": step_count,
                    "object_states": get_all_object_states(sim),
                }
            )
            step_count += 1

        ground_truth_states = get_ground_truth()

        assert len(states) == len(ground_truth_states)

        for i in range(len(states)):
            assert states[i]["world_time"] == ground_truth_states[i]["world_time"]
            assert states[i]["step_count"] == ground_truth_states[i]["step_count"]

            assert states[i].get("object_states") is not None

            if states[i].get("object_states"):
                object_states = states[i].get("object_states")
                ground_truth_object_states = ground_truth_states[i].get("object_states")
                for idx in range(len(object_states)):
                    assert (
                        object_states[idx]["object_id"]
                        == ground_truth_object_states[idx]["object_id"]
                    )
                    assert (
                        object_states[idx]["translation"]
                        == ground_truth_object_states[idx]["translation"]
                    )


if __name__ == "__main__":
    test_physics_determinism()
