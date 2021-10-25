#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from os import path as osp

import magnum as mn
import numpy as np
import pytest
import quaternion  # noqa: F401

import examples.settings
import habitat_sim


@pytest.mark.skipif(
    not osp.exists("data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"),
    reason="Requires the habitat-test-scenes",
)
def test_unproject():
    cfg_settings = examples.settings.default_sim_settings.copy()

    # configure some settings in case defaults change
    cfg_settings["scene"] = "data/scene_datasets/habitat-test-scenes/apartment_1.glb"
    cfg_settings["width"] = 101
    cfg_settings["height"] = 101
    cfg_settings["sensor_height"] = 0
    cfg_settings["color_sensor"] = True

    # loading the scene
    hab_cfg = examples.settings.make_cfg(cfg_settings)
    with habitat_sim.Simulator(hab_cfg) as sim:
        # position agent
        sim.agents[0].scene_node.rotation = mn.Quaternion()
        sim.agents[0].scene_node.translation = mn.Vector3(0.5, 0, 0)

        # setup camera
        render_camera = sim._sensors["color_sensor"]._sensor_object.render_camera

        # test unproject
        center_ray = render_camera.unproject(
            mn.Vector2i(50, 50)
        )  # middle of the viewport
        assert np.allclose(center_ray.origin, np.array([0.5, 0, 0]), atol=0.07)
        assert np.allclose(center_ray.direction, np.array([0, 0, -1.0]), atol=0.02)

        test_ray_2 = render_camera.unproject(
            mn.Vector2i(100, 100)
        )  # bottom right of the viewport
        assert np.allclose(
            test_ray_2.direction, np.array([0.569653, -0.581161, -0.581161]), atol=0.07
        )


@pytest.mark.parametrize(
    "sensor_type",
    [
        habitat_sim.SensorType.COLOR,
        habitat_sim.SensorType.DEPTH,
        habitat_sim.SensorType.SEMANTIC,
    ],
)
def test_empty_scene(sensor_type):
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene_id = "NONE"

    agent_cfg = habitat_sim.AgentConfiguration()
    agent_cfg.sensor_specifications = [habitat_sim.CameraSensorSpec()]
    agent_cfg.sensor_specifications[-1].sensor_type = sensor_type

    with habitat_sim.Simulator(
        habitat_sim.Configuration(backend_cfg, [agent_cfg])
    ) as sim:
        _ = sim.get_sensor_observations()
