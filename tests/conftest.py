# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os
import sys
from os import path as osp

sys.path.append(os.path.join(os.path.dirname(__file__), "helpers"))

import pytest

import habitat_sim

_test_scene = osp.abspath(
    osp.join(
        osp.dirname(__file__),
        "../data/scene_datasets/habitat-test-scenes/skokloster-castle.glb",
    )
)

# Testing configurations
@pytest.fixture(scope="function")
def make_cfg_settings():
    from habitat_sim.utils.settings import add_sensor_to_settings, default_sim_settings

    settings = default_sim_settings.copy()
    add_sensor_to_settings(settings, "color_sensor")
    add_sensor_to_settings(
        settings, "depth_sensor", sensor_type=habitat_sim.SensorType.DEPTH
    )
    add_sensor_to_settings(
        settings, "semantic_sensor", sensor_type=habitat_sim.SensorType.SEMANTIC
    )
    settings["silent"] = True
    settings["scene"] = _test_scene
    settings["frustum_culling"] = True
    return settings


def pytest_report_header(config):
    del config  # unused
    output = ["C++ Build Info:"]
    for setting in [
        "audio_enabled",
        "built_with_bullet",
        "cuda_enabled",
        "vhacd_enabled",
    ]:
        output.append(f"--{setting}: {getattr(habitat_sim, setting)}")
    return output
