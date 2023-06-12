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
    import habitat_sim.utils.settings

    cfg = habitat_sim.utils.settings.default_sim_settings.copy()
    cfg["height"] = 480
    cfg["width"] = 640
    cfg["sensor_height"] = 1.5
    cfg["color_sensor"] = True
    cfg["semantic_sensor"] = True
    cfg["depth_sensor"] = True
    cfg["silent"] = True
    cfg["scene"] = _test_scene
    cfg["frustum_culling"] = True
    return cfg


def pytest_report_header(config):
    del config  # unused
    output = ["C++ Build Info:"]
    for setting in [
        "audio_enabled",
        "built_with_bullet",
        "cuda_enabled",
    ]:
        output.append(f"--{setting}: {getattr(habitat_sim, setting)}")
    return output
