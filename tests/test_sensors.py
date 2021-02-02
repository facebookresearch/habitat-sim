#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import itertools
import json
from os import path as osp

import numpy as np
import pytest
import quaternion  # noqa: F401

import habitat_sim
import habitat_sim.errors
from examples.settings import make_cfg
from habitat_sim.utils.common import quat_from_coeffs


def _render_and_load_gt(sim, scene, sensor_type, gpu2gpu):
    gt_data_pose_file = osp.abspath(
        osp.join(
            osp.dirname(__file__),
            "gt_data",
            "{}-state.json".format(osp.basename(osp.splitext(scene)[0])),
        )
    )
    with open(gt_data_pose_file, "r") as f:
        render_state = json.load(f)
        state = habitat_sim.AgentState()
        state.position = render_state["pos"]
        state.rotation = quat_from_coeffs(render_state["rot"])

    sim.initialize_agent(0, state)
    obs = sim.step("move_forward")

    assert sensor_type in obs, f"{sensor_type} not in obs"

    gt_obs_file = osp.abspath(
        osp.join(
            osp.dirname(__file__),
            "gt_data",
            "{}-{}.npy".format(osp.basename(osp.splitext(scene)[0]), sensor_type),
        )
    )
    gt = np.load(gt_obs_file)

    if gpu2gpu:
        torch = pytest.importorskip("torch")

        for k, v in obs.items():
            if torch.is_tensor(v):
                obs[k] = v.cpu().numpy()

    return obs, gt


_test_scenes = [
    osp.abspath(
        osp.join(
            osp.dirname(__file__),
            "../data/scene_datasets/mp3d/1LXtFkjw3qL/1LXtFkjw3qL.glb",
        )
    ),
    osp.abspath(
        osp.join(
            osp.dirname(__file__),
            "../data/scene_datasets/mp3d/17DRP5sb8fy/17DRP5sb8fy.glb",
        )
    ),
    osp.abspath(
        osp.join(
            osp.dirname(__file__),
            "../data/scene_datasets/habitat-test-scenes/skokloster-castle.glb",
        )
    ),
    osp.abspath(
        osp.join(
            osp.dirname(__file__),
            "../data/scene_datasets/habitat-test-scenes/van-gogh-room.glb",
        )
    ),
]

all_sensor_types = ["color_sensor", "depth_sensor", "semantic_sensor"]


@pytest.mark.gfxtest
@pytest.mark.parametrize(
    "scene,sensor_type",
    list(itertools.product(_test_scenes[0:2], all_sensor_types))
    + list(itertools.product(_test_scenes[2:], all_sensor_types[0:2])),
)
@pytest.mark.parametrize("gpu2gpu", [True, False])
# NB: This should go last, we have to force a close on the simulator when
# this value changes, thus we should change it as little as possible
@pytest.mark.parametrize("frustum_culling", [True, False])
@pytest.mark.parametrize("add_sensor_lazy", [True, False])
def test_sensors(
    scene,
    sensor_type,
    gpu2gpu,
    frustum_culling,
    add_sensor_lazy,
    make_cfg_settings,
):
    if not osp.exists(scene):
        pytest.skip("Skipping {}".format(scene))

    if not habitat_sim.cuda_enabled and gpu2gpu:
        pytest.skip("Skipping GPU->GPU test")

    # We only support adding more RGB Sensors if one is already in a scene
    # We can add depth sensors whenever
    add_sensor_lazy = add_sensor_lazy and all_sensor_types[1] == sensor_type

    for sens in all_sensor_types:
        if add_sensor_lazy:
            make_cfg_settings[sens] = (
                sens in all_sensor_types[:2] and sens != sensor_type
            )
        else:
            make_cfg_settings[sens] = False

    make_cfg_settings[sensor_type] = True
    make_cfg_settings["scene"] = scene
    make_cfg_settings["frustum_culling"] = frustum_culling

    cfg = make_cfg(make_cfg_settings)
    if add_sensor_lazy:
        additional_sensors = cfg.agents[0].sensor_specifications[1:]
        cfg.agents[0].sensor_specifications = cfg.agents[0].sensor_specifications[:1]
    for sensor_spec in cfg.agents[0].sensor_specifications:
        sensor_spec.gpu2gpu_transfer = gpu2gpu

    with habitat_sim.Simulator(cfg) as sim:
        if add_sensor_lazy:
            obs: np.ndarray = sim.reset()
            assert len(obs) == 1, "Other sensors were not removed"
            for sensor_spec in additional_sensors:
                sim.add_sensor(sensor_spec)
        obs, gt = _render_and_load_gt(sim, scene, sensor_type, gpu2gpu)

        # Different GPUs and different driver version will produce slightly
        # different images; differences on aliased edges might also stem from how a
        # particular importer parses transforms
        assert np.linalg.norm(
            obs[sensor_type].astype(np.float) - gt.astype(np.float)
        ) < 9.0e-2 * np.linalg.norm(
            gt.astype(np.float)
        ), f"Incorrect {sensor_type} output"


@pytest.mark.gfxtest
@pytest.mark.parametrize("scene", _test_scenes)
@pytest.mark.parametrize("sensor_type", all_sensor_types[0:2])
def test_reconfigure_render(
    scene,
    sensor_type,
    make_cfg_settings,
):
    if not osp.exists(scene):
        pytest.skip("Skipping {}".format(scene))

    for sens in all_sensor_types:
        make_cfg_settings[sens] = False

    make_cfg_settings["scene"] = _test_scenes[-1]
    make_cfg_settings[sensor_type] = True

    cfg = make_cfg(make_cfg_settings)

    with habitat_sim.Simulator(cfg) as sim:
        make_cfg_settings["scene"] = scene
        sim.reconfigure(make_cfg(make_cfg_settings))
        obs, gt = _render_and_load_gt(sim, scene, sensor_type, False)

        # Different GPUs and different driver version will produce slightly
        # different images; differences on aliased edges might also stem from how a
        # particular importer parses transforms
        assert np.linalg.norm(
            obs[sensor_type].astype(np.float) - gt.astype(np.float)
        ) < 9.0e-2 * np.linalg.norm(
            gt.astype(np.float)
        ), f"Incorrect {sensor_type} output"

    sim.close()


# Tests to make sure that no sensors is supported and doesn't crash
# Also tests to make sure we can have multiple instances
# of the simulator with no sensors
def test_smoke_no_sensors(make_cfg_settings):
    sims = []
    for scene in _test_scenes:
        if not osp.exists(scene):
            continue

        make_cfg_settings = {k: v for k, v in make_cfg_settings.items()}
        make_cfg_settings["semantic_sensor"] = False
        make_cfg_settings["scene"] = scene
        cfg = make_cfg(make_cfg_settings)
        cfg.agents[0].sensor_specifications = []
        sims.append(habitat_sim.Simulator(cfg))


@pytest.mark.gfxtest
@pytest.mark.parametrize(
    "scene,gpu2gpu", itertools.product(_test_scenes, [True, False])
)
def test_smoke_redwood_noise(scene, gpu2gpu, make_cfg_settings):
    if not osp.exists(scene):
        pytest.skip("Skipping {}".format(scene))

    if not habitat_sim.cuda_enabled and gpu2gpu:
        pytest.skip("Skipping GPU->GPU test")

    make_cfg_settings["depth_sensor"] = True
    make_cfg_settings["color_sensor"] = False
    make_cfg_settings["semantic_sensor"] = False
    make_cfg_settings["scene"] = scene
    hsim_cfg = make_cfg(make_cfg_settings)
    hsim_cfg.agents[0].sensor_specifications[0].noise_model = "RedwoodDepthNoiseModel"
    for sensor_spec in hsim_cfg.agents[0].sensor_specifications:
        sensor_spec.gpu2gpu_transfer = gpu2gpu

    with habitat_sim.Simulator(hsim_cfg) as sim:
        obs, gt = _render_and_load_gt(sim, scene, "depth_sensor", gpu2gpu)

        assert np.linalg.norm(
            obs["depth_sensor"].astype(np.float) - gt.astype(np.float)
        ) > 1.5e-2 * np.linalg.norm(
            gt.astype(np.float)
        ), "Incorrect depth_sensor output"

    sim.close()


@pytest.mark.gfxtest
@pytest.mark.parametrize("scene", _test_scenes)
@pytest.mark.parametrize(
    "model_name",
    [
        "SpeckleNoiseModel",
        "GaussianNoiseModel",
        "SaltAndPepperNoiseModel",
        "PoissonNoiseModel",
    ],
)
def test_rgb_noise(scene, model_name, make_cfg_settings):
    if not osp.exists(scene):
        pytest.skip("Skipping {}".format(scene))

    make_cfg_settings["depth_sensor"] = False
    make_cfg_settings["color_sensor"] = True
    make_cfg_settings["semantic_sensor"] = False
    make_cfg_settings["scene"] = scene
    hsim_cfg = make_cfg(make_cfg_settings)
    hsim_cfg.agents[0].sensor_specifications[0].noise_model = model_name

    with habitat_sim.Simulator(hsim_cfg) as sim:
        obs, gt = _render_and_load_gt(sim, scene, "color_sensor", False)

        assert np.linalg.norm(
            obs["color_sensor"].astype(np.float) - gt.astype(np.float)
        ) > 1.5e-2 * np.linalg.norm(
            gt.astype(np.float)
        ), "Incorrect color_sensor output"
