#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
import importlib.util
import itertools
import json
from os import path as osp
from typing import Any, Dict

import magnum as mn
import numpy as np
import pytest
import quaternion  # noqa: F401

import habitat_sim
import habitat_sim.errors
from habitat_sim.utils.common import quat_from_coeffs
from habitat_sim.utils.settings import make_cfg

torch_spec = importlib.util.find_spec("torch")
_HAS_TORCH = torch_spec is not None


def _render_scene(sim, scene, sensor_type, gpu2gpu):
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

    if gpu2gpu:
        torch = pytest.importorskip("torch")

        for k, v in obs.items():
            if torch.is_tensor(v):
                obs[k] = v.cpu().numpy()

    assert sensor_type in obs, f"{sensor_type} not in obs"

    return obs


def _render_and_load_gt(sim, scene, sensor_type, gpu2gpu):
    obs = _render_scene(sim, scene, sensor_type, gpu2gpu)

    # now that sensors are constructed, test some getter/setters
    if hasattr(sim.get_agent(0)._sensors[sensor_type], "fov"):
        sim.get_agent(0)._sensors[sensor_type].fov = mn.Deg(80)
        assert sim.get_agent(0)._sensors[sensor_type].fov == mn.Deg(
            80
        ), "fov not set correctly"
        assert sim.get_agent(0)._sensors[sensor_type].hfov == mn.Deg(
            80
        ), "hfov not set correctly"
    gt_obs_file = osp.abspath(
        osp.join(
            osp.dirname(__file__),
            "gt_data",
            "{}-{}.npy".format(osp.basename(osp.splitext(scene)[0]), sensor_type),
        )
    )
    # if not osp.exists(gt_obs_file):
    #    np.save(gt_obs_file, obs[sensor_type])
    gt = np.load(gt_obs_file)

    return obs, gt


# List of tuples holding scene ID and Scene Dataset Configuration to use.
# Scene Dataset Config is required to provide transform for semantic mesh,
# which, in Mp3d dataset, is oriented orthogonally to the render mesh
_semantic_scenes = [
    (
        osp.abspath(
            osp.join(
                osp.dirname(__file__),
                "../data/scene_datasets/mp3d/1LXtFkjw3qL/1LXtFkjw3qL.glb",
            )
        ),
        osp.abspath(
            osp.join(
                osp.dirname(__file__),
                "../data/scene_datasets/mp3d/mp3d.scene_dataset_config.json",
            )
        ),
    ),
    (
        osp.abspath(
            osp.join(
                osp.dirname(__file__),
                "../data/scene_datasets/mp3d_example/17DRP5sb8fy/17DRP5sb8fy.glb",
            )
        ),
        osp.abspath(
            osp.join(
                osp.dirname(__file__),
                "../data/scene_datasets/mp3d_example/mp3d.scene_dataset_config.json",
            )
        ),
    ),
]

# Non-semantic meshes can use default, built-in scene dataset config
_non_semantic_scenes = [
    (
        osp.abspath(
            osp.join(
                osp.dirname(__file__),
                "../data/scene_datasets/habitat-test-scenes/skokloster-castle.glb",
            )
        ),
        "default",
    ),
    (
        osp.abspath(
            osp.join(
                osp.dirname(__file__),
                "../data/scene_datasets/habitat-test-scenes/van-gogh-room.glb",
            )
        ),
        "default",
    ),
]
_test_scenes = _semantic_scenes + _non_semantic_scenes

all_base_sensor_types = [
    "color_sensor",
    "depth_sensor",
    "semantic_sensor",
]

# Sensors that don't have GT NPY files (yet)
all_exotic_sensor_types = [
    "ortho_rgba_sensor",
    "ortho_depth_sensor",
    "fisheye_rgba_sensor",
    "fisheye_depth_sensor",
    "equirect_rgba_sensor",
    "equirect_depth_sensor",
]

all_exotic_semantic_sensor_types = [
    "ortho_semantic_sensor",
    "fisheye_semantic_sensor",
    "equirect_semantic_sensor",
]


@pytest.mark.gfxtest
@pytest.mark.parametrize(
    "scene_and_dataset, sensor_type",
    list(itertools.product(_semantic_scenes, all_base_sensor_types))
    + list(itertools.product(_non_semantic_scenes, all_base_sensor_types[0:2]))
    + list(itertools.product(_test_scenes, all_exotic_sensor_types))
    + list(itertools.product(_semantic_scenes, all_exotic_semantic_sensor_types)),
)
@pytest.mark.parametrize("gpu2gpu", [True, False])
# NB: This should go last, we have to force a close on the simulator when
# this value changes, thus we should change it as little as possible
@pytest.mark.parametrize("frustum_culling", [True, False])
@pytest.mark.parametrize("add_sensor_lazy", [True, False])
def test_sensors(
    scene_and_dataset,
    sensor_type,
    gpu2gpu,
    frustum_culling,
    add_sensor_lazy,
    make_cfg_settings,
):
    scene = scene_and_dataset[0]
    if not osp.exists(scene):
        pytest.skip("Skipping {}".format(scene))
    if gpu2gpu and (not habitat_sim.cuda_enabled or not _HAS_TORCH):
        pytest.skip("Skipping GPU->GPU test")
    scene_dataset_config = scene_and_dataset[1]

    # We only support adding more RGB Sensors if one is already in a scene
    # We can add depth sensors whenever
    add_sensor_lazy = add_sensor_lazy and all_base_sensor_types[1] == sensor_type

    for sens in all_base_sensor_types:
        if add_sensor_lazy:
            make_cfg_settings[sens] = (
                sens in all_base_sensor_types[:2] and sens != sensor_type
            )
        else:
            make_cfg_settings[sens] = False

    make_cfg_settings[sensor_type] = True
    make_cfg_settings["scene"] = scene
    make_cfg_settings["scene_dataset_config_file"] = scene_dataset_config
    make_cfg_settings["frustum_culling"] = frustum_culling

    cfg = make_cfg(make_cfg_settings)
    if add_sensor_lazy:
        additional_sensors = cfg.agents[0].sensor_specifications[1:]
        cfg.agents[0].sensor_specifications = cfg.agents[0].sensor_specifications[:1]
    for sensor_spec in cfg.agents[0].sensor_specifications:
        sensor_spec.gpu2gpu_transfer = gpu2gpu

    with habitat_sim.Simulator(cfg) as sim:
        if add_sensor_lazy:
            obs: Dict[str, Any] = sim.reset()
            assert len(obs) == 1, "Other sensors were not removed"
            for sensor_spec in additional_sensors:
                sim.add_sensor(sensor_spec)
        if sensor_type not in all_base_sensor_types:
            obs = _render_scene(sim, scene, sensor_type, gpu2gpu)
            # Smoke Test.
            return
        obs, gt = _render_and_load_gt(sim, scene, sensor_type, gpu2gpu)

        # Different GPUs and different driver version will produce slightly
        # different images; differences on aliased edges might also stem from how a
        # particular importer parses transforms
        assert np.linalg.norm(
            obs[sensor_type].astype(float) - gt.astype(float)
        ) < 9.0e-2 * np.linalg.norm(gt.astype(float)), f"Incorrect {sensor_type} output"


@pytest.mark.gfxtest
@pytest.mark.parametrize("scene_and_dataset", _test_scenes)
@pytest.mark.parametrize("sensor_type", all_base_sensor_types[0:2])
def test_reconfigure_render(
    scene_and_dataset,
    sensor_type,
    make_cfg_settings,
):
    scene = scene_and_dataset[0]
    if not osp.exists(scene):
        pytest.skip("Skipping {}".format(scene))

    for sens in all_base_sensor_types:
        make_cfg_settings[sens] = False
    scene_dataset_config = scene_and_dataset[1]

    make_cfg_settings["scene"] = _test_scenes[-1][0]
    make_cfg_settings["scene_dataset_config_file"] = _test_scenes[-1][1]
    make_cfg_settings[sensor_type] = True

    cfg = make_cfg(make_cfg_settings)

    with habitat_sim.Simulator(cfg) as sim:
        make_cfg_settings["scene"] = scene
        make_cfg_settings["scene_dataset_config_file"] = scene_dataset_config
        sim.reconfigure(make_cfg(make_cfg_settings))
        obs, gt = _render_and_load_gt(sim, scene, sensor_type, False)

        # Different GPUs and different driver version will produce slightly
        # different images; differences on aliased edges might also stem from how a
        # particular importer parses transforms
        assert np.linalg.norm(
            obs[sensor_type].astype(float) - gt.astype(float)
        ) < 9.0e-2 * np.linalg.norm(gt.astype(float)), f"Incorrect {sensor_type} output"


# Tests to make sure that no sensors is supported and doesn't crash
# Also tests to make sure we can have multiple instances
# of the simulator with no sensors
def test_smoke_no_sensors(make_cfg_settings):
    sims = []
    for scene_and_dataset in _test_scenes:
        scene = scene_and_dataset[0]
        if not osp.exists(scene):
            continue
        scene_dataset_config = scene_and_dataset[1]
        make_cfg_settings["semantic_sensor"] = False
        make_cfg_settings["scene"] = scene
        make_cfg_settings["scene_dataset_config_file"] = scene_dataset_config
        cfg = make_cfg(make_cfg_settings)
        cfg.agents[0].sensor_specifications = []
        sims.append(habitat_sim.Simulator(cfg))
    for sim in sims:
        sim.close()


@pytest.mark.gfxtest
@pytest.mark.parametrize(
    "scene_and_dataset",
    _test_scenes,
)
@pytest.mark.parametrize("gpu2gpu", [True, False])
def test_smoke_redwood_noise(scene_and_dataset, gpu2gpu, make_cfg_settings):
    scene = scene_and_dataset[0]
    if not osp.exists(scene):
        pytest.skip("Skipping {}".format(scene))
    if gpu2gpu and (not habitat_sim.cuda_enabled or not _HAS_TORCH):
        pytest.skip("Skipping GPU->GPU test")
    scene_dataset_config = scene_and_dataset[1]
    make_cfg_settings["depth_sensor"] = True
    make_cfg_settings["color_sensor"] = False
    make_cfg_settings["semantic_sensor"] = False
    make_cfg_settings["scene"] = scene
    make_cfg_settings["scene_dataset_config_file"] = scene_dataset_config
    hsim_cfg = make_cfg(make_cfg_settings)
    hsim_cfg.agents[0].sensor_specifications[0].noise_model = "RedwoodDepthNoiseModel"
    for sensor_spec in hsim_cfg.agents[0].sensor_specifications:
        sensor_spec.gpu2gpu_transfer = gpu2gpu

    with habitat_sim.Simulator(hsim_cfg) as sim:
        obs, gt = _render_and_load_gt(sim, scene, "depth_sensor", gpu2gpu)

        assert np.linalg.norm(
            obs["depth_sensor"].astype(float) - gt.astype(float)
        ) > 1.5e-2 * np.linalg.norm(gt.astype(float)), "Incorrect depth_sensor output"

    sim.close()


@pytest.mark.gfxtest
@pytest.mark.parametrize("scene_and_dataset", _test_scenes)
@pytest.mark.parametrize("sensor_type", all_base_sensor_types[:2])
def test_initial_hfov(scene_and_dataset, sensor_type, make_cfg_settings):
    scene = scene_and_dataset[0]
    if not osp.exists(scene):
        pytest.skip("Skipping {}".format(scene))
    make_cfg_settings["hfov"] = 70
    with habitat_sim.Simulator(make_cfg(make_cfg_settings)) as sim:
        assert sim.agents[0]._sensors[sensor_type].hfov == mn.Deg(
            70
        ), "HFOV was not properly set"


@pytest.mark.gfxtest
@pytest.mark.parametrize("scene_and_dataset", _test_scenes)
@pytest.mark.parametrize(
    "model_name",
    [
        "SpeckleNoiseModel",
        "GaussianNoiseModel",
        "SaltAndPepperNoiseModel",
        "PoissonNoiseModel",
    ],
)
def test_rgba_noise(scene_and_dataset, model_name, make_cfg_settings):
    scene = scene_and_dataset[0]
    if not osp.exists(scene):
        pytest.skip("Skipping {}".format(scene))
    scene_dataset_config = scene_and_dataset[1]
    make_cfg_settings["depth_sensor"] = False
    make_cfg_settings["color_sensor"] = True
    make_cfg_settings["semantic_sensor"] = False
    make_cfg_settings["scene"] = scene
    make_cfg_settings["scene_dataset_config_file"] = scene_dataset_config
    hsim_cfg = make_cfg(make_cfg_settings)
    hsim_cfg.agents[0].sensor_specifications[0].noise_model = model_name

    with habitat_sim.Simulator(hsim_cfg) as sim:
        obs, gt = _render_and_load_gt(sim, scene, "color_sensor", False)

        assert np.linalg.norm(
            obs["color_sensor"].astype(float) - gt.astype(float)
        ) > 1.5e-2 * np.linalg.norm(gt.astype(float)), "Incorrect color_sensor output"
