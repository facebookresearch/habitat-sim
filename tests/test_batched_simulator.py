#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import torch  # isort:skip # noqa: F401  must import torch before importing bps_pytorch

from collections import OrderedDict
from typing import Dict, List

import bps_pytorch  # see https://github.com/shacklettbp/bps-nav#building
import numpy as np

from habitat_sim._ext.habitat_sim_bindings import BatchedSimulator
from habitat_sim.utils import viz_utils as vut

if __name__ == "__main__":

    SIMULATOR_GPU_ID = 0
    batch_size = 11
    double_buffered = False
    out_dim = [1024, 1024]

    bsim = BatchedSimulator()

    observations = []

    for buffer_index in range(2 if double_buffered else 1):

        observation = OrderedDict()

        observation["rgb"] = bps_pytorch.make_color_tensor(
            bsim.rgba(buffer_index),
            SIMULATOR_GPU_ID,
            batch_size // (2 if double_buffered else 1),
            out_dim,
        )[..., 0:3].permute(0, 3, 1, 2)

        observations.append(observation)

    batch_saved_observations: List[List[Dict]] = []
    for _ in range(batch_size):
        batch_saved_observations.append([])

    primary_obs_name = "rgba_camera"
    buffer_index = 0
    for _ in range(30):
        bsim.step_physics()
        bsim.start_render()
        bsim.wait_for_frame()

        # convert obs to numpy
        rgb = observations[buffer_index]["rgb"]
        if not isinstance(rgb, np.ndarray):
            rgb = rgb.cpu().numpy()
        for env_index in range(batch_size):
            env_rgb = rgb[env_index, ...]
            # [3, width, height] => [width, height, 3]
            env_rgb = np.swapaxes(env_rgb, 0, 2)
            env_rgb = np.swapaxes(env_rgb, 0, 1)
            env_saved_observations = batch_saved_observations[env_index]
            env_saved_observations.append({primary_obs_name: env_rgb})

    video_folder = "./videos"
    for env_index in range(batch_size):

        env_saved_observations = batch_saved_observations[env_index]
        vut.make_video(
            env_saved_observations,
            primary_obs_name,
            "color",
            video_folder + "/env_" + str(env_index) + "_rgba_camera",
            fps=10,  # very slow fps
            open_vid=False,
        )
