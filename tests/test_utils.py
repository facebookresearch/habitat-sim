#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# Need to import quaternion library here despite it not being used or else importing
# habitat_sim below will cause an invalid free() when audio is enabled in sim compilation
import quaternion  # noqa: F401

from habitat_sim.utils.collect_env import main as collect_env


def test_collect_env():
    collect_env()
