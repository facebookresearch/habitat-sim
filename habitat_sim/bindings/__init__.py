#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# TODO: this whole thing needs to get removed, kept just for compatibility
#   with existing code

modules = [
    "cuda_enabled",
    "SceneNodeType",
    "GreedyFollowerCodes",
    "GreedyGeodesicFollowerImpl",
    "MultiGoalShortestPath",
    "PathFinder",
    "PinholeCamera",
    "SceneGraph",
    "SceneNode",
    "Sensor",
    "SensorSpec",
    "SensorType",
    "ShortestPath",
    "SimulatorConfiguration",
    "ConfigurationGroup",
    "RigidState",
]

from habitat_sim._ext.habitat_sim_bindings import (  # noqa: F401 isort:skip
    Simulator as SimulatorBackend,
)

exec(
    "from habitat_sim._ext.habitat_sim_bindings import ({})".format(", ".join(modules))
)

__all__ = ["SimulatorBackend"] + modules
