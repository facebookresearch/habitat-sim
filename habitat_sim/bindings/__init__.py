#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

modules = [
    "AttachedObject",
    "AttachedObjectType",
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
    "geo",
]

from habitat_sim._ext.habitat_sim_bindings import Simulator as SimulatorBackend

exec(
    "from habitat_sim._ext.habitat_sim_bindings import ({})".format(", ".join(modules))
)

__all__ = ["SimulatorBackend"] + modules
