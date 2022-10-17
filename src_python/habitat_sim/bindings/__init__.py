#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# TODO: this whole thing needs to get removed, kept just for compatibility
#   with existing code

from habitat_sim._ext.habitat_sim_bindings import (
    AudioSensor,
    AudioSensorSpec,
    CameraSensor,
    CameraSensorSpec,
    ConfigStoredType,
    Configuration,
    CubeMapSensorBase,
    CubeMapSensorBaseSpec,
    EquirectangularSensor,
    EquirectangularSensorSpec,
    FisheyeSensor,
    FisheyeSensorDoubleSphereSpec,
    FisheyeSensorModelType,
    FisheyeSensorSpec,
    GreedyFollowerCodes,
    GreedyGeodesicFollowerImpl,
    MultiGoalShortestPath,
    PathFinder,
    RigidState,
    RLRAudioPropagationChannelLayout,
    RLRAudioPropagationChannelLayoutType,
    RLRAudioPropagationConfiguration,
    SceneGraph,
    SceneNode,
    SceneNodeType,
    Sensor,
    SensorFactory,
    SensorSpec,
    SensorSubType,
    SensorType,
    ShortestPath,
)
from habitat_sim._ext.habitat_sim_bindings import Simulator as SimulatorBackend
from habitat_sim._ext.habitat_sim_bindings import (
    SimulatorConfiguration,
    VisualSensorSpec,
    audio_enabled,
    built_with_bullet,
    cuda_enabled,
    vhacd_enabled,
)
