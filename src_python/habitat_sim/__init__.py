#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import builtins

__version__ = "0.2.2"

if not getattr(builtins, "__HSIM_SETUP__", False):
    # TODO: kept only for compatibiliy with existing code. Please gradually remove
    import habitat_sim._ext.habitat_sim_bindings
    from habitat_sim import (
        agent,
        attributes,
        attributes_managers,
        geo,
        gfx,
        logging,
        metadata,
        nav,
        physics,
        robots,
        scene,
        sensor,
        sensors,
        sim,
        simulator,
        utils,
    )
    from habitat_sim._ext.habitat_sim_bindings import MapStringString

    try:
        from habitat_sim._ext.habitat_sim_bindings import VHACDParameters  # noqa: F401
    except Exception:
        pass

    # if getattr()
    from habitat_sim.agent.agent import (  # noqa: F401
        ActionSpec,
        Agent,
        AgentConfiguration,
        AgentState,
        SixDOFPose,
    )
    from habitat_sim.agent.controls import (  # noqa: F401
        ActuationSpec,
        ObjectControls,
        PyRobotNoisyActuationSpec,
        SceneNodeControl,
        controls,
        default_controls,
        object_controls,
        pyrobot_noisy_controls,
    )
    from habitat_sim.bindings import (  # noqa: F401
        RigidState,
        SceneGraph,
        SceneNode,
        SceneNodeType,
        SimulatorConfiguration,
        audio_enabled,
        built_with_bullet,
        cuda_enabled,
        vhacd_enabled,
    )
    from habitat_sim.nav import (  # noqa: F401
        GreedyFollowerCodes,
        GreedyGeodesicFollower,
        HitRecord,
        MultiGoalShortestPath,
        NavMeshSettings,
        PathFinder,
        ShortestPath,
        VectorGreedyCodes,
    )
    from habitat_sim.registry import registry
    from habitat_sim.sensor import (
        AudioSensor,
        AudioSensorSpec,
        CameraSensorSpec,
        EquirectangularSensor,
        EquirectangularSensorSpec,
        FisheyeSensorDoubleSphereSpec,
        FisheyeSensorModelType,
        FisheyeSensorSpec,
        RLRAudioPropagationChannelLayout,
        RLRAudioPropagationConfiguration,
        Sensor,
        SensorFactory,
        SensorSpec,
        SensorSubType,
        SensorType,
        VisualSensorSpec,
    )
    from habitat_sim.simulator import Configuration, Simulator  # noqa: F401

    __all__ = [
        "agent",
        "attributes",
        "attributes_managers",
        "metadata",
        "nav",
        "sensors",
        "errors",
        "geo",
        "gfx",
        "logging",
        "nav",
        "physics",
        "scene",
        "sensor",
        "sim",
        "simulator",
        "utils",
        "MapStringString",
        "registry",
        "robots",
    ]
