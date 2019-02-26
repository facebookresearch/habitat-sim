#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from habitat_sim.bindings import *
import numpy as np


class Simulator:
    def __init__(self, config):
        super().__init__()
        self._viewer = None
        self._num_total_frames = 0
        self._sim = None
        self.reconfigure(config)

    def close(self):
        self._sensors = None
        if self._sim is not None:
            del self._sim

    def seed(self, new_seed):
        self._sim.seed(new_seed)

    def reset(self):
        self._sim.reset()
        return self.get_sensor_observations()

    def reconfigure(self, config):
        if self._sim is None:
            self._sim = SimulatorBackend(config)
        else:
            self._sim.reconfigure(config)
        self._config = config
        self._default_agent = self.get_agent(agent_id=config.default_agent_id)
        agent_cfg = config.agents[config.default_agent_id]
        self._sensors = {}
        for spec in agent_cfg.sensor_specifications:
            self._sensors[spec.uuid] = Sensor(
                sim=self._sim, agent_id=config.default_agent_id, sensor_id=spec.uuid
            )
        self._last_state = AgentState()

    def get_agent(self, agent_id):
        return self._sim.agent(agent_id=agent_id)

    def initialize_agent(self, agent_id, initial_state=None):
        agent = self.get_agent(agent_id=agent_id)
        if initial_state is None:
            initial_state = AgentState()
            self._sim.sample_random_agent_state(initial_state)
        agent.set_state(initial_state)
        agent.get_state(self._last_state)
        return agent

    def sample_random_agent_state(self, state_to_return):
        return self._sim.sample_random_agent_state(state_to_return)

    @property
    def pathfinder(self):
        return self._sim.pathfinder

    @property
    def semantic_scene(self):
        return self._sim.semantic_scene

    def get_sensor_observations(self):
        observations = {}
        for sensor_uuid, sensor in self._sensors.items():
            observations[sensor_uuid] = sensor.get_observation()
        return observations

    def last_state(self):
        return self._last_state

    def step(self, action):
        self._num_total_frames += 1
        self._default_agent.act(action)
        self._default_agent.get_state(self._last_state)
        observations = self.get_sensor_observations()
        return observations

    def make_action_pathfinder(self, agent_id=0):
        return self._sim.make_action_pathfinder(agent_id)


class Sensor:
    r"""Wrapper around habitat_sim.Sensor

    TODO(MS) define entire Sensor class in python, reducing complexity
    """

    def __init__(self, sim, agent_id, sensor_id):
        self._sim = sim
        self._agent = sim.agent(agent_id=agent_id)

        # sensor is an attached object to the scene node
        # store such "attached object" in _sensor_object
        self._sensor_object = self._agent.sensors.get(sensor_id)

        self._spec = self._sensor_object.specification()
        if self._spec.sensor_type == SensorType.SEMANTIC:
            self._buffer = np.empty(
                (self._spec.resolution[0], self._spec.resolution[1]), dtype=np.uint32
            )
        elif self._spec.sensor_type == SensorType.DEPTH:
            self._buffer = np.empty(
                (self._spec.resolution[0], self._spec.resolution[1]), dtype=np.float32
            )
        else:
            self._buffer = np.empty(
                (
                    self._spec.resolution[0],
                    self._spec.resolution[1] * self._spec.channels,
                ),
                dtype=np.uint8,
            )

    def get_observation(self):
        # sanity check:
        # see if the sensor is attached to a scene graph, otherwise it is invalid,
        # and cannot make any observation
        if not self._sensor_object.is_valid:
            raise RuntimeError(
                "Sensor observation requested but sensor is invalid.\
                 (has it been detached from a scene node?)"
            )

        # get the correct scene graph based on application
        if self._spec.sensor_type == SensorType.SEMANTIC:
            if self._sim.semantic_scene is None:
                raise RuntimeError(
                    "SemanticSensor observation requested but no SemanticScene is loaded"
                )
            scene = self._sim.get_active_semantic_scene_graph()
        else:  # SensorType is DEPTH or any other type
            scene = self._sim.get_active_scene_graph()

        # now, connect the agent to the root node of the current scene graph

        # sanity check is not needed on agent:
        # because if a sensor is attached to a scene graph,
        # it implies the agent is attached to the same scene graph
        # (it assumes backend simulator will guarantee it.)

        agent_node = self._agent.get_scene_node()
        agent_node.set_parent(scene.get_root_node())

        # draw the scene with the visual sensor:
        # it asserts the sensor is a visual sensor;
        # internally it will set the camera parameters (from the sensor) to the
        # default render camera in the scene so that
        # it has correct modelview matrix, projection matrix to render the scene
        self._sim.renderer.draw(self._sensor_object, scene)

        if self._spec.sensor_type == SensorType.SEMANTIC:
            self._sim.renderer.readFrameObjectId(self._buffer)
            return np.flip(self._buffer, axis=0).copy()
        elif self._spec.sensor_type == SensorType.DEPTH:
            self._sim.renderer.readFrameDepth(self._buffer)
            return np.flip(self._buffer, axis=0).copy()
        else:
            self._sim.renderer.readFrameRgba(self._buffer)
            return np.flip(
                self._buffer.reshape(
                    (
                        self._spec.resolution[0],
                        self._spec.resolution[1],
                        self._spec.channels,
                    )
                ),
                axis=0,
            ).copy()
