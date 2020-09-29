#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import time
from os import path as osp
from typing import Any, Dict, List, Optional, Union

import attr
import magnum as mn
import numpy as np
from magnum import Vector3
from numpy import ndarray

try:
    import torch
    from torch import Tensor
except ImportError:
    torch = None

import habitat_sim.errors
from habitat_sim.agent.agent import Agent, AgentConfiguration, AgentState
from habitat_sim.bindings import cuda_enabled
from habitat_sim.logging import logger
from habitat_sim.nav import GreedyGeodesicFollower, NavMeshSettings, PathFinder
from habitat_sim.sensor import SensorType
from habitat_sim.sensors.noise_models import make_sensor_noise_model
from habitat_sim.sim import SimulatorBackend, SimulatorConfiguration
from habitat_sim.utils.common import quat_from_angle_axis


@attr.s(auto_attribs=True, slots=True)
class Configuration(object):
    r"""Specifies how to configure the simulator.

    :property sim_cfg: The configuration of the backend of the simulator
    :property agents: A list of agent configurations

    Ties together a backend config, `sim_cfg` and a list of agent
    configurations `agents`.
    """

    sim_cfg: Optional[SimulatorConfiguration] = None
    agents: Optional[List[AgentConfiguration]] = None


@attr.s(auto_attribs=True)
class Simulator(SimulatorBackend):
    r"""The core class of habitat-sim

    :property config: configuration for the simulator

    The simulator ties together the backend, the agent, controls functions,
    NavMesh collision checking/pathfinding, attribute template management,
    object manipulation, and physics simulation.
    """

    config: Configuration
    agents: List[Agent] = attr.ib(factory=list, init=False)
    _num_total_frames: int = attr.ib(default=0, init=False)
    _default_agent: Agent = attr.ib(init=False, default=None)
    _sensors: Dict = attr.ib(factory=dict, init=False)
    _initialized: bool = attr.ib(default=False, init=False)
    _previous_step_time: float = attr.ib(
        default=0.0, init=False
    )  # track the compute time of each step

    @staticmethod
    def _sanitize_config(config: Configuration) -> None:
        if not len(config.agents) > 0:
            raise RuntimeError(
                "Config has not agents specified.  Must specify at least 1 agent"
            )

        config.sim_cfg.create_renderer = any(
            map(lambda cfg: len(cfg.sensor_specifications) > 0, config.agents)
        )
        config.sim_cfg.load_semantic_mesh = any(
            map(
                lambda cfg: any(
                    map(
                        lambda sens_spec: sens_spec.sensor_type == SensorType.SEMANTIC,
                        cfg.sensor_specifications,
                    )
                ),
                config.agents,
            )
        )

        config.sim_cfg.requires_textures = any(
            map(
                lambda cfg: any(
                    map(
                        lambda sens_spec: sens_spec.sensor_type == SensorType.COLOR,
                        cfg.sensor_specifications,
                    )
                ),
                config.agents,
            )
        )

    def __attrs_post_init__(self) -> None:
        self._sanitize_config(self.config)
        self.__set_from_config(self.config)

    def close(self) -> None:
        for sensor in self._sensors.values():
            sensor.close()
            del sensor

        self._sensors = {}

        for agent in self.agents:
            agent.close()
            del agent

        self.agents = []

        del self._default_agent
        self._default_agent = None

        self.config = None

        super().close()

    def __enter__(self) -> "Simulator":
        return self

    def __exit__(self, exc_type: None, exc_val: None, exc_tb: None) -> None:
        self.close()

    def seed(self, new_seed: int) -> None:
        super().seed(new_seed)
        self.pathfinder.seed(new_seed)

    def reset(self) -> Dict[str, ndarray]:
        super().reset()
        for i in range(len(self.agents)):
            self.reset_agent(i)

        return self.get_sensor_observations()

    def reset_agent(self, agent_id: int) -> None:
        agent = self.get_agent(agent_id)
        initial_agent_state = agent.initial_state
        if initial_agent_state is None:
            raise RuntimeError("reset called before agent was given an initial state")

        self.initialize_agent(agent_id, initial_agent_state)

    def _config_backend(self, config: Configuration) -> None:
        if not self._initialized:
            super().__init__(config.sim_cfg)
            self._initialized = True
        else:
            super().reconfigure(config.sim_cfg)

    def _config_agents(self, config: Configuration) -> None:
        self.agents = [
            Agent(self.get_active_scene_graph().get_root_node().create_child(), cfg)
            for cfg in config.agents
        ]

    def _config_pathfinder(self, config: Configuration) -> None:
        if "navmesh" in config.sim_cfg.scene.filepaths:
            navmesh_filenname = config.sim_cfg.scene.filepaths["navmesh"]
        else:
            scene_basename = osp.basename(config.sim_cfg.scene.id)
            # "mesh.ply" is identified as a replica model, whose navmesh
            # is named as "mesh_semantic.navmesh" and is placed in the
            # subfolder called "habitat" (a level deeper than the "mesh.ply")
            if scene_basename == "mesh.ply":
                scene_dir = osp.dirname(config.sim_cfg.scene.id)
                navmesh_filenname = osp.join(
                    scene_dir, "habitat", "mesh_semantic.navmesh"
                )
            else:
                navmesh_filenname = (
                    osp.splitext(config.sim_cfg.scene.id)[0] + ".navmesh"
                )

        self.pathfinder = PathFinder()
        if osp.exists(navmesh_filenname):
            self.pathfinder.load_nav_mesh(navmesh_filenname)
            logger.info(f"Loaded navmesh {navmesh_filenname}")
        else:
            logger.warning(
                f"Could not find navmesh {navmesh_filenname}, no collision checking will be done"
            )

        agent_legacy_config = AgentConfiguration()
        default_agent_config = config.agents[config.sim_cfg.default_agent_id]
        if not np.isclose(
            agent_legacy_config.radius, default_agent_config.radius
        ) or not np.isclose(agent_legacy_config.height, default_agent_config.height):
            logger.info(
                f"Recomputing navmesh for agent's height {default_agent_config.height} and radius"
                f" {default_agent_config.radius}."
            )
            navmesh_settings = NavMeshSettings()
            navmesh_settings.set_defaults()
            navmesh_settings.agent_radius = default_agent_config.radius
            navmesh_settings.agent_height = default_agent_config.height
            self.recompute_navmesh(self.pathfinder, navmesh_settings)

        self.pathfinder.seed(config.sim_cfg.random_seed)

    def reconfigure(self, config: Configuration) -> None:
        self._sanitize_config(config)

        if self.config != config:
            self.__set_from_config(config)
            self.config = config

    def __set_from_config(self, config: Configuration):
        self._config_backend(config)
        self._config_agents(config)
        self._config_pathfinder(config)
        self.frustum_culling = config.sim_cfg.frustum_culling

        for i in range(len(self.agents)):
            self.agents[i].controls.move_filter_fn = self.step_filter

        self._default_agent = self.get_agent(config.sim_cfg.default_agent_id)

        agent_cfg = config.agents[config.sim_cfg.default_agent_id]
        self._sensors = {}
        for spec in agent_cfg.sensor_specifications:
            self._sensors[spec.uuid] = Sensor(
                sim=self, agent=self._default_agent, sensor_id=spec.uuid
            )

        for i in range(len(self.agents)):
            self.initialize_agent(i)

    def get_agent(self, agent_id: int) -> Agent:
        return self.agents[agent_id]

    def initialize_agent(
        self, agent_id: int, initial_state: Optional[AgentState] = None
    ) -> Agent:
        agent = self.get_agent(agent_id=agent_id)
        if initial_state is None:
            initial_state = AgentState()
            if self.pathfinder.is_loaded:
                initial_state.position = self.pathfinder.get_random_navigable_point()
                initial_state.rotation = quat_from_angle_axis(
                    self.random.uniform_float(0, 2.0 * np.pi), np.array([0, 1, 0])
                )

        agent.set_state(initial_state, is_initial=True)
        self._last_state = agent.state
        return agent

    def get_sensor_observations(self) -> Dict[str, Union[ndarray, "Tensor"]]:
        for _, sensor in self._sensors.items():
            sensor.draw_observation()

        observations = {}
        for sensor_uuid, sensor in self._sensors.items():
            observations[sensor_uuid] = sensor.get_observation()

        return observations

    def last_state(self):
        return self._last_state

    def step(
        self, action: str, dt: float = 1.0 / 60.0
    ) -> Dict[str, Union[bool, ndarray, "Tensor"]]:
        self._num_total_frames += 1
        collided = self._default_agent.act(action)
        self._last_state = self._default_agent.get_state()

        # step physics by dt
        step_start_Time = time.time()
        super().step_world(dt)
        self._previous_step_time = time.time() - step_start_Time

        observations = self.get_sensor_observations()
        # Whether or not the action taken resulted in a collision
        observations["collided"] = collided

        return observations

    def make_greedy_follower(
        self,
        agent_id: int = 0,
        goal_radius: float = None,
        *,
        stop_key: Optional[Any] = None,
        forward_key: Optional[Any] = None,
        left_key: Optional[Any] = None,
        right_key: Optional[Any] = None,
        fix_thrashing: bool = True,
        thrashing_threshold: int = 16,
    ):
        return GreedyGeodesicFollower(
            self.pathfinder,
            self.get_agent(agent_id),
            goal_radius,
            stop_key=stop_key,
            forward_key=forward_key,
            left_key=left_key,
            right_key=right_key,
            fix_thrashing=fix_thrashing,
            thrashing_threshold=thrashing_threshold,
        )

    def step_filter(self, start_pos: Vector3, end_pos: Vector3) -> Vector3:
        r"""Computes a valid navigable end point given a target translation on the NavMesh.
        Uses the configured sliding flag.

        :param start_pos: The valid initial position of a translation.
        :param end_pos: The target end position of a translation.
        """
        if self.pathfinder.is_loaded:
            if self.config.sim_cfg.allow_sliding:
                end_pos = self.pathfinder.try_step(start_pos, end_pos)
            else:
                end_pos = self.pathfinder.try_step_no_sliding(start_pos, end_pos)

        return end_pos

    def __del__(self) -> None:
        self.close()

    def step_physics(self, dt: float, scene_id: int = 0) -> None:
        self.step_world(dt)


class Sensor:
    r"""Wrapper around habitat_sim.Sensor

    TODO(MS) define entire Sensor class in python, reducing complexity
    """

    def __init__(self, sim: Simulator, agent: Agent, sensor_id: str) -> None:
        self._sim = sim
        self._agent = agent

        # sensor is an attached object to the scene node
        # store such "attached object" in _sensor_object
        self._sensor_object = self._agent._sensors.get(sensor_id)
        self._spec = self._sensor_object.specification()

        self._sim.renderer.bind_render_target(self._sensor_object)

        if self._spec.gpu2gpu_transfer:
            assert cuda_enabled, "Must build habitat sim with cuda for gpu2gpu-transfer"
            assert torch is not None
            device = torch.device("cuda", self._sim.gpu_device)  # type: ignore[attr-defined]
            torch.cuda.set_device(device)

            resolution = self._spec.resolution
            if self._spec.sensor_type == SensorType.SEMANTIC:
                self._buffer = torch.empty(
                    resolution[0], resolution[1], dtype=torch.int32, device=device
                )
            elif self._spec.sensor_type == SensorType.DEPTH:
                self._buffer = torch.empty(
                    resolution[0], resolution[1], dtype=torch.float32, device=device
                )
            else:
                self._buffer = torch.empty(
                    resolution[0], resolution[1], 4, dtype=torch.uint8, device=device
                )
        else:
            if self._spec.sensor_type == SensorType.SEMANTIC:
                self._buffer = np.empty(
                    (self._spec.resolution[0], self._spec.resolution[1]),
                    dtype=np.uint32,
                )
            elif self._spec.sensor_type == SensorType.DEPTH:
                self._buffer = np.empty(
                    (self._spec.resolution[0], self._spec.resolution[1]),
                    dtype=np.float32,
                )
            else:
                self._buffer = np.empty(
                    (
                        self._spec.resolution[0],
                        self._spec.resolution[1],
                        self._spec.channels,
                    ),
                    dtype=np.uint8,
                )

        noise_model_kwargs = self._spec.noise_model_kwargs
        self._noise_model = make_sensor_noise_model(
            self._spec.noise_model,
            {"gpu_device_id": self._sim.gpu_device, **noise_model_kwargs},
        )
        assert self._noise_model.is_valid_sensor_type(
            self._spec.sensor_type
        ), "Noise model '{}' is not valid for sensor '{}'".format(
            self._spec.noise_model, self._spec.uuid
        )

    def draw_observation(self) -> None:
        # sanity check:

        # see if the sensor is attached to a scene graph, otherwise it is invalid,
        # and cannot make any observation
        if not self._sensor_object.object:
            raise habitat_sim.errors.InvalidAttachedObject(
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

        agent_node = self._agent.scene_node
        agent_node.parent = scene.get_root_node()

        render_flags = habitat_sim.gfx.Camera.Flags.NONE

        if self._sim.frustum_culling:
            render_flags |= habitat_sim.gfx.Camera.Flags.FRUSTUM_CULLING

        with self._sensor_object.render_target:
            self._sim.renderer.draw(self._sensor_object, scene, render_flags)

        # add an OBJECT only 2nd pass on the standard SceneGraph if SEMANTIC sensor with separate semantic SceneGraph
        if (
            self._spec.sensor_type == SensorType.SEMANTIC
            and self._sim.get_active_scene_graph()
            is not self._sim.get_active_semantic_scene_graph()
        ):
            agent_node.parent = self._sim.get_active_scene_graph().get_root_node()
            render_flags |= habitat_sim.gfx.Camera.Flags.OBJECTS_ONLY
            self._sim.renderer.draw(
                self._sensor_object, self._sim.get_active_scene_graph(), render_flags
            )

    def get_observation(self) -> Union[ndarray, "Tensor"]:

        tgt = self._sensor_object.render_target

        if self._spec.gpu2gpu_transfer:
            with torch.cuda.device(self._buffer.device):  # type: ignore[attr-defined]
                if self._spec.sensor_type == SensorType.SEMANTIC:
                    tgt.read_frame_object_id_gpu(self._buffer.data_ptr())  # type: ignore[attr-defined]
                elif self._spec.sensor_type == SensorType.DEPTH:
                    tgt.read_frame_depth_gpu(self._buffer.data_ptr())  # type: ignore[attr-defined]
                else:
                    tgt.read_frame_rgba_gpu(self._buffer.data_ptr())  # type: ignore[attr-defined]

                obs = self._buffer.flip(0)
        else:
            size = self._sensor_object.framebuffer_size

            if self._spec.sensor_type == SensorType.SEMANTIC:
                tgt.read_frame_object_id(
                    mn.MutableImageView2D(mn.PixelFormat.R32UI, size, self._buffer)
                )
            elif self._spec.sensor_type == SensorType.DEPTH:
                tgt.read_frame_depth(
                    mn.MutableImageView2D(mn.PixelFormat.R32F, size, self._buffer)
                )
            else:
                tgt.read_frame_rgba(
                    mn.MutableImageView2D(
                        mn.PixelFormat.RGBA8_UNORM,
                        size,
                        self._buffer.reshape(self._spec.resolution[0], -1),
                    )
                )

            obs = np.flip(self._buffer, axis=0)

        return self._noise_model(obs)

    def close(self) -> None:
        self._sim = None
        self._agent = None
        self._sensor_object = None
