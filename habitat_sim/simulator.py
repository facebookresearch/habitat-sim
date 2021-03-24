#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import time
from collections import OrderedDict
from collections.abc import MutableMapping
from os import path as osp
from typing import Any, Dict, List
from typing import MutableMapping as MutableMapping_T
from typing import Optional, Union, cast, overload

import attr
import magnum as mn
import numpy as np
from magnum import Vector3
from numpy import ndarray

try:
    import torch
    from torch import Tensor

    _HAS_TORCH = True
except ImportError:
    _HAS_TORCH = False


from habitat_sim.agent.agent import Agent, AgentConfiguration, AgentState
from habitat_sim.logging import logger
from habitat_sim.metadata import MetadataMediator
from habitat_sim.nav import GreedyGeodesicFollower, NavMeshSettings, PathFinder
from habitat_sim.sensor import Sensor, SensorSpec, SensorType
from habitat_sim.sensors.noise_models import SensorNoiseModel, make_sensor_noise_model
from habitat_sim.sim import SimulatorBackend, SimulatorConfiguration
from habitat_sim.utils.common import quat_from_angle_axis

# TODO maybe clean up types with TypeVars


@attr.s(auto_attribs=True, slots=True)
class Configuration(object):
    r"""Specifies how to configure the simulator.

    :property sim_cfg: The configuration of the backend of the simulator
    :property agents: A list of agent configurations
    :property metadata_mediator: (optional) The metadata mediator to build the simulator from
    Ties together a backend config, `sim_cfg` and a list of agent
    configurations `agents`.
    """

    sim_cfg: SimulatorConfiguration
    agents: List[AgentConfiguration]
    # An existing Metadata Mediator can also be used to construct a SimulatorBackend
    metadata_mediator: Optional[MetadataMediator] = None


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
    _default_agent_id: int = attr.ib(default=0, init=False)
    __noise_models: Dict[str, SensorNoiseModel] = attr.ib(factory=dict, init=False)
    _initialized: bool = attr.ib(default=False, init=False)
    _previous_step_time: float = attr.ib(
        default=0.0, init=False
    )  # track the compute time of each step
    __last_state: Dict[int, AgentState] = attr.ib(factory=dict, init=False)

    @staticmethod
    def _sanitize_config(config: Configuration) -> None:
        if len(config.agents) == 0:
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
        self.__noise_models.clear()

        for agent in self.agents:
            agent.close()
            del agent

        self.agents = []

        self.__last_state.clear()

        super().close()

    def __enter__(self) -> "Simulator":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def seed(self, new_seed: int) -> None:
        super().seed(new_seed)
        self.pathfinder.seed(new_seed)

    @overload
    def reset(
        self, agent_ids: List[int]
    ) -> Dict[int, Dict[str, Union[ndarray, "Tensor"]]]:
        ...

    @overload
    def reset(
        self, agent_ids: Optional[int] = None
    ) -> Dict[str, Union[ndarray, "Tensor"]]:
        ...

    def reset(
        self, agent_ids: Union[Optional[int], List[int]] = None
    ) -> Union[
        Dict[str, Union[ndarray, "Tensor"]],
        Dict[int, Dict[str, Union[ndarray, "Tensor"]]],
    ]:
        super().reset()
        for i in range(len(self.agents)):
            self.reset_agent(i)

        if agent_ids is None:
            agent_ids = [self._default_agent_id]
            return_single = True
        else:
            agent_ids = cast(List[int], agent_ids)
            return_single = False
        obs = self.get_sensor_observations(agent_ids=agent_ids)
        if return_single:
            return obs[agent_ids[0]]
        return obs

    def reset_agent(self, agent_id: int) -> None:
        agent = self.get_agent(agent_id)
        initial_agent_state = agent.initial_state
        if initial_agent_state is None:
            raise RuntimeError("reset called before agent was given an initial state")

        self.initialize_agent(agent_id, initial_agent_state)

    def _config_backend(self, config: Configuration) -> None:
        if not self._initialized:
            super().__init__(config.sim_cfg, config.metadata_mediator)
            self._initialized = True
        else:
            super().reconfigure(config.sim_cfg)

    def _config_agents(self, config: Configuration) -> None:
        self.agents = [
            Agent(self.get_active_scene_graph().get_root_node().create_child(), cfg)
            for cfg in config.agents
        ]

    def _config_pathfinder(self, config: Configuration) -> None:
        scene_basename = osp.basename(config.sim_cfg.scene_id)
        # "mesh.ply" is identified as a replica model, whose navmesh
        # is named as "mesh_semantic.navmesh" and is placed in the
        # subfolder called "habitat" (a level deeper than the "mesh.ply")
        if scene_basename == "mesh.ply":
            scene_dir = osp.dirname(config.sim_cfg.scene_id)
            navmesh_filenname = osp.join(scene_dir, "habitat", "mesh_semantic.navmesh")
        else:
            navmesh_filenname = osp.splitext(config.sim_cfg.scene_id)[0] + ".navmesh"

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

    def __set_from_config(self, config: Configuration) -> None:
        self._config_backend(config)
        self._config_agents(config)
        self._config_pathfinder(config)
        self.frustum_culling = config.sim_cfg.frustum_culling

        for i in range(len(self.agents)):
            self.agents[i].controls.move_filter_fn = self.step_filter

        self._default_agent_id = config.sim_cfg.default_agent_id

        self.__noise_models: Dict[str, SensorNoiseModel] = {}
        self.__last_state = dict()
        for agent_id, _ in enumerate(config.agents):
            self.initialize_agent(agent_id)

    def make_noise_model(self, sensor_spec: SensorSpec):
        noise_model_kwargs = sensor_spec.noise_model_kwargs
        gpu_device = 0
        if sensor_spec.gpu2gpu_transfer:
            gpu_device = self.gpu_device
        noise_model = make_sensor_noise_model(
            sensor_spec.noise_model, {"gpu_device_id": gpu_device, **noise_model_kwargs}
        )
        assert noise_model.is_valid_sensor_type(
            sensor_spec.sensor_type
        ), "Noise model '{}' is not valid for sensor '{}'".format(
            sensor_spec.noise_model, sensor_spec.uuid
        )
        return noise_model

    def get_noise_model(self, sensor_spec: SensorSpec):
        if sensor_spec.noise_model not in self.__noise_models:
            self.__noise_models[sensor_spec.noise_model] = self.make_noise_model(
                sensor_spec
            )
        return self.__noise_models[sensor_spec.noise_model]

    def add_sensor(
        self, sensor_spec: SensorSpec, agent_id: Optional[int] = None
    ) -> None:
        if (
            (
                not self.config.sim_cfg.load_semantic_mesh
                and sensor_spec.sensor_type == SensorType.SEMANTIC
            )
            or (
                not self.config.sim_cfg.requires_textures
                and sensor_spec.sensor_type == SensorType.COLOR
            )
            or (
                not self.config.sim_cfg.create_renderer
                and sensor_spec.sensor_type == SensorType.DEPTH
            )
        ):
            sensor_type = sensor_spec.sensor_type
            raise ValueError(
                f"""Data for {sensor_type} sensor was not loaded during Simulator init.
                    Cannot dynamically add a {sensor_type} sensor unless one already exists.
                    """
            )
        if agent_id is None:
            agent_id = self._default_agent_id
        agent = self.get_agent(agent_id=agent_id)
        agent._add_sensor(sensor_spec)

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
        self.__last_state[agent_id] = agent.state
        return agent

    @overload
    def get_sensor_observations(
        self, agent_ids: int = 0
    ) -> Dict[str, Union[ndarray, "Tensor"]]:
        ...

    @overload
    def get_sensor_observations(
        self, agent_ids: List[int]
    ) -> Dict[int, Dict[str, Union[ndarray, "Tensor"]]]:
        ...

    def get_sensor_observations(
        self, agent_ids: Union[int, List[int]] = 0
    ) -> Union[
        Dict[str, Union[ndarray, "Tensor"]],
        Dict[int, Dict[str, Union[ndarray, "Tensor"]]],
    ]:
        if isinstance(agent_ids, int):
            agent_ids = [agent_ids]
            return_single = True
        else:
            return_single = False

        for agent_id in agent_ids:
            agent_sensors = self.get_agent(agent_id).scene_node.subtree_sensors
            for _, sensor in agent_sensors.items():
                sensor.draw_observation(self)

        # As backport. All Dicts are ordered in Python >= 3.7
        observations: Dict[int, Dict[str, Union[ndarray, "Tensor"]]] = OrderedDict()
        for agent_id in agent_ids:
            agent_sensors = self.get_agent(agent_id).scene_node.subtree_sensors
            agent_observations: Dict[str, Union[ndarray, "Tensor"]] = {}
            for sensor_uuid, sensor in agent_sensors.items():
                agent_observations[sensor_uuid] = self.get_observation(sensor)
            observations[agent_id] = agent_observations
        if return_single:
            return next(iter(observations.values()))
        return observations

    @property
    def _default_agent(self) -> Agent:
        # TODO Deprecate and remove
        return self.get_agent(agent_id=self._default_agent_id)

    @property
    def _last_state(self) -> AgentState:
        # TODO Deprecate and remove
        return self.__last_state[self._default_agent_id]

    @_last_state.setter
    def _last_state(self, state: AgentState) -> None:
        # TODO Deprecate and remove
        self.__last_state[self._default_agent_id] = state

    def last_state(self, agent_id: Optional[int] = None) -> AgentState:
        if agent_id is None:
            agent_id = self._default_agent_id
        return self.__last_state[agent_id]

    @overload
    def step(
        self, action: Union[str, int], dt: float = 1.0 / 60.0
    ) -> Dict[str, Union[bool, ndarray, "Tensor"]]:
        ...

    @overload
    def step(
        self, action: MutableMapping_T[int, Union[str, int]], dt: float = 1.0 / 60.0
    ) -> Dict[int, Dict[str, Union[bool, ndarray, "Tensor"]]]:
        ...

    def step(
        self,
        action: Union[str, int, MutableMapping_T[int, Union[str, int]]],
        dt: float = 1.0 / 60.0,
    ) -> Union[
        Dict[str, Union[bool, ndarray, "Tensor"]],
        Dict[int, Dict[str, Union[bool, ndarray, "Tensor"]]],
    ]:
        self._num_total_frames += 1
        if isinstance(action, MutableMapping):
            return_single = False
        else:
            action = cast(Dict[int, Union[str, int]], {self._default_agent_id: action})
            return_single = True
        collided_dict: Dict[int, bool] = {}
        for agent_id, agent_act in action.items():
            agent = self.get_agent(agent_id)
            collided_dict[agent_id] = agent.act(agent_act)
            self.__last_state[agent_id] = agent.get_state()

        # step physics by dt
        step_start_Time = time.time()
        super().step_world(dt)
        self._previous_step_time = time.time() - step_start_Time

        multi_observations = self.get_sensor_observations(agent_ids=list(action.keys()))
        for agent_id, agent_observation in multi_observations.items():
            agent_observation["collided"] = collided_dict[agent_id]
        if return_single:
            return multi_observations[self._default_agent_id]
        return multi_observations

    def make_greedy_follower(
        self,
        agent_id: Optional[int] = None,
        goal_radius: float = None,
        *,
        stop_key: Optional[Any] = None,
        forward_key: Optional[Any] = None,
        left_key: Optional[Any] = None,
        right_key: Optional[Any] = None,
        fix_thrashing: bool = True,
        thrashing_threshold: int = 16,
    ):
        if agent_id is None:
            agent_id = self._default_agent_id
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

    def get_observation(self, sensor: Sensor) -> Union[ndarray, "Tensor"]:
        self.renderer.bind_render_target(sensor)
        tgt = sensor.render_target
        noise_model = self.get_noise_model(sensor.specification())
        buffer = sensor.buffer(self.gpu_device)

        if sensor.specification().gpu2gpu_transfer:
            with torch.cuda.device(self.gpu_device):  # type: ignore[attr-defined]
                if sensor.specification().sensor_type == SensorType.SEMANTIC:
                    tgt.read_frame_object_id_gpu(buffer.data_ptr())  # type: ignore[attr-defined]
                elif sensor.specification().sensor_type == SensorType.DEPTH:
                    tgt.read_frame_depth_gpu(buffer.data_ptr())  # type: ignore[attr-defined]
                else:
                    tgt.read_frame_rgba_gpu(buffer.data_ptr())  # type: ignore[attr-defined]

                obs = buffer.flip(0)
        else:
            size = sensor.framebuffer_size

            if sensor.specification().sensor_type == SensorType.SEMANTIC:
                tgt.read_frame_object_id(
                    mn.MutableImageView2D(
                        mn.PixelFormat.R32UI,
                        size,
                        np.array(buffer),
                    )
                )
            elif sensor.specification().sensor_type == SensorType.DEPTH:
                tgt.read_frame_depth(
                    mn.MutableImageView2D(
                        mn.PixelFormat.R32F,
                        size,
                        np.array(buffer),
                    )
                )
            else:

                tgt.read_frame_rgba(
                    mn.MutableImageView2D(
                        mn.PixelFormat.RGBA8_UNORM,
                        size,
                        np.array(buffer).reshape(
                            sensor.specification().resolution[0], -1
                        ),
                    )
                )

            obs = np.flip(buffer, axis=0)

        return noise_model(obs)
