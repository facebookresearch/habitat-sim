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

import habitat_sim.errors
from habitat_sim.agent.agent import Agent, AgentConfiguration, AgentState
from habitat_sim.logging import LoggingContext, logger
from habitat_sim.metadata import MetadataMediator
from habitat_sim.nav import GreedyGeodesicFollower, NavMeshSettings
from habitat_sim.sensor import Sensor, SensorSpec, SensorType
from habitat_sim.sensors.noise_models import SensorNoiseModel, make_sensor_noise_model
from habitat_sim.sim import SimulatorBackend, SimulatorConfiguration
from habitat_sim.utils.common import quat_from_angle_axis

# A sensor observation
SensorObservation = Union[bool, ndarray, "Tensor"]
# A dictionary of sensor observations
ObservationDict = Dict[str, SensorObservation]


@attr.s(auto_attribs=True, slots=True)
class Configuration:
    r"""Specifies how to configure the simulator.

    :property sim_cfg: The configuration of the backend of the simulator
    :property agents: A list of agent configurations
    :property metadata_mediator: (optional) The metadata mediator to build the simulator from.

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
    _async_draw_agent_ids: Optional[Union[int, List[int]]] = None
    __last_state: Dict[int, AgentState] = attr.ib(factory=dict, init=False)

    @staticmethod
    def _sanitize_config(config: Configuration) -> None:
        if len(config.agents) == 0:
            raise RuntimeError(
                "Config has not agents specified.  Must specify at least 1 agent"
            )

        config.sim_cfg.create_renderer = any(
            (len(cfg.sensor_specifications) > 0 for cfg in config.agents)
        )
        config.sim_cfg.load_semantic_mesh |= any(
            (
                any(
                    sens_spec.sensor_type == SensorType.SEMANTIC
                    for sens_spec in cfg.sensor_specifications
                )
                for cfg in config.agents
            )
        )

        config.sim_cfg.requires_textures = any(
            (
                any(
                    sens_spec.sensor_type == SensorType.COLOR
                    for sens_spec in cfg.sensor_specifications
                )
                for cfg in config.agents
            )
        )

    def __attrs_post_init__(self) -> None:
        LoggingContext.reinitialize_from_env()
        self._sanitize_config(self.config)
        self.__set_from_config(self.config)

    def close(self, destroy: bool = True) -> None:
        r"""Close the simulator instance.

        :param destroy: Whether or not to force the OpenGL context to be
            destroyed if async rendering was used.  If async rendering wasn't used,
            this has no effect.
        """
        # NB: Python still still call __del__ (and thus)
        # close even if __init__ errors. We don't
        # have anything to close if we aren't initialized so
        # we can just return.
        if not self._initialized:
            return

        if self.renderer is not None:
            self.renderer.acquire_gl_context()

        self.__noise_models.clear()

        for agent in self.agents:
            agent.close()
            del agent

        self.agents = []

        self.__last_state.clear()

        super().close(destroy)

    def __enter__(self) -> "Simulator":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close(destroy=True)

    def seed(self, new_seed: int) -> None:
        super().seed(new_seed)
        self.pathfinder.seed(new_seed)

    @overload
    def reset(self, agent_ids: List[int]) -> Dict[int, ObservationDict]:
        ...

    @overload
    def reset(self, agent_ids: Optional[int] = None) -> ObservationDict:
        ...

    def reset(
        self, agent_ids: Union[Optional[int], List[int]] = None
    ) -> Union[ObservationDict, Dict[int, ObservationDict]]:
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

        if osp.exists(navmesh_filenname) and not self.pathfinder.is_loaded:
            self.pathfinder.load_nav_mesh(navmesh_filenname)
            logger.info(f"Loaded navmesh {navmesh_filenname}")

        # NOTE: this recomputed NavMesh does not include STATIC objects.
        needed_settings = NavMeshSettings()
        default_agent_config = config.agents[config.sim_cfg.default_agent_id]
        needed_settings.agent_radius = default_agent_config.radius
        needed_settings.agent_height = default_agent_config.height
        if (
            # If we loaded a navmesh and we need one with different settings,
            # always try and recompute
            (
                self.pathfinder.is_loaded
                and self.pathfinder.nav_mesh_settings != needed_settings
            )
            # If we didn't load a navmesh, only try to recompute one if we can.
            # This allows for use cases where we just want to view a single
            # object or similar.
            or (
                not self.pathfinder.is_loaded
                and config.sim_cfg.scene_id.lower() != "none"
                and config.sim_cfg.create_renderer
            )
        ):
            logger.info(
                f"Recomputing navmesh for agent's height {default_agent_config.height} and radius"
                f" {default_agent_config.radius}."
            )
            self.recompute_navmesh(self.pathfinder, needed_settings)

        self.pathfinder.seed(config.sim_cfg.random_seed)

        if not self.pathfinder.is_loaded:
            logger.warning(
                "Could not find a navmesh nor could one be computed, "
                "no collision checking will be done"
            )

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
        if sensor_spec.uuid not in self.__noise_models:
            self.__noise_models[sensor_spec.uuid] = self.make_noise_model(sensor_spec)
        return self.__noise_models[sensor_spec.uuid]

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

    def start_async_render(self, agent_ids: Union[int, List[int]] = 0):
        if self._async_draw_agent_ids is not None:
            raise RuntimeError(
                "start_async_render_and_step_physics was already called.  "
                "Call get_sensor_observations_async_finish before calling this again.  "
                "Use step_physics to step physics additional times."
            )

        self._async_draw_agent_ids = agent_ids
        if isinstance(agent_ids, int):
            agent_ids = [agent_ids]

        for agent_id in agent_ids:
            agent_sensorsuite = self.get_agent(agent_id).scene_node.node_sensor_suite
            for _sensor_uuid, sensor in agent_sensorsuite.items():
                sensor._draw_observation_async()

        self.renderer.start_draw_jobs()

    def start_async_render_and_step_physics(
        self, dt: float, agent_ids: Union[int, List[int]] = 0
    ):
        self.start_async_render(agent_ids)
        self.step_physics(dt)

    def get_sensor_observations_async_finish(
        self,
    ) -> Union[ObservationDict, Dict[int, ObservationDict]]:
        if self._async_draw_agent_ids is None:
            raise RuntimeError(
                "get_sensor_observations_async_finish was called before calling start_async_render_and_step_physics."
            )

        agent_ids = self._async_draw_agent_ids
        self._async_draw_agent_ids = None
        if isinstance(agent_ids, int):
            agent_ids = [agent_ids]
            return_single = True
        else:
            return_single = False

        self.renderer.wait_draw_jobs()
        # As backport. All Dicts are ordered in Python >= 3.7
        all_agents_observations: Dict[int, ObservationDict] = OrderedDict()
        for agent_id in agent_ids:
            agent_observations: ObservationDict = {}
            for sensor_uuid, sensor in self.get_agent(
                agent_id
            ).scene_node.node_sensor_suite:
                agent_observations[sensor_uuid] = self._get_observation_async(
                    sensor, agent_id
                )

            all_agents_observations[agent_id] = agent_observations
        if return_single:
            return next(iter(all_agents_observations.values()))
        return all_agents_observations

    @overload
    def get_sensor_observations(self, agent_ids: int = 0) -> ObservationDict:
        ...

    @overload
    def get_sensor_observations(
        self, agent_ids: List[int]
    ) -> Dict[int, ObservationDict]:
        ...

    def get_sensor_observations(
        self, agent_ids: Union[int, List[int]] = 0
    ) -> Union[ObservationDict, Dict[int, ObservationDict]]:
        if isinstance(agent_ids, int):
            agent_ids = [agent_ids]
            return_single = True
        else:
            return_single = False

        # As backport. All Dicts are ordered in Python >= 3.7
        all_agents_observations: Dict[int, ObservationDict] = OrderedDict()
        for agent_id in agent_ids:
            agent_sensors = self.get_agent(agent_id).scene_node.node_sensor_suite
            agent_observations: ObservationDict = {}
            for sensor_uuid, sensor in agent_sensors.items():
                agent_observations[sensor_uuid] = self.get_observation(sensor)
                self.draw_observation(sensor)
            all_agents_observations[agent_id] = agent_observations
        if return_single:
            return next(iter(all_agents_observations.values()))
        return all_agents_observations

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
    def step(self, action: Union[str, int], dt: float = 1.0 / 60.0) -> ObservationDict:
        ...

    @overload
    def step(
        self, action: MutableMapping_T[int, Union[str, int]], dt: float = 1.0 / 60.0
    ) -> Dict[int, ObservationDict]:
        ...

    def step(
        self,
        action: Union[str, int, MutableMapping_T[int, Union[str, int]]],
        dt: float = 1.0 / 60.0,
    ) -> Union[ObservationDict, Dict[int, ObservationDict],]:
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
        self.close(destroy=True)

    def step_physics(self, dt: float, scene_id: int = 0) -> None:
        self.step_world(dt)

    def get_observation(self, sensor: Sensor) -> SensorObservation:
        if sensor.specification().sensor_type == SensorType.AUDIO:
            return self._get_audio_observation(sensor)

        if not sensor.has_render_target():
            self.renderer.bind_render_target(sensor)
        tgt = sensor.render_target
        noise_model = self.get_noise_model(sensor.specification())
        obs_buffer = sensor.buffer(self.gpu_device)

        if sensor.specification().gpu2gpu_transfer:
            with torch.cuda.device(self.gpu_device):  # type: ignore[attr-defined]
                if sensor.specification().sensor_type == SensorType.SEMANTIC:
                    tgt.read_frame_object_id_gpu(obs_buffer.data_ptr())  # type: ignore[attr-defined]
                elif sensor.specification().sensor_type == SensorType.DEPTH:
                    tgt.read_frame_depth_gpu(obs_buffer.data_ptr())  # type: ignore[attr-defined]
                elif sensor.specification().sensor_type == SensorType.COLOR:
                    tgt.read_frame_rgba_gpu(obs_buffer.data_ptr())  # type: ignore[attr-defined]

                obs = obs_buffer.flip(0)
        else:
            size = sensor.framebuffer_size

            if sensor.specification().sensor_type == SensorType.SEMANTIC:
                tgt.read_frame_object_id(
                    mn.MutableImageView2D(
                        mn.PixelFormat.R32UI,
                        size,
                        obs_buffer,
                    )
                )
            elif sensor.specification().sensor_type == SensorType.DEPTH:
                tgt.read_frame_depth(
                    mn.MutableImageView2D(
                        mn.PixelFormat.R32F,
                        size,
                        obs_buffer,
                    )
                )
            elif sensor.specification().sensor_type == SensorType.COLOR:
                tgt.read_frame_rgba(
                    mn.MutableImageView2D(
                        mn.PixelFormat.RGBA8_UNORM,
                        size,
                        obs_buffer.reshape(sensor.specification().resolution[0], -1),
                    )
                )

            obs = np.flip(obs_buffer, axis=0)

        return noise_model(obs)

    def _get_observation_async(
        self, sensor: Sensor, agent_id: Optional[int] = None
    ) -> SensorObservation:
        if self._spec.sensor_type == SensorType.AUDIO:
            return self._get_audio_observation(sensor, agent_id)

        if self._spec.gpu2gpu_transfer:
            obs = self._buffer.flip(0)  # type: ignore[union-attr]
        else:
            obs = np.flip(self._buffer, axis=0)

        return self._noise_model(obs)

    def _get_audio_observation(
        self, sensor: Sensor, agent_id: Optional[int] = None
    ) -> SensorObservation:

        assert sensor.specification().sensor_type == SensorType.AUDIO

        # tell the audio sensor about the agent location (if any)
        agent = self.get_agent(agent_id)
        if agent is not None:
            rot = agent.state.rotation
        else:
            rot = quat_from_angle_axis(0, np.array([1, 0, 0]))

        sensor.setAudioListenerTransform(
            sensor.node.absolute_translation,  # set the listener position
            np.array([rot.w, rot.x, rot.y, rot.z]),  # set the listener orientation
        )

        # run the simulation
        sensor.runSimulation(self)
        obs = sensor.getIR()
        return obs

    def draw_observation(self, sensor: Sensor) -> None:
        if sensor.specification().sensor_type == SensorType.AUDIO:
            # do nothing in draw observation, get_observation will be called after this
            # run the simulation there
            return
        else:
            assert self.renderer is not None
            # see if the sensor is attached to a scene graph, otherwise it is invalid,
            # and cannot make any observation
            if sensor.node is None:
                raise habitat_sim.errors.InvalidAttachedObject(
                    "Sensor observation requested but sensor is invalid.\
                    (has it been detached from a scene node?)"
                )
            self.renderer.draw(sensor, self.get_active_scene_graph())

    def _draw_observation_async(self, sensor: Sensor) -> None:
        if sensor.specification().sensor_type == SensorType.AUDIO:
            # do nothing in draw observation, get_observation will be called after this
            # run the simulation there
            return
        else:
            assert self.renderer is not None
            if (
                sensor.specification().sensor_type == SensorType.SEMANTIC
                and self.get_active_scene_graph()
                is not self.get_active_semantic_scene_graph()
            ):
                raise RuntimeError(
                    "Async drawing doesn't support semantic rendering when there are multiple scene graphs"
                )
            # TODO: sync this path with renderer changes as above (render from sensor object)

            # see if the sensor is attached to a scene graph, otherwise it is invalid,
            # and cannot make any observation
            if sensor.node is None:
                raise habitat_sim.errors.InvalidAttachedObject(
                    "Sensor observation requested but sensor is invalid.\
                    (has it been detached from a scene node?)"
                )

            # get the correct scene graph based on application
            if sensor.specification().sensor_type == SensorType.SEMANTIC:
                if self.semantic_scene is None:
                    raise RuntimeError(
                        "SemanticSensor observation requested but no SemanticScene is loaded"
                    )
                pixel_format = mn.PixelFormat.R32UI
                scene = self.get_active_semantic_scene_graph()
            elif sensor.specification().sensor_type == SensorType.DEPTH:
                pixel_format = mn.PixelFormat.R32F
                scene = self.get_active_scene_graph()
            else:  # assume sensor type is COLOR
                pixel_format = mn.PixelFormat.R32UI
                scene = self.get_active_scene_graph()

            render_flags = habitat_sim.gfx.Camera.Flags.NONE

            if self.frustum_culling:
                render_flags |= habitat_sim.gfx.Camera.Flags.FRUSTUM_CULLING

            view = mn.MutableImageView2D(
                pixel_format,
                sensor.framebuffer_size,
                sensor.buffer(self.gpu_device),
            )

            self.renderer.enqueue_async_draw_job(sensor, scene, view, render_flags)
