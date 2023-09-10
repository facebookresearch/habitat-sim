#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import time
from collections import OrderedDict
from collections.abc import MutableMapping
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
from habitat_sim.bindings import cuda_enabled
from habitat_sim.logging import LoggingContext, logger
from habitat_sim.metadata import MetadataMediator
from habitat_sim.nav import GreedyGeodesicFollower
from habitat_sim.sensor import SensorSpec, SensorType
from habitat_sim.sensors.noise_models import make_sensor_noise_model
from habitat_sim.sim import SimulatorBackend, SimulatorConfiguration
from habitat_sim.utils.common import quat_from_angle_axis

# TODO maybe clean up types with TypeVars
ObservationDict = Dict[str, Union[bool, np.ndarray, "Tensor"]]


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
    enable_batch_renderer: bool = False


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
    __sensors: List[Dict[str, "Sensor"]] = attr.ib(factory=list, init=False)
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

        config.sim_cfg.create_renderer = not config.enable_batch_renderer and any(
            len(cfg.sensor_specifications) > 0 for cfg in config.agents
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
        # NB: Python still calls __del__ (and thus)
        # closes even if __init__ fails. We don't
        # have anything to close if we aren't initialized so
        # we can just return.
        if not self._initialized:
            return

        if self.renderer is not None:
            self.renderer.acquire_gl_context()

        for agent_sensorsuite in self.__sensors:
            for sensor in agent_sensorsuite.values():
                sensor.close()
                del sensor

        self.__sensors = []

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
    ) -> Union[ObservationDict, Dict[int, ObservationDict],]:
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
        self.pathfinder.seed(config.sim_cfg.random_seed)

        if self.pathfinder is None or not self.pathfinder.is_loaded:
            logger.warning(
                "Navmesh not loaded or computed, no collision checking will be done."
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

        self.__sensors: List[Dict[str, Sensor]] = [
            dict() for i in range(len(config.agents))
        ]
        self.__last_state = dict()
        for agent_id, agent_cfg in enumerate(config.agents):
            for spec in agent_cfg.sensor_specifications:
                self._update_simulator_sensors(spec.uuid, agent_id=agent_id)
            self.initialize_agent(agent_id)

    def _update_simulator_sensors(self, uuid: str, agent_id: int) -> None:
        self.__sensors[agent_id][uuid] = Sensor(
            sim=self, agent=self.get_agent(agent_id), sensor_id=uuid
        )

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
        self._update_simulator_sensors(sensor_spec.uuid, agent_id=agent_id)

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

    def start_async_render_and_step_physics(
        self, dt: float, agent_ids: Union[int, List[int]] = 0
    ):
        assert not self.config.enable_batch_renderer

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
            agent_sensorsuite = self.__sensors[agent_id]
            for sensor in agent_sensorsuite.values():
                sensor._draw_observation_async()

        self.renderer.start_draw_jobs()
        self.step_physics(dt)

    def start_async_render(self, agent_ids: Union[int, List[int]] = 0):
        assert not self.config.enable_batch_renderer

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
            agent_sensorsuite = self.__sensors[agent_id]
            for sensor in agent_sensorsuite.values():
                sensor._draw_observation_async()

        self.renderer.start_draw_jobs()

    def get_sensor_observations_async_finish(
        self,
    ) -> Union[
        Dict[str, Union[ndarray, "Tensor"]],
        Dict[int, Dict[str, Union[ndarray, "Tensor"]]],
    ]:
        assert not self.config.enable_batch_renderer

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
        observations: Dict[int, Dict[str, Union[ndarray, "Tensor"]]] = OrderedDict()
        for agent_id in agent_ids:
            agent_observations: Dict[str, Union[ndarray, "Tensor"]] = {}
            for sensor_uuid, sensor in self.__sensors[agent_id].items():
                agent_observations[sensor_uuid] = sensor._get_observation_async()

            observations[agent_id] = agent_observations
        if return_single:
            return next(iter(observations.values()))
        return observations

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
    ) -> Union[ObservationDict, Dict[int, ObservationDict],]:
        if isinstance(agent_ids, int):
            agent_ids = [agent_ids]
            return_single = True
        else:
            return_single = False

        # As backport. All Dicts are ordered in Python >= 3.7.
        observations: Dict[int, ObservationDict] = OrderedDict()

        # Draw observations (for classic non-batched renderer).
        if not self.config.enable_batch_renderer:
            for agent_id in agent_ids:
                agent_sensorsuite = self.__sensors[agent_id]
                for _sensor_uuid, sensor in agent_sensorsuite.items():
                    sensor.draw_observation()
        else:
            # The batch renderer draws observations from external code.
            # Sensors are only used as data containers.
            pass

        # Get observations.
        for agent_id in agent_ids:
            agent_observations: ObservationDict = {}
            for sensor_uuid, sensor in self.__sensors[agent_id].items():
                agent_observations[sensor_uuid] = sensor.get_observation()
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

    @property
    def _sensors(self) -> Dict[str, "Sensor"]:
        # TODO Deprecate and remove
        return self.__sensors[self._default_agent_id]

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
        goal_radius: Optional[float] = None,
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

    def step_physics(self, dt: float) -> None:
        self.step_world(dt)


class Sensor:
    r"""Wrapper around habitat_sim.Sensor

    TODO(MS) define entire Sensor class in python, reducing complexity
    """
    buffer = Union[np.ndarray, "Tensor"]

    def __init__(self, sim: Simulator, agent: Agent, sensor_id: str) -> None:
        self._sim = sim
        self._agent = agent

        # sensor is an attached object to the scene node
        # store such "attached object" in _sensor_object
        self._sensor_object = self._agent._sensors[sensor_id]

        self._spec = self._sensor_object.specification()

        # When using the batch renderer, no memory is allocated here.
        if not self._sim.config.enable_batch_renderer:
            self._initialize_sensor()

    def _initialize_sensor(self):
        r"""
        Allocate buffers and initialize noise model in preparation for rendering.
        """
        if self._spec.sensor_type == SensorType.AUDIO:
            return

        if self._sim.renderer is not None:
            self._sim.renderer.bind_render_target(self._sensor_object)

        if self._spec.gpu2gpu_transfer:
            assert cuda_enabled, "Must build habitat sim with cuda for gpu2gpu-transfer"
            assert _HAS_TORCH
            device = torch.device("cuda", self._sim.gpu_device)  # type: ignore[attr-defined]
            torch.cuda.set_device(device)

            resolution = self._spec.resolution
            if self._spec.sensor_type == SensorType.SEMANTIC:
                self._buffer: Union[np.ndarray, "Tensor"] = torch.empty(
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
            size = self._sensor_object.framebuffer_size
            if self._spec.sensor_type == SensorType.SEMANTIC:
                self._buffer = np.empty(
                    (self._spec.resolution[0], self._spec.resolution[1]),
                    dtype=np.uint32,
                )
                self.view = mn.MutableImageView2D(
                    mn.PixelFormat.R32UI, size, self._buffer
                )
            elif self._spec.sensor_type == SensorType.DEPTH:
                self._buffer = np.empty(
                    (self._spec.resolution[0], self._spec.resolution[1]),
                    dtype=np.float32,
                )
                self.view = mn.MutableImageView2D(
                    mn.PixelFormat.R32F, size, self._buffer
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
                self.view = mn.MutableImageView2D(
                    mn.PixelFormat.RGBA8_UNORM,
                    size,
                    self._buffer.reshape(self._spec.resolution[0], -1),
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
        # Batch rendering happens elsewhere.
        assert not self._sim.config.enable_batch_renderer

        if self._spec.sensor_type == SensorType.AUDIO:
            # do nothing in draw observation, get_observation will be called after this
            # run the simulation there
            return

        assert self._sim.renderer is not None
        # see if the sensor is attached to a scene graph, otherwise it is invalid,
        # and cannot make any observation
        if not self._sensor_object.object:
            raise habitat_sim.errors.InvalidAttachedObject(
                "Sensor observation requested but sensor is invalid.\
                    (has it been detached from a scene node?)"
            )
        self._sim.renderer.draw(self._sensor_object, self._sim)

    def _draw_observation_async(self) -> None:
        # Batch rendering happens elsewhere.
        assert not self._sim.config.enable_batch_renderer

        if self._spec.sensor_type == SensorType.AUDIO:
            # do nothing in draw observation, get_observation will be called after this
            # run the simulation there
            return

        assert self._sim.renderer is not None
        if (
            self._spec.sensor_type == SensorType.SEMANTIC
            and self._sim.get_active_scene_graph()
            is not self._sim.get_active_semantic_scene_graph()
        ):
            raise RuntimeError(
                "Async drawing doesn't support semantic rendering when there are multiple scene graphs"
            )
        # TODO: sync this path with renderer changes as above (render from sensor object)

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

        # get the correct scene graph based on application
        if self._spec.sensor_type == SensorType.SEMANTIC:
            scene = self._sim.get_active_semantic_scene_graph()
        else:  # SensorType is DEPTH or any other type
            scene = self._sim.get_active_scene_graph()

        render_flags = habitat_sim.gfx.Camera.Flags.NONE

        if self._sim.frustum_culling:
            render_flags |= habitat_sim.gfx.Camera.Flags.FRUSTUM_CULLING

        self._sim.renderer.enqueue_async_draw_job(
            self._sensor_object, scene, self.view, render_flags
        )

    def get_observation(self) -> Union[ndarray, "Tensor"]:
        if self._spec.sensor_type == SensorType.AUDIO:
            return self._get_audio_observation()

        # Placeholder until batch renderer emplaces the final value.
        if self._sim.config.enable_batch_renderer:
            return None

        assert self._sim.renderer is not None
        tgt = self._sensor_object.render_target

        if self._spec.gpu2gpu_transfer:
            with torch.cuda.device(self._buffer.device):  # type: ignore[attr-defined, union-attr]
                if self._spec.sensor_type == SensorType.SEMANTIC:
                    tgt.read_frame_object_id_gpu(self._buffer.data_ptr())  # type: ignore[attr-defined, union-attr]
                elif self._spec.sensor_type == SensorType.DEPTH:
                    tgt.read_frame_depth_gpu(self._buffer.data_ptr())  # type: ignore[attr-defined, union-attr]
                else:
                    tgt.read_frame_rgba_gpu(self._buffer.data_ptr())  # type: ignore[attr-defined, union-attr]

                obs = self._buffer.flip(0)  # type: ignore[union-attr]
        else:
            if self._spec.sensor_type == SensorType.SEMANTIC:
                tgt.read_frame_object_id(self.view)
            elif self._spec.sensor_type == SensorType.DEPTH:
                tgt.read_frame_depth(self.view)
            else:
                tgt.read_frame_rgba(self.view)

            obs = np.flip(self._buffer, axis=0)

        return self._noise_model(obs)

    def _get_observation_async(self) -> Union[ndarray, "Tensor"]:
        if self._spec.sensor_type == SensorType.AUDIO:
            return self._get_audio_observation()
        if self._spec.gpu2gpu_transfer:
            obs = self._buffer.flip(0)  # type: ignore[union-attr]
        else:
            obs = np.flip(self._buffer, axis=0)

        return self._noise_model(obs)

    def _get_audio_observation(self) -> Union[ndarray, "Tensor"]:
        assert self._spec.sensor_type == SensorType.AUDIO
        audio_sensor = self._agent._sensors["audio_sensor"]
        # tell the audio sensor about the agent location
        rot = self._agent.state.rotation

        audio_sensor.setAudioListenerTransform(
            audio_sensor.node.absolute_translation,  # set the listener position
            np.array([rot.w, rot.x, rot.y, rot.z]),  # set the listener orientation
        )

        # run the simulation
        audio_sensor.runSimulation(self._sim)
        obs = audio_sensor.getIR()
        return obs

    def close(self) -> None:
        self._sim = None
        self._agent = None
        self._sensor_object = None
