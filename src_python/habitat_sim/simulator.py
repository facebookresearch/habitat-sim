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
import numpy as np
from magnum import Vector3
from numpy import ndarray

try:
    import torch  # noqa: F401
    from torch import Tensor

    _HAS_TORCH = True
except ImportError:
    _HAS_TORCH = False

from habitat_sim import bindings as hsim
from habitat_sim.agent.agent import Agent, AgentConfiguration, AgentState
from habitat_sim.logging import LoggingContext, logger
from habitat_sim.metadata import MetadataMediator
from habitat_sim.nav import GreedyGeodesicFollower
from habitat_sim.sensor import SensorFactory, SensorSpec, SensorType
from habitat_sim.sensors.sensor_wrapper import Sensor
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
    _sensor_registry: Dict[str, Sensor] = attr.ib(factory=dict, init=False)
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
        if not self._initialized:
            return

        if self.renderer is not None:
            self.renderer.acquire_gl_context()

        for sensor in self._sensor_registry.values():
            sensor.close()
        self._sensor_registry.clear()

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
        """
        Reset the simulation state including the state of all physics objects, agents, and the default light setup.
        Sets the world time to 0.0, changes the physical state of all objects back to their initial states.
        Does not invalidate existing ManagedObject wrappers.
        Does not add or remove object instances.
        Only changes motion_type when scene_instance specified a motion type.

        :param agent_ids: An optional list of agent ids for which to return the sensor observations. If none is provide, default agent is used.

        :return: Sensor observations in the reset state.
        """
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

        # Build the flat sensor registry from all agents' sensors.
        self._sensor_registry = {}
        self.__last_state = dict()
        for agent_id, agent_cfg in enumerate(config.agents):
            agent = self.get_agent(agent_id)
            # Agent.reconfigure() already called SensorFactory.create_sensors()
            # during __init__, so the C++ sensors exist on the agent's subtree.
            # We just need to wrap them in Python Sensor objects.
            for spec in agent_cfg.sensor_specifications:
                cpp_sensor = agent.scene_node.subtree_sensors[spec.uuid]
                self._sensor_registry[spec.uuid] = Sensor(
                    sim=self, sensor_object=cpp_sensor
                )
            self.initialize_agent(agent_id)

    # ------------------------------------------------------------------
    # Sensor lifecycle — public API
    # ------------------------------------------------------------------

    @property
    def sensors(self) -> Dict[str, Sensor]:
        """All sensors registered with this simulator."""
        return self._sensor_registry

    def create_sensor(
        self,
        spec: SensorSpec,
        scene_node: "hsim.SceneNode",
    ) -> Sensor:
        """Create a sensor on the given SceneNode and register it.

        Uses the C++ SensorFactory to create a leaf child node on
        *scene_node* and attach the sensor.  Returns the Python
        :class:`Sensor` wrapper.

        :param spec: The SensorSpec describing the sensor to create.
        :param scene_node: The SceneNode to attach the sensor to.
            A leaf child will be created automatically.
        :return: The newly created :class:`Sensor`.
        """
        if spec.uuid in self._sensor_registry:
            raise ValueError(
                f"Sensor '{spec.uuid}' already exists in the registry. "
                "Remove it first with remove_sensor()."
            )

        # Validate that the required backend resources are loaded.
        if (
            not self.config.sim_cfg.load_semantic_mesh
            and spec.sensor_type == SensorType.SEMANTIC
        ):
            raise ValueError(
                "Cannot create a SEMANTIC sensor — semantic mesh was not loaded "
                "during Simulator init."
            )
        if (
            not self.config.sim_cfg.requires_textures
            and spec.sensor_type == SensorType.COLOR
        ):
            raise ValueError(
                "Cannot create a COLOR sensor — textures were not loaded "
                "during Simulator init."
            )
        if (
            not self.config.sim_cfg.create_renderer
            and spec.sensor_type == SensorType.DEPTH
        ):
            raise ValueError(
                "Cannot create a DEPTH sensor — renderer was not created "
                "during Simulator init."
            )

        cpp_sensor_suite = SensorFactory.create_sensors(scene_node, [spec])
        cpp_sensor = cpp_sensor_suite[spec.uuid]
        wrapper = Sensor(sim=self, sensor_object=cpp_sensor)
        self._sensor_registry[spec.uuid] = wrapper
        return wrapper

    def remove_sensor(self, uuid: str) -> None:
        """Remove and destroy a sensor by its UUID.

        Pops the sensor from the registry, deletes the underlying C++
        Sensor (and its leaf SceneNode), and closes the Python wrapper.

        :param uuid: The UUID of the sensor to remove.
        """
        if uuid not in self._sensor_registry:
            raise KeyError(f"Sensor '{uuid}' not found in the registry.")

        wrapper = self._sensor_registry.pop(uuid)
        SensorFactory.delete_sensor(wrapper.sensor_object)
        wrapper.close()

    def get_sensor(self, uuid: str) -> Sensor:
        """Look up a sensor by its UUID.

        :param uuid: The UUID of the sensor.
        :return: The :class:`Sensor` wrapper.
        """
        return self._sensor_registry[uuid]

    def render_sensors(
        self,
        sensors: Optional[List[Sensor]] = None,
    ) -> Dict[str, Union[ndarray, "Tensor"]]:
        """Render a batch of sensors and return observations.

        Draws all sensors first, then reads back all observations.
        This is the primary observation API.

        :param sensors: An explicit list of sensors to render.
            If ``None``, renders all registered sensors.
        :return: A dict mapping sensor UUID → observation array/tensor.
        """
        if sensors is None:
            sensors = list(self._sensor_registry.values())

        if not self.config.enable_batch_renderer:
            for sensor in sensors:
                sensor.draw_observation()

        observations: Dict[str, Union[ndarray, "Tensor"]] = {}
        for sensor in sensors:
            observations[sensor.uuid] = sensor.get_observation()

        return observations

    # ------------------------------------------------------------------
    # Backward-compatible sensor API
    # ------------------------------------------------------------------

    @property
    def _sensors(self) -> Dict[str, Sensor]:
        """Deprecated. Use ``sim.sensors`` instead."""
        return self._sensor_registry

    def add_sensor(
        self, sensor_spec: SensorSpec, agent_id: Optional[int] = None
    ) -> None:
        """Add a sensor to an agent's scene node and register it.

        This is a convenience wrapper around :meth:`create_sensor` that
        attaches the sensor to the given agent's scene node and updates
        the agent's config.
        """
        if agent_id is None:
            agent_id = self._default_agent_id
        agent = self.get_agent(agent_id=agent_id)

        # Validate backend resources (same checks as create_sensor).
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

        # Create via factory on agent's scene node.
        self.create_sensor(sensor_spec, agent.scene_node)

        # Keep agent config in sync.
        if sensor_spec not in agent.agent_config.sensor_specifications:
            agent.agent_config.sensor_specifications.append(sensor_spec)

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

    # ------------------------------------------------------------------
    # Async rendering
    # ------------------------------------------------------------------

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
            agent = self.get_agent(agent_id)
            for sensor_uuid in agent.scene_node.subtree_sensors:
                if sensor_uuid in self._sensor_registry:
                    self._sensor_registry[sensor_uuid].render_async()

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
            agent = self.get_agent(agent_id)
            for sensor_uuid in agent.scene_node.subtree_sensors:
                if sensor_uuid in self._sensor_registry:
                    self._sensor_registry[sensor_uuid].render_async()

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
        observations: Dict[int, Dict[str, Union[ndarray, "Tensor"]]] = OrderedDict()
        for agent_id in agent_ids:
            agent = self.get_agent(agent_id)
            agent_observations: Dict[str, Union[ndarray, "Tensor"]] = {}
            for sensor_uuid in agent.scene_node.subtree_sensors:
                if sensor_uuid in self._sensor_registry:
                    agent_observations[sensor_uuid] = self._sensor_registry[
                        sensor_uuid
                    ].get_observation_async()
            observations[agent_id] = agent_observations

        if return_single:
            return next(iter(observations.values()))
        return observations

    # ------------------------------------------------------------------
    # Synchronous observation (backward-compatible)
    # ------------------------------------------------------------------

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

        observations: Dict[int, ObservationDict] = OrderedDict()

        # Collect sensors per agent from their subtrees.
        agent_sensor_map: Dict[int, List[Sensor]] = {}
        for agent_id in agent_ids:
            agent = self.get_agent(agent_id)
            agent_sensors = []
            for sensor_uuid in agent.scene_node.subtree_sensors:
                if sensor_uuid in self._sensor_registry:
                    agent_sensors.append(self._sensor_registry[sensor_uuid])
            agent_sensor_map[agent_id] = agent_sensors

        # Draw all sensors.
        if not self.config.enable_batch_renderer:
            for agent_sensors in agent_sensor_map.values():
                for sensor in agent_sensors:
                    sensor.draw_observation()

        # Read observations per agent.
        for agent_id, agent_sensors in agent_sensor_map.items():
            agent_observations: ObservationDict = {}
            for sensor in agent_sensors:
                agent_observations[sensor.uuid] = sensor.get_observation()
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
