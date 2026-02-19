#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from typing import TYPE_CHECKING, Union

import magnum as mn
import numpy as np
from numpy import ndarray

try:
    import torch
    from torch import Tensor

    _HAS_TORCH = True
except ImportError:
    _HAS_TORCH = False

import habitat_sim.errors
from habitat_sim.bindings import cuda_enabled
from habitat_sim.sensor import SensorSpec, SensorType
from habitat_sim.sensors.noise_models import make_sensor_noise_model

if TYPE_CHECKING:
    from habitat_sim import bindings as hsim
    from habitat_sim._ext.habitat_sim_bindings import SceneNode
    from habitat_sim.sim import SimulatorBackend


class Sensor:
    r"""Wrapper around a C++ habitat_sim.Sensor that manages observation
    buffers, noise models, and render target binding.

    Sensors are created via :meth:`Simulator.create_sensor` and are
    decoupled from any particular Agent.  They can be attached to any
    SceneNode in the SceneGraph -- agent body nodes, articulated-object
    link nodes, or arbitrary static nodes.

    :property uuid: The unique identifier for this sensor.
    :property node: The SceneNode this sensor's leaf node lives under.
    :property sensor_object: The underlying C++ Sensor feature.
    :property spec: The SensorSpec that configured this sensor.
    """

    buffer: Union[np.ndarray, "Tensor"]

    def __init__(
        self,
        sim: "SimulatorBackend",
        sensor_object: "hsim.Sensor",
    ) -> None:
        self._sim = sim
        self._sensor_object = sensor_object
        self._spec = self._sensor_object.specification()

        if not self._sim.config.enable_batch_renderer:
            self._initialize_sensor()

    # ------------------------------------------------------------------
    # Public properties
    # ------------------------------------------------------------------

    @property
    def uuid(self) -> str:
        """The unique identifier for this sensor (from its SensorSpec)."""
        return self._spec.uuid

    @property
    def node(self) -> "SceneNode":
        """The leaf SceneNode this sensor is attached to."""
        return self._sensor_object.node

    @property
    def sensor_object(self) -> "hsim.Sensor":
        """The underlying C++ Sensor object."""
        return self._sensor_object

    @property
    def spec(self) -> SensorSpec:
        """The SensorSpec that configured this sensor."""
        return self._spec

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

    def _initialize_sensor(self) -> None:
        r"""Allocate buffers and initialize the noise model."""
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
                    resolution[0],
                    resolution[1],
                    dtype=torch.int32,
                    device=device,
                )
            elif self._spec.sensor_type == SensorType.DEPTH:
                self._buffer = torch.empty(
                    resolution[0],
                    resolution[1],
                    dtype=torch.float32,
                    device=device,
                )
            else:
                self._buffer = torch.empty(
                    resolution[0],
                    resolution[1],
                    4,
                    dtype=torch.uint8,
                    device=device,
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

    # ------------------------------------------------------------------
    # Synchronous rendering
    # ------------------------------------------------------------------

    def draw_observation(self) -> None:
        """Render this sensor's observation to the framebuffer."""
        assert not self._sim.config.enable_batch_renderer

        if self._spec.sensor_type == SensorType.AUDIO:
            return

        assert self._sim.renderer is not None
        if not self._sensor_object.object:
            raise habitat_sim.errors.InvalidAttachedObject(
                "Sensor observation requested but sensor is invalid."
                " (has it been detached from a scene node?)"
            )
        self._sim.renderer.draw(self._sensor_object, self._sim)

    def get_observation(self) -> Union[ndarray, "Tensor"]:
        """Read the rendered observation from the framebuffer and apply
        the noise model."""
        if self._spec.sensor_type == SensorType.AUDIO:
            return self._get_audio_observation()

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

    # ------------------------------------------------------------------
    # Asynchronous rendering
    # ------------------------------------------------------------------

    def render_async(self) -> None:
        """Enqueue this sensor's draw job for async rendering.

        Call :py:meth:`Simulator.renderer.start_draw_jobs` after enqueuing
        all desired sensors, then call :py:meth:`get_observation_async` to
        read back results.
        """
        self._draw_observation_async()

    def get_observation_async(self) -> Union[ndarray, "Tensor"]:
        """Read the observation after an async render completes."""
        return self._get_observation_async()

    def _draw_observation_async(self) -> None:
        assert not self._sim.config.enable_batch_renderer

        if self._spec.sensor_type == SensorType.AUDIO:
            return

        assert self._sim.renderer is not None
        if (
            self._spec.sensor_type == SensorType.SEMANTIC
            and self._sim.get_active_scene_graph()
            is not self._sim.get_active_semantic_scene_graph()
        ):
            raise RuntimeError(
                "Async drawing doesn't support semantic rendering "
                "when there are multiple scene graphs"
            )

        if not self._sensor_object.object:
            raise habitat_sim.errors.InvalidAttachedObject(
                "Sensor observation requested but sensor is invalid."
                " (has it been detached from a scene node?)"
            )

        # Walk up from the sensor leaf node to find the nearest ancestor
        # that is in the correct scene graph, and reparent it if needed
        # for semantic rendering.
        sensor_node = self._sensor_object.node
        # The sensor's parent (the node passed to create_sensor) is the
        # natural attachment point.  For agent-attached sensors, this is
        # the agent's scene node.  We reparent that into the correct
        # scene graph for this render.
        attach_node = sensor_node.parent

        if self._spec.sensor_type == SensorType.SEMANTIC:
            if self._sim.semantic_scene is None:
                raise RuntimeError(
                    "SemanticSensor observation requested but "
                    "no SemanticScene is loaded"
                )
            scene = self._sim.get_active_semantic_scene_graph()
        else:
            scene = self._sim.get_active_scene_graph()

        if attach_node is not None:
            attach_node.parent = scene.get_root_node()

        render_flags = habitat_sim.gfx.Camera.Flags.NONE
        if self._sim.frustum_culling:
            render_flags |= habitat_sim.gfx.Camera.Flags.FRUSTUM_CULLING

        self._sim.renderer.enqueue_async_draw_job(
            self._sensor_object, scene, self.view, render_flags
        )

    def _get_observation_async(self) -> Union[ndarray, "Tensor"]:
        if self._spec.sensor_type == SensorType.AUDIO:
            return self._get_audio_observation()
        if self._spec.gpu2gpu_transfer:
            obs = self._buffer.flip(0)  # type: ignore[union-attr]
        else:
            obs = np.flip(self._buffer, axis=0)
        return self._noise_model(obs)

    # ------------------------------------------------------------------
    # Audio
    # ------------------------------------------------------------------

    def _get_audio_observation(self) -> Union[ndarray, "Tensor"]:
        assert self._spec.sensor_type == SensorType.AUDIO
        audio_sensor = self._sensor_object

        # Derive listener orientation from the sensor node's ancestry.
        # The audio sensor's parent node carries the listener pose.
        node = audio_sensor.node
        parent = node.parent
        if parent is not None:
            rot = parent.rotation
            rot_array = np.array(
                [rot.scalar, rot.vector[0], rot.vector[1], rot.vector[2]]
            )
        else:
            rot_array = np.array([1.0, 0.0, 0.0, 0.0])

        audio_sensor.setAudioListenerTransform(
            audio_sensor.node.absolute_translation,
            rot_array,
        )
        audio_sensor.runSimulation(self._sim)
        return audio_sensor.getIR()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def close(self) -> None:
        """Release references held by this wrapper."""
        self._sim = None
        self._sensor_object = None
        self._spec = None
