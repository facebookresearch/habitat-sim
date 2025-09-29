#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from __future__ import annotations

from abc import ABC, abstractmethod

import magnum as mn
import numpy as np

import habitat_sim

DEFAULT_OBSERVATION_TYPE: str = "color"
OBSERVATION_PLACEHOLDER_NAME: str = (
    "s"  # Dummy sensor name for circumventing habitat-sim's recording API.
)


class VideoSource(ABC):
    def __init__(self):
        self._recording_images: list[dict[str, np.ndarray]] = []

    @abstractmethod
    def record_frame(self):
        pass

    def reset(self):
        self._recording_images.clear()

    @property
    def frames(self) -> list[dict[str, np.ndarray]]:
        return self._recording_images

    def _record_frame_internal(self, frame: np.ndarray):
        self._recording_images.append({OBSERVATION_PLACEHOLDER_NAME: frame})


class VideoSourceFramebuffer(VideoSource):
    def __init__(
        self,
        framebuffer: mn.gl.AbstractFramebuffer = mn.gl.default_framebuffer,
    ):
        super().__init__()
        self._framebuffer = framebuffer
        viewport = framebuffer.viewport
        self._gpu_to_cpu_buffer = np.empty(
            (
                viewport.size_y(),
                viewport.size_x(),
                3,
            ),
            dtype=np.uint8,
        )
        self._gpu_to_cpu_image = mn.MutableImageView2D(
            mn.PixelFormat.RGB8_UNORM,
            [
                viewport.size_x(),
                viewport.size_y(),
            ],
            self._gpu_to_cpu_buffer,
        )
        # Flip the view vertically for presentation
        self._gpu_to_cpu_buffer = np.flip(self._gpu_to_cpu_buffer.view(), axis=0)

    def record_frame(self):
        viewport = self._framebuffer.viewport
        rect = mn.Range2Di(
            mn.Vector2i(),
            mn.Vector2i(viewport.size_x(), viewport.size_y()),
        )
        mn.gl.default_framebuffer.read(rect, self._gpu_to_cpu_image)
        frame = self._gpu_to_cpu_buffer.copy()
        self._record_frame_internal(frame)


class VideoRecorder:
    """
    Tool for recording creating videos from the specified magnum framebuffer.

    **Usage:**
    * `record_video_frame`: Create a new frame (typically once per simulation step).
    * `save_video`: Save the recorded sequence.
    * `reset`: Clear the recorded sequence.
    """

    def __init__(self, video_source: VideoSource):
        self._source = video_source
        self.reset()

    def reset(self):
        """
        Clear the recorded frames for reusing the recorder.
        """
        self._source.reset()

    def show_frame(self, frame_id: int):
        """
        Show a recorded frame in an external viewer.
        """
        frames = self._source.frames
        if 0 <= frame_id < len(frames):
            habitat_sim.utils.viz_utils.observation_to_image(
                frames[frame_id][OBSERVATION_PLACEHOLDER_NAME], DEFAULT_OBSERVATION_TYPE
            ).show()
        else:
            print(f"Frame id {frame_id} is out of range [0, {len(frames)})")

    def save_video(self, output_file_path: str, fps: int = 30):
        """
        Create a video from the recorded frames.
        This does not clear the recording video. Call `reset` to reuse the recorder.
        """
        frames = self._source.frames
        if len(frames) > 0:
            habitat_sim.utils.viz_utils.make_video(
                observations=frames,
                primary_obs=OBSERVATION_PLACEHOLDER_NAME,
                primary_obs_type=DEFAULT_OBSERVATION_TYPE,
                video_file=output_file_path,
                open_vid=False,
                fps=fps,
            )
        else:
            print(
                f"No frame recorded. Skipping video generation for '{output_file_path}'."
            )

    def record_video_frame(self):
        """
        Record the contents of the framebuffer.
        """
        self._source.record_frame()
