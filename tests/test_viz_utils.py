#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


import numpy as np
import pytest
from PIL import Image

from habitat_sim.utils.viz_utils import (
    border_frames_from_overlay,
    depth_to_rgb,
    get_island_colored_map,
    is_notebook,
    make_video_frame,
    observation_to_image,
    semantic_to_rgb,
)

# -- depth_to_rgb tests --


class TestDepthToRgb:
    """Tests for the depth_to_rgb function."""

    def test_output_dtype(self):
        depth = np.ones((4, 4), dtype=np.float32) * 5.0
        result = depth_to_rgb(depth)
        assert result.dtype == np.uint8

    def test_output_shape_matches_input(self):
        depth = np.zeros((10, 20), dtype=np.float32)
        result = depth_to_rgb(depth)
        assert result.shape == (10, 20)

    def test_zero_depth_maps_to_zero(self):
        depth = np.zeros((2, 2), dtype=np.float32)
        result = depth_to_rgb(depth)
        np.testing.assert_array_equal(result, 0)

    def test_max_depth_maps_to_255(self):
        depth = np.ones((2, 2), dtype=np.float32) * 10.0
        result = depth_to_rgb(depth, clip_max=10.0)
        np.testing.assert_array_equal(result, 255)

    def test_clipping_above_max(self):
        """Values above clip_max should be clamped to 255."""
        depth = np.ones((2, 2), dtype=np.float32) * 100.0
        result = depth_to_rgb(depth, clip_max=10.0)
        np.testing.assert_array_equal(result, 255)

    def test_negative_depth_clipped_to_zero(self):
        depth = np.full((2, 2), -5.0, dtype=np.float32)
        result = depth_to_rgb(depth)
        np.testing.assert_array_equal(result, 0)

    def test_half_depth(self):
        """Half of clip_max should give roughly 127 or 128."""
        depth = np.ones((1, 1), dtype=np.float32) * 5.0
        result = depth_to_rgb(depth, clip_max=10.0)
        assert result[0, 0] == 127 or result[0, 0] == 128

    def test_custom_clip_max(self):
        depth = np.ones((1, 1), dtype=np.float32) * 2.0
        result = depth_to_rgb(depth, clip_max=2.0)
        np.testing.assert_array_equal(result, 255)

    def test_linear_gradient(self):
        """Increasing depth should produce monotonically increasing grayscale."""
        depth = np.array([[0.0, 2.5, 5.0, 7.5, 10.0]], dtype=np.float32)
        result = depth_to_rgb(depth, clip_max=10.0)
        for i in range(len(result[0]) - 1):
            assert result[0, i] <= result[0, i + 1]


# -- semantic_to_rgb tests --


class TestSemanticToRgb:
    """Tests for the semantic_to_rgb function."""

    def test_returns_pil_image(self):
        sem = np.zeros((4, 4), dtype=np.int32)
        result = semantic_to_rgb(sem)
        assert isinstance(result, Image.Image)

    def test_output_mode_is_rgba(self):
        sem = np.zeros((4, 4), dtype=np.int32)
        result = semantic_to_rgb(sem)
        assert result.mode == "RGBA"

    def test_output_dimensions(self):
        sem = np.zeros((10, 20), dtype=np.int32)
        result = semantic_to_rgb(sem)
        assert result.size == (20, 10)  # PIL size is (width, height)

    def test_different_ids_produce_different_colors(self):
        """Distinct semantic IDs should map to different colors."""
        sem = np.array([[0, 1]], dtype=np.int32)
        result = semantic_to_rgb(sem)
        pixels = list(result.getdata())
        assert pixels[0] != pixels[1]

    def test_wrapping_ids(self):
        """IDs >= 40 wrap modulo 40, so ID 0 and ID 40 should match."""
        sem0 = np.array([[0]], dtype=np.int32)
        sem40 = np.array([[40]], dtype=np.int32)
        rgb0 = list(semantic_to_rgb(sem0).getdata())[0]
        rgb40 = list(semantic_to_rgb(sem40).getdata())[0]
        assert rgb0 == rgb40


# -- observation_to_image tests --


class TestObservationToImage:
    """Tests for the observation_to_image function."""

    def test_color_observation(self):
        """Color observations should return an RGB PIL Image."""
        obs = np.random.randint(0, 255, (48, 64, 4), dtype=np.uint8)
        result = observation_to_image(obs, "color")
        assert isinstance(result, Image.Image)
        assert result.size == (64, 48)

    def test_depth_observation(self):
        obs = np.random.rand(48, 64).astype(np.float32) * 10.0
        result = observation_to_image(obs, "depth", depth_clip=10.0)
        assert isinstance(result, Image.Image)

    def test_semantic_observation(self):
        obs = np.random.randint(0, 20, (48, 64), dtype=np.int32)
        result = observation_to_image(obs, "semantic")
        assert isinstance(result, Image.Image)

    def test_unsupported_type_returns_none(self):
        obs = np.zeros((4, 4), dtype=np.uint8)
        result = observation_to_image(obs, "unknown_type")
        assert result is None

    def test_depth_clip_parameter_passed(self):
        """Depth clip should affect the output."""
        obs = np.ones((4, 4), dtype=np.float32) * 5.0
        result_clip10 = observation_to_image(obs, "depth", depth_clip=10.0)
        result_clip5 = observation_to_image(obs, "depth", depth_clip=5.0)
        arr10 = np.array(result_clip10)
        arr5 = np.array(result_clip5)
        # clip_max=5.0 should make 5.0 appear as white (255)
        # clip_max=10.0 should make 5.0 appear as ~127
        assert arr5.mean() > arr10.mean()


# -- border_frames_from_overlay tests --


class TestBorderFramesFromOverlay:
    """Tests for the border_frames_from_overlay function."""

    def test_none_overlay_returns_empty(self):
        assert border_frames_from_overlay(None) == []

    def test_empty_list_returns_empty(self):
        assert border_frames_from_overlay([]) == []

    def test_single_overlay_returns_one_frame(self):
        settings = [{"dims": (100, 80), "border": 5}]
        result = border_frames_from_overlay(settings)
        assert len(result) == 1
        assert isinstance(result[0], Image.Image)

    def test_border_frame_dimensions(self):
        """Border frame should be dims + 2*border in each direction."""
        settings = [{"dims": (100, 80), "border": 5}]
        result = border_frames_from_overlay(settings)
        # observation_to_image("color") on a (H, W, 3) array
        # border_image shape: (80 + 10, 100 + 10, 3) = (90, 110, 3)
        # PIL Image size is (width, height) = (110, 90)
        assert result[0].size == (110, 90)

    def test_multiple_overlays(self):
        settings = [
            {"dims": (50, 50), "border": 2},
            {"dims": (30, 30), "border": 4},
        ]
        result = border_frames_from_overlay(settings)
        assert len(result) == 2

    def test_custom_border_color(self):
        """Custom border_color should change the output."""
        settings_gray = [{"dims": (10, 10), "border": 1}]
        settings_red = [{"dims": (10, 10), "border": 1, "border_color": [255, 0, 0]}]
        result_gray = border_frames_from_overlay(settings_gray)
        result_red = border_frames_from_overlay(settings_red)
        # The border pixels should differ between gray (150) and red (255,0,0)
        arr_gray = np.array(result_gray[0])
        arr_red = np.array(result_red[0])
        assert not np.array_equal(arr_gray, arr_red)


# -- make_video_frame tests --


class TestMakeVideoFrame:
    """Tests for the make_video_frame function."""

    def test_basic_frame(self):
        obs = {"color": np.random.randint(0, 255, (48, 64, 4), dtype=np.uint8)}
        frame = make_video_frame(obs, "color", "color", video_dims=None)
        assert isinstance(frame, Image.Image)

    def test_video_dims_resize(self):
        obs = {"color": np.random.randint(0, 255, (48, 64, 4), dtype=np.uint8)}
        frame = make_video_frame(obs, "color", "color", video_dims=(320, 240))
        assert frame.size == (320, 240)

    def test_missing_primary_obs_raises(self):
        obs = {"color": np.zeros((4, 4, 4), dtype=np.uint8)}

        def bad_obs_to_image(img, obs_type, depth_clip=10.0):
            return None

        with pytest.raises(RuntimeError, match="primary image processing failed"):
            make_video_frame(
                obs,
                "color",
                "color",
                video_dims=None,
                observation_to_image=bad_obs_to_image,
            )

    def test_with_overlay(self):
        obs = {
            "color": np.random.randint(0, 255, (100, 100, 4), dtype=np.uint8),
            "depth": np.random.rand(100, 100).astype(np.float32) * 10.0,
        }
        overlay = [
            {
                "obs": "depth",
                "type": "depth",
                "dims": (40, 40),
                "pos": (5, 5),
                "border": 2,
            }
        ]
        frame = make_video_frame(
            obs, "color", "color", video_dims=None, overlay_settings=overlay
        )
        assert isinstance(frame, Image.Image)


# -- is_notebook tests --


class TestIsNotebook:
    """Tests for the is_notebook function."""

    def test_returns_false_in_script(self):
        """Running in a normal script should return False."""
        assert is_notebook() is False

    def test_returns_bool(self):
        assert isinstance(is_notebook(), bool)


# -- get_island_colored_map tests --


class TestGetIslandColoredMap:
    """Tests for the get_island_colored_map function."""

    def test_returns_pil_image(self):
        data = np.full((10, 10), -1, dtype=np.int32)
        result = get_island_colored_map(data)
        assert isinstance(result, Image.Image)

    def test_output_mode_is_rgb(self):
        data = np.full((10, 10), -1, dtype=np.int32)
        result = get_island_colored_map(data)
        assert result.mode == "RGB"

    def test_all_negative_is_white(self):
        """All-negative map (no islands) should be white."""
        data = np.full((5, 5), -1, dtype=np.int32)
        result = get_island_colored_map(data)
        arr = np.array(result)
        np.testing.assert_array_equal(arr, 255)

    def test_output_dimensions(self):
        data = np.full((8, 12), -1, dtype=np.int32)
        result = get_island_colored_map(data)
        assert result.size == (
            8,
            12,
        )  # PIL: (width, height) but image shape is data.shape

    def test_single_island_colored(self):
        """A single island (index 0) should have non-white pixels."""
        data = np.full((5, 5), -1, dtype=np.int32)
        data[2, 2] = 0
        result = get_island_colored_map(data)
        arr = np.array(result)
        # At least one pixel should not be white
        assert not np.all(arr == 255)

    def test_many_islands_overflow_to_random(self):
        """More than 40 islands should still produce a valid image without error."""
        data = np.arange(50).reshape(5, 10).astype(np.int32)
        result = get_island_colored_map(data)
        assert isinstance(result, Image.Image)
