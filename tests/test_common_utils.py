#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import numpy as np

# Need to import quaternion library here despite it not being used or else importing
# habitat_sim below will cause an invalid free() when audio is enabled in sim compilation
import quaternion  # noqa: F401

from habitat_sim.utils.common.common import (
    colorize_ids,
    d3_40_colors_hex,
    d3_40_colors_rgb,
)

# -- d3_40_colors_rgb data tests --


class TestD340ColorsRgb:
    """Tests for the d3_40_colors_rgb constant array."""

    def test_shape(self):
        assert d3_40_colors_rgb.shape == (40, 3)

    def test_dtype(self):
        assert d3_40_colors_rgb.dtype == np.uint8

    def test_values_in_range(self):
        """All color values should be valid uint8 (0-255)."""
        assert d3_40_colors_rgb.min() >= 0
        assert d3_40_colors_rgb.max() <= 255

    def test_first_color(self):
        np.testing.assert_array_equal(d3_40_colors_rgb[0], [31, 119, 180])

    def test_last_color(self):
        np.testing.assert_array_equal(d3_40_colors_rgb[39], [222, 158, 214])


# -- d3_40_colors_hex data tests --


class TestD340ColorsHex:
    """Tests for the d3_40_colors_hex constant list."""

    def test_length(self):
        assert len(d3_40_colors_hex) == 40

    def test_format(self):
        """All entries should be hex strings starting with 0x."""
        for color in d3_40_colors_hex:
            assert color.startswith("0x")
            assert len(color) == 8  # "0x" + 6 hex chars

    def test_first_entry(self):
        assert d3_40_colors_hex[0] == "0x1f77b4"

    def test_rgb_hex_consistency(self):
        """First RGB color should match first hex color."""
        r, g, b = d3_40_colors_rgb[0]
        expected_hex = f"0x{r:02x}{g:02x}{b:02x}"
        assert d3_40_colors_hex[0] == expected_hex


# -- colorize_ids tests --


class TestColorizeIds:
    """Tests for the colorize_ids function."""

    def test_output_shape(self):
        """Output should be (H, W, 3) for an (H, W) input."""
        ids = np.array([[0, 1], [2, 3]], dtype=np.int32)
        out = colorize_ids(ids)
        assert out.shape == (2, 2, 3)

    def test_output_dtype(self):
        """Output should be uint8 for display."""
        ids = np.zeros((3, 3), dtype=np.int32)
        out = colorize_ids(ids)
        assert out.dtype == np.uint8

    def test_negative_ids_are_black(self):
        """Negative IDs (invalid/background) map to [0, 0, 0]."""
        ids = np.full((2, 2), -1, dtype=np.int32)
        out = colorize_ids(ids)
        np.testing.assert_array_equal(out, np.zeros((2, 2, 3), dtype=np.uint8))

    def test_zero_id_maps_to_first_color(self):
        """ID 0 should map to d3_40_colors_rgb[0]."""
        ids = np.array([[0]], dtype=np.int32)
        out = colorize_ids(ids)
        np.testing.assert_array_equal(out[0, 0], d3_40_colors_rgb[0])

    def test_wrapping_at_40(self):
        """IDs >= 40 should wrap around via modulo."""
        ids = np.array([[40]], dtype=np.int32)
        out = colorize_ids(ids)
        np.testing.assert_array_equal(out[0, 0], d3_40_colors_rgb[0])

        ids = np.array([[41]], dtype=np.int32)
        out = colorize_ids(ids)
        np.testing.assert_array_equal(out[0, 0], d3_40_colors_rgb[1])

    def test_mixed_positive_negative(self):
        """Positive IDs get color, negative IDs remain black."""
        ids = np.array([[0, -1], [5, -2]], dtype=np.int32)
        out = colorize_ids(ids)
        np.testing.assert_array_equal(out[0, 0], d3_40_colors_rgb[0])
        np.testing.assert_array_equal(out[0, 1], [0, 0, 0])
        np.testing.assert_array_equal(out[1, 0], d3_40_colors_rgb[5])
        np.testing.assert_array_equal(out[1, 1], [0, 0, 0])

    def test_single_pixel(self):
        """1x1 image with a valid ID."""
        ids = np.array([[7]], dtype=np.int32)
        out = colorize_ids(ids)
        np.testing.assert_array_equal(out[0, 0], d3_40_colors_rgb[7])

    def test_large_id(self):
        """Very large ID wraps correctly."""
        ids = np.array([[1000]], dtype=np.int32)
        out = colorize_ids(ids)
        np.testing.assert_array_equal(out[0, 0], d3_40_colors_rgb[1000 % 40])

    def test_all_same_id(self):
        """All pixels with the same ID should produce uniform color."""
        ids = np.full((4, 4), 3, dtype=np.int32)
        out = colorize_ids(ids)
        expected = np.tile(d3_40_colors_rgb[3], (4, 4, 1))
        np.testing.assert_array_equal(out, expected)
