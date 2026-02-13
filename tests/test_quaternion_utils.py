#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import math

import magnum as mn
import numpy as np
import pytest

# Need to import quaternion library here despite it not being used or else importing
# habitat_sim below will cause an invalid free() when audio is enabled in sim compilation
import quaternion
import quaternion as qt

from habitat_sim.utils.common.quaternion_utils import (
    angle_between_quats,
    quat_from_angle_axis,
    quat_from_coeffs,
    quat_from_magnum,
    quat_from_two_vectors,
    quat_rotate_vector,
    quat_to_angle_axis,
    quat_to_coeffs,
    quat_to_magnum,
    random_quaternion,
)

# -- quat_from_coeffs / quat_to_coeffs round-trip tests --


class TestQuatCoeffs:
    """Tests for quat_from_coeffs and quat_to_coeffs."""

    def test_identity_quaternion(self):
        """Identity quaternion [0, 0, 0, 1] round-trips correctly."""
        coeffs = np.array([0.0, 0.0, 0.0, 1.0])
        q = quat_from_coeffs(coeffs)
        assert q.real == pytest.approx(1.0)
        np.testing.assert_array_almost_equal(q.imag, [0.0, 0.0, 0.0])

        out = quat_to_coeffs(q)
        np.testing.assert_array_almost_equal(out, coeffs)

    def test_arbitrary_quaternion_round_trip(self):
        """An arbitrary normalized quaternion round-trips through coeffs."""
        coeffs = np.array([0.1, 0.2, 0.3, 0.9])
        coeffs /= np.linalg.norm(coeffs)
        q = quat_from_coeffs(coeffs)
        out = quat_to_coeffs(q)
        np.testing.assert_array_almost_equal(out, coeffs)

    def test_coeffs_ordering(self):
        """Verify [b, c, d, a] ordering: imag first, real last."""
        coeffs = np.array([1.0, 2.0, 3.0, 4.0])
        q = quat_from_coeffs(coeffs)
        assert q.real == pytest.approx(4.0)
        np.testing.assert_array_almost_equal(q.imag, [1.0, 2.0, 3.0])

    def test_zero_quaternion(self):
        """All-zero coefficients produce zero quaternion."""
        coeffs = np.array([0.0, 0.0, 0.0, 0.0])
        q = quat_from_coeffs(coeffs)
        assert q.real == pytest.approx(0.0)
        np.testing.assert_array_almost_equal(q.imag, [0.0, 0.0, 0.0])

    @pytest.mark.parametrize(
        "coeffs",
        [
            np.array([1.0, 0.0, 0.0, 0.0]),
            np.array([0.0, 1.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 1.0, 0.0]),
            np.array([0.0, 0.0, 0.0, 1.0]),
        ],
    )
    def test_axis_aligned_quaternions(self, coeffs):
        """Single-component quaternions round-trip correctly."""
        q = quat_from_coeffs(coeffs)
        out = quat_to_coeffs(q)
        np.testing.assert_array_almost_equal(out, coeffs)

    def test_to_coeffs_output_shape(self):
        """quat_to_coeffs returns a 1D array of length 4."""
        q = qt.quaternion(1, 0, 0, 0)
        out = quat_to_coeffs(q)
        assert out.shape == (4,)


# -- quat_to_magnum / quat_from_magnum round-trip tests --


class TestMagnumConversion:
    """Tests for quat_to_magnum and quat_from_magnum."""

    def test_identity_round_trip(self):
        q = qt.quaternion(1, 0, 0, 0)
        mn_q = quat_to_magnum(q)
        back = quat_from_magnum(mn_q)
        assert back.real == pytest.approx(q.real)
        np.testing.assert_array_almost_equal(back.imag, q.imag)

    def test_arbitrary_round_trip(self):
        q = qt.quaternion(0.5, 0.5, 0.5, 0.5)
        mn_q = quat_to_magnum(q)
        back = quat_from_magnum(mn_q)
        assert back.real == pytest.approx(q.real)
        np.testing.assert_array_almost_equal(back.imag, q.imag)

    def test_magnum_quaternion_properties(self):
        """Verify that the magnum quaternion has expected scalar/vector."""
        q = qt.quaternion(0.7071, 0.7071, 0.0, 0.0)
        mn_q = quat_to_magnum(q)
        assert mn_q.scalar == pytest.approx(0.7071, abs=1e-4)
        np.testing.assert_array_almost_equal(
            list(mn_q.vector), [0.7071, 0.0, 0.0], decimal=4
        )


# -- quat_to_angle_axis / quat_from_angle_axis tests --


class TestAngleAxis:
    """Tests for quat_to_angle_axis and quat_from_angle_axis."""

    def test_identity_angle_axis(self):
        """Identity quaternion maps to zero angle."""
        q = qt.quaternion(1, 0, 0, 0)
        theta, axis = quat_to_angle_axis(q)
        assert theta == pytest.approx(0.0, abs=1e-5)
        np.testing.assert_array_almost_equal(axis, [1, 0, 0])

    @pytest.mark.parametrize(
        "angle,axis",
        [
            (math.pi / 2, np.array([1.0, 0.0, 0.0])),
            (math.pi / 2, np.array([0.0, 1.0, 0.0])),
            (math.pi / 2, np.array([0.0, 0.0, 1.0])),
            (math.pi, np.array([1.0, 0.0, 0.0])),
            (0.1, np.array([0.0, 0.0, 1.0])),
        ],
    )
    def test_angle_axis_round_trip(self, angle, axis):
        """Angle-axis round-trips through quaternion."""
        q = quat_from_angle_axis(angle, axis)
        theta_out, axis_out = quat_to_angle_axis(q)
        assert theta_out == pytest.approx(angle, abs=1e-5)
        np.testing.assert_array_almost_equal(np.abs(axis_out), np.abs(axis), decimal=5)

    def test_from_angle_axis_normalizes(self):
        """quat_from_angle_axis normalizes a non-unit axis."""
        axis = np.array([2.0, 0.0, 0.0])
        q = quat_from_angle_axis(math.pi / 4, axis)
        assert np.isclose(q.norm(), 1.0, atol=1e-6)


# -- quat_from_two_vectors tests --


class TestFromTwoVectors:
    """Tests for quat_from_two_vectors."""

    def test_same_vector(self):
        """Rotating a vector to itself gives identity."""
        v = np.array([1.0, 0.0, 0.0])
        q = quat_from_two_vectors(v, v)
        assert np.isclose(q.norm(), 1.0, atol=1e-6)
        rotated = quat_rotate_vector(q, v)
        np.testing.assert_array_almost_equal(rotated, v, decimal=5)

    def test_perpendicular_vectors(self):
        """Rotation from x-axis to y-axis is 90 degrees."""
        v0 = np.array([1.0, 0.0, 0.0])
        v1 = np.array([0.0, 1.0, 0.0])
        q = quat_from_two_vectors(v0, v1)
        rotated = quat_rotate_vector(q, v0)
        np.testing.assert_array_almost_equal(rotated, v1, decimal=5)

    def test_opposite_vectors(self):
        """Rotation from +x to -x (near-opposite) produces valid quaternion."""
        v0 = np.array([1.0, 0.0, 0.0])
        v1 = np.array([-1.0, 0.0, 0.0])
        q = quat_from_two_vectors(v0, v1)
        assert np.isclose(q.norm(), 1.0, atol=1e-5)
        rotated = quat_rotate_vector(q, v0)
        np.testing.assert_array_almost_equal(rotated, v1, decimal=4)

    def test_non_unit_vectors(self):
        """Non-unit input vectors still produce correct rotation."""
        v0 = np.array([3.0, 0.0, 0.0])
        v1 = np.array([0.0, 5.0, 0.0])
        q = quat_from_two_vectors(v0, v1)
        v0_unit = v0 / np.linalg.norm(v0)
        rotated = quat_rotate_vector(q, v0_unit)
        v1_unit = v1 / np.linalg.norm(v1)
        np.testing.assert_array_almost_equal(rotated, v1_unit, decimal=5)

    def test_arbitrary_vectors(self):
        """Arbitrary 3D vectors produce a valid rotation."""
        v0 = np.array([1.0, 2.0, 3.0])
        v1 = np.array([-2.0, 1.0, 0.5])
        q = quat_from_two_vectors(v0, v1)
        assert np.isclose(q.norm(), 1.0, atol=1e-5)
        v0_unit = v0 / np.linalg.norm(v0)
        v1_unit = v1 / np.linalg.norm(v1)
        rotated = quat_rotate_vector(q, v0_unit)
        np.testing.assert_array_almost_equal(rotated, v1_unit, decimal=5)


# -- angle_between_quats tests --


class TestAngleBetweenQuats:
    """Tests for angle_between_quats."""

    def test_same_quaternion(self):
        """Angle between a quaternion and itself is 0."""
        q = mn.Quaternion.identity_init()
        assert angle_between_quats(q, q) == pytest.approx(0.0, abs=1e-6)

    def test_90_degree_rotation(self):
        """90-degree rotation around y-axis."""
        q1 = mn.Quaternion.identity_init()
        q2 = mn.Quaternion.rotation(mn.Deg(90.0), mn.Vector3.y_axis())
        angle = angle_between_quats(q1, q2)
        assert angle == pytest.approx(math.pi / 2, abs=1e-4)

    def test_180_degree_rotation(self):
        """180-degree rotation should produce π radians."""
        q1 = mn.Quaternion.identity_init()
        q2 = mn.Quaternion.rotation(mn.Deg(180.0), mn.Vector3.y_axis())
        angle = angle_between_quats(q1, q2)
        assert angle == pytest.approx(math.pi, abs=1e-4)

    def test_symmetry(self):
        """angle_between_quats(q1, q2) == angle_between_quats(q2, q1)."""
        q1 = mn.Quaternion.rotation(mn.Deg(30.0), mn.Vector3.x_axis())
        q2 = mn.Quaternion.rotation(mn.Deg(75.0), mn.Vector3.y_axis())
        assert angle_between_quats(q1, q2) == pytest.approx(
            angle_between_quats(q2, q1), abs=1e-5
        )


# -- quat_rotate_vector tests --


class TestQuatRotateVector:
    """Tests for quat_rotate_vector."""

    def test_identity_rotation(self):
        """Identity quaternion does not change the vector."""
        q = qt.quaternion(1, 0, 0, 0)
        v = np.array([1.0, 2.0, 3.0])
        rotated = quat_rotate_vector(q, v)
        np.testing.assert_array_almost_equal(rotated, v)

    def test_90_deg_z_rotation(self):
        """90-degree rotation around z-axis: x → y."""
        q = quat_from_angle_axis(math.pi / 2, np.array([0.0, 0.0, 1.0]))
        v = np.array([1.0, 0.0, 0.0])
        rotated = quat_rotate_vector(q, v)
        np.testing.assert_array_almost_equal(rotated, [0.0, 1.0, 0.0], decimal=5)

    def test_180_deg_rotation(self):
        """180-degree rotation around z-axis: x → -x."""
        q = quat_from_angle_axis(math.pi, np.array([0.0, 0.0, 1.0]))
        v = np.array([1.0, 0.0, 0.0])
        rotated = quat_rotate_vector(q, v)
        np.testing.assert_array_almost_equal(rotated, [-1.0, 0.0, 0.0], decimal=5)

    def test_preserves_vector_length(self):
        """Rotation preserves the magnitude of the vector."""
        q = quat_from_angle_axis(1.23, np.array([1.0, 1.0, 1.0]))
        v = np.array([3.0, 4.0, 0.0])
        rotated = quat_rotate_vector(q, v)
        assert np.linalg.norm(rotated) == pytest.approx(np.linalg.norm(v), abs=1e-6)


# -- random_quaternion tests --


class TestRandomQuaternion:
    """Tests for random_quaternion."""

    def test_returns_magnum_quaternion(self):
        q = random_quaternion()
        assert isinstance(q, mn.Quaternion)

    def test_is_normalized(self):
        """Random quaternion should be approximately unit length."""
        for _ in range(10):
            q = random_quaternion()
            length = math.sqrt(q.scalar**2 + q.vector.dot())
            assert length == pytest.approx(1.0, abs=1e-5)

    def test_produces_different_values(self):
        """Two calls with different random state should generally differ."""
        q1 = random_quaternion()
        q2 = random_quaternion()
        # With overwhelming probability these will differ
        assert q1.scalar != q2.scalar or list(q1.vector) != list(q2.vector)
