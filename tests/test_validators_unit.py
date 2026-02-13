#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import attr
import magnum as mn
import numpy as np
import pytest

# Need to import quaternion library here despite it not being used or else importing
# habitat_sim below will cause an invalid free() when audio is enabled in sim compilation
import quaternion
import quaternion as qt

from habitat_sim.utils.validators import (
    NoAttrValidationContext,
    all_is_finite,
    is_unit_length,
    value_is_validated,
)

# Helpers: lightweight attrs classes to use as validator targets


@attr.s(auto_attribs=True)
class _FiniteHolder:
    value: np.ndarray = attr.ib(validator=all_is_finite)


@attr.s(auto_attribs=True)
class _UnitLengthVec:
    value: np.ndarray = attr.ib(validator=is_unit_length)


@attr.s(auto_attribs=True)
class _UnitLengthQuat:
    value: qt.quaternion = attr.ib(validator=is_unit_length)


@attr.s(auto_attribs=True)
class _UnitLengthMagnumQuat:
    value: mn.Quaternion = attr.ib(validator=is_unit_length)


@attr.s(auto_attribs=True)
class _Inner:
    x: int = attr.ib(validator=attr.validators.instance_of(int))


@attr.s(auto_attribs=True)
class _Outer:
    child: _Inner = attr.ib(validator=value_is_validated)


# -- all_is_finite tests --


class TestAllIsFinite:
    """Tests for the all_is_finite validator."""

    def test_finite_array_passes(self):
        _FiniteHolder(value=np.array([1.0, 2.0, 3.0]))

    def test_single_nan_raises(self):
        with pytest.raises(ValueError, match="non-finite"):
            _FiniteHolder(value=np.array([1.0, np.nan, 3.0]))

    def test_inf_raises(self):
        with pytest.raises(ValueError, match="non-finite"):
            _FiniteHolder(value=np.array([np.inf, 0.0]))

    def test_neg_inf_raises(self):
        with pytest.raises(ValueError, match="non-finite"):
            _FiniteHolder(value=np.array([-np.inf]))

    def test_all_nan_raises(self):
        with pytest.raises(ValueError, match="non-finite"):
            _FiniteHolder(value=np.array([np.nan, np.nan]))

    def test_zero_array_passes(self):
        _FiniteHolder(value=np.array([0.0, 0.0, 0.0]))

    def test_negative_values_pass(self):
        _FiniteHolder(value=np.array([-1.0, -999.0, -0.001]))

    def test_scalar_finite_passes(self):
        _FiniteHolder(value=np.array([42.0]))

    def test_empty_array_passes(self):
        """Empty array has all-finite vacuously."""
        _FiniteHolder(value=np.array([]))


# -- is_unit_length tests --


class TestIsUnitLength:
    """Tests for the is_unit_length validator."""

    # --- numpy vector branch ---

    def test_unit_vector_passes(self):
        _UnitLengthVec(value=np.array([1.0, 0.0, 0.0]))

    def test_diagonal_unit_vector_passes(self):
        v = np.array([1.0, 1.0, 1.0])
        v /= np.linalg.norm(v)
        _UnitLengthVec(value=v)

    def test_non_unit_vector_raises(self):
        with pytest.raises(ValueError, match="not a unit length"):
            _UnitLengthVec(value=np.array([2.0, 0.0, 0.0]))

    def test_zero_vector_raises(self):
        with pytest.raises((ValueError, AssertionError)):
            _UnitLengthVec(value=np.array([0.0, 0.0, 0.0]))

    # --- numpy-quaternion branch ---

    def test_normalized_quaternion_passes(self):
        q = qt.quaternion(1, 0, 0, 0)
        _UnitLengthQuat(value=q)

    def test_arbitrary_normalized_quaternion_passes(self):
        q = qt.quaternion(0.5, 0.5, 0.5, 0.5)
        _UnitLengthQuat(value=q)

    def test_non_unit_quaternion_raises(self):
        q = qt.quaternion(2, 0, 0, 0)
        with pytest.raises(ValueError, match="normalized quaternion"):
            _UnitLengthQuat(value=q)

    # --- magnum Quaternion branch ---

    def test_magnum_normalized_passes(self):
        q = mn.Quaternion.identity_init()
        _UnitLengthMagnumQuat(value=q)

    def test_magnum_rotation_passes(self):
        q = mn.Quaternion.rotation(mn.Deg(45.0), mn.Vector3.y_axis())
        _UnitLengthMagnumQuat(value=q)

    def test_magnum_non_normalized_raises(self):
        q = mn.Quaternion(mn.Vector3(0.0, 0.0, 0.0), 5.0)
        with pytest.raises(ValueError, match="normalized quaternion"):
            _UnitLengthMagnumQuat(value=q)


# -- NoAttrValidationContext tests --


class TestNoAttrValidationContext:
    """Tests for the NoAttrValidationContext context manager."""

    def test_validators_disabled_inside_context(self):
        """Inside the context, attr validators should not run."""
        with NoAttrValidationContext():
            assert not attr.get_run_validators()
            obj = _FiniteHolder(value=np.array([np.nan]))  # would normally raise
        assert obj.value[0] != obj.value[0]  # NaN != NaN

    def test_validators_restored_after_context(self):
        """After exiting the context, validators should be re-enabled."""
        assert attr.get_run_validators()
        with NoAttrValidationContext():
            pass
        assert attr.get_run_validators()

    def test_validators_run_outside_context(self):
        """Outside the context, validators still fire normally."""
        with pytest.raises(ValueError):
            _FiniteHolder(value=np.array([np.inf]))

    def test_nested_contexts_restore_correctly(self):
        """Nested contexts restore the original state on exit."""
        assert attr.get_run_validators()
        with NoAttrValidationContext():
            assert not attr.get_run_validators()
            with NoAttrValidationContext():
                assert not attr.get_run_validators()
            assert not attr.get_run_validators()
        assert attr.get_run_validators()

    def test_as_decorator(self):
        """NoAttrValidationContext can be used as a decorator."""

        @NoAttrValidationContext()
        def create_invalid():
            return _FiniteHolder(value=np.array([np.nan]))

        obj = create_invalid()
        assert obj.value[0] != obj.value[0]  # NaN check


# -- value_is_validated tests --


class TestValueIsValidated:
    """Tests for the value_is_validated validator."""

    def test_valid_child_passes(self):
        inner = _Inner(x=42)
        _Outer(child=inner)

    def test_invalid_child_raises(self):
        """If the child has invalid fields, value_is_validated should re-raise."""
        with NoAttrValidationContext():
            inner = _Inner(x="not_an_int")  # type: ignore[arg-type]
        with pytest.raises(TypeError):
            _Outer(child=inner)
