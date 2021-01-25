#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# contains validators for attrs
import attr
import magnum as mn
import numpy as np
import quaternion


def all_is_finite(instance, attribute, value) -> None:
    if not np.all(np.isfinite(value)):
        raise ValueError(
            f"{value} contains non-finite values which are forbidden in the {attribute} of {instance}"
        )


def is_unit_length(instance, attribute, value, tol=1e-5) -> None:
    if isinstance(value, mn.Quaternion):
        if not value.is_normalized():
            raise ValueError(
                f"""{value} is suppose to be a normalized quaternion but is not.
                    This is not valid for an {attribute} of {instance} which requires a unit length"""
            )
    elif isinstance(value, (quaternion.quaternion)):
        if not np.isclose(value.norm(), 1.0, rtol=tol, atol=0):
            raise ValueError(
                f"""{value} is suppose to be a normalized quaternion but is not {value.norm()}.
                This is not valid for an {attribute} of {instance} which requires a unit length"""
            )
    else:
        new_value = np.asarray(value)
        assert new_value.ndim == 1
        if not np.isclose(np.linalg.norm(new_value), 1.0, rtol=tol, atol=0):
            raise ValueError(
                f"{value} is not a unit length vector which is required for {attribute} of {instance}"
            )


def value_is_validated(instance, attribute, value) -> None:
    attr.validate(value)
