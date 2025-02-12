#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import math
from typing import Sequence, Tuple, Union

import magnum as mn
import numpy as np
import quaternion as qt


def quat_from_coeffs(coeffs: Union[Sequence[float], np.ndarray]) -> qt.quaternion:
    r"""Creates a quaternion from the coeffs returned by the simulator backend

    :param coeffs: Coefficients of a quaternion in :py:`[b, c, d, a]` format,
        where :math:`q = a + bi + cj + dk`
    :return: A quaternion from the coeffs
    """
    quat = qt.quaternion(1, 0, 0, 0)
    quat.real = coeffs[3]
    quat.imag = coeffs[0:3]
    return quat


def quat_to_coeffs(quat: qt.quaternion) -> np.ndarray:
    r"""Converts a quaternion into the coeffs format the backend expects

    :param quat: The quaternion
    :return: Coefficients of a quaternion in :py:`[b, c, d, a]` format,
        where :math:`q = a + bi + cj + dk`
    """
    coeffs = np.empty(4)
    coeffs[0:3] = quat.imag
    coeffs[3] = quat.real
    return coeffs


def quat_to_magnum(quat: qt.quaternion) -> mn.Quaternion:
    return mn.Quaternion(quat.imag, quat.real)


def quat_from_magnum(quat: mn.Quaternion) -> qt.quaternion:
    a = qt.quaternion(1, 0, 0, 0)
    a.real = quat.scalar
    a.imag = quat.vector
    return a


def quat_to_angle_axis(quat: qt.quaternion) -> Tuple[float, np.ndarray]:
    r"""Converts a quaternion to angle axis format

    :param quat: The quaternion
    :return:
        -   `float` --- The angle to rotate about the axis by
        -   `numpy.ndarray` --- The axis to rotate about. If :math:`\theta = 0`,
            then this is hardcoded to be the +x axis
    """

    rot_vec = qt.as_rotation_vector(quat)

    theta = float(np.linalg.norm(rot_vec))
    if np.abs(theta) < 1e-5:
        w = np.array([1, 0, 0])
        theta = 0.0
    else:
        w = rot_vec / theta

    return (theta, w)


def quat_from_angle_axis(theta: float, axis: np.ndarray) -> qt.quaternion:
    r"""Creates a quaternion from angle axis format

    :param theta: The angle to rotate about the axis by
    :param axis: The axis to rotate about
    :return: The quaternion
    """
    axis = axis.astype(float)
    axis /= np.linalg.norm(axis)
    return qt.from_rotation_vector(theta * axis)


def quat_from_two_vectors(v0: np.ndarray, v1: np.ndarray) -> qt.quaternion:
    r"""Creates a quaternion that rotates the first vector onto the second vector

    :param v0: The starting vector, does not need to be a unit vector
    :param v1: The end vector, does not need to be a unit vector
    :return: The quaternion

    Calculates the quaternion q such that

    .. code:: py

        v1 = quat_rotate_vector(q, v0)
    """

    v0 = v0 / np.linalg.norm(v0)
    v1 = v1 / np.linalg.norm(v1)
    c = v0.dot(v1)
    if c < (-1 + 1e-8):
        c = max(c, -1)
        m = np.stack([v0, v1], 0)
        _, _, vh = np.linalg.svd(m, full_matrices=True)
        axis = vh[2]
        w2 = (1 + c) * 0.5
        w = np.sqrt(w2)
        axis = axis * np.sqrt(1 - w2)
        return qt.quaternion(w, *axis)

    axis = np.cross(v0, v1)
    s = np.sqrt((1 + c) * 2)
    return qt.quaternion(s * 0.5, *(axis / s))


def angle_between_quats(q1: mn.Quaternion, q2: mn.Quaternion) -> float:
    r"""Computes the angular distance between two magnum quaternions

    :return: The angular distance between q1 and q2 in radians
    """

    dq = q1.inverted() * q2

    return 2 * np.arctan2(dq.vector.length(), np.abs(dq.scalar))


def quat_rotate_vector(q: qt.quaternion, v: np.ndarray) -> np.ndarray:
    r"""Helper function to rotate a vector by a quaternion

    :param q: The quaternion to rotate the vector with
    :param v: The vector to rotate
    :return: The rotated vector

    Does

    .. code:: py

        v = (q * qt.quaternion(0, *v) * q.inverse()).imag
    """

    vq = qt.quaternion(0, 0, 0, 0)
    vq.imag = v
    return (q * vq * q.inverse()).imag


def random_quaternion():
    r"""Convenience function to sample a random Magnum::Quaternion.
    See http://planning.cs.uiuc.edu/node198.html.
    """
    u = np.random.rand(3)
    qAxis = np.array(
        [
            math.sqrt(1 - u[0]) * math.cos(2 * math.pi * u[1]),
            math.sqrt(u[0]) * math.sin(2 * math.pi * u[2]),
            math.sqrt(u[0]) * math.cos(2 * math.pi * u[2]),
        ]
    )
    return mn.Quaternion(qAxis, math.sqrt(1 - u[0]) * math.sin(2 * math.pi * u[1]))
