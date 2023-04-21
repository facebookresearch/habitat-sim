#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import math
from io import BytesIO
from typing import List, Sequence, Tuple, Union
from urllib.request import urlopen
from zipfile import ZipFile

import magnum as mn
import numpy as np
import quaternion as qt

from habitat_sim._ext.habitat_sim_bindings.core import (  # noqa: F401
    orthonormalize_rotation_shear,
)


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


def angle_between_quats(q1: qt.quaternion, q2: qt.quaternion) -> float:
    r"""Computes the angular distance between two quaternions

    :return: The angular distance between q1 and q2 in radians
    """

    q1_inv = np.conjugate(q1)
    dq = q1_inv * q2

    return 2 * np.arctan2(np.linalg.norm(dq.imag), np.abs(dq.real))


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


def download_and_unzip(file_url, local_directory):
    response = urlopen(file_url)
    zipfile = ZipFile(BytesIO(response.read()))
    zipfile.extractall(path=local_directory)


def colorize_ids(ids):
    out = np.zeros((ids.shape[0], ids.shape[1], 3), dtype=np.uint8)
    for i in range(ids.shape[0]):
        for j in range(ids.shape[1]):
            object_index = ids[i, j]
            if object_index >= 0:
                out[i, j] = d3_40_colors_rgb[object_index % 40]
    return out


d3_40_colors_rgb: np.ndarray = np.array(
    [
        [31, 119, 180],
        [174, 199, 232],
        [255, 127, 14],
        [255, 187, 120],
        [44, 160, 44],
        [152, 223, 138],
        [214, 39, 40],
        [255, 152, 150],
        [148, 103, 189],
        [197, 176, 213],
        [140, 86, 75],
        [196, 156, 148],
        [227, 119, 194],
        [247, 182, 210],
        [127, 127, 127],
        [199, 199, 199],
        [188, 189, 34],
        [219, 219, 141],
        [23, 190, 207],
        [158, 218, 229],
        [57, 59, 121],
        [82, 84, 163],
        [107, 110, 207],
        [156, 158, 222],
        [99, 121, 57],
        [140, 162, 82],
        [181, 207, 107],
        [206, 219, 156],
        [140, 109, 49],
        [189, 158, 57],
        [231, 186, 82],
        [231, 203, 148],
        [132, 60, 57],
        [173, 73, 74],
        [214, 97, 107],
        [231, 150, 156],
        [123, 65, 115],
        [165, 81, 148],
        [206, 109, 189],
        [222, 158, 214],
    ],
    dtype=np.uint8,
)


# [d3_40_colors_hex]
d3_40_colors_hex: List[str] = [
    "0x1f77b4",
    "0xaec7e8",
    "0xff7f0e",
    "0xffbb78",
    "0x2ca02c",
    "0x98df8a",
    "0xd62728",
    "0xff9896",
    "0x9467bd",
    "0xc5b0d5",
    "0x8c564b",
    "0xc49c94",
    "0xe377c2",
    "0xf7b6d2",
    "0x7f7f7f",
    "0xc7c7c7",
    "0xbcbd22",
    "0xdbdb8d",
    "0x17becf",
    "0x9edae5",
    "0x393b79",
    "0x5254a3",
    "0x6b6ecf",
    "0x9c9ede",
    "0x637939",
    "0x8ca252",
    "0xb5cf6b",
    "0xcedb9c",
    "0x8c6d31",
    "0xbd9e39",
    "0xe7ba52",
    "0xe7cb94",
    "0x843c39",
    "0xad494a",
    "0xd6616b",
    "0xe7969c",
    "0x7b4173",
    "0xa55194",
    "0xce6dbd",
    "0xde9ed6",
]
# [/d3_40_colors_hex]
