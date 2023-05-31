# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Encapsulates global geometry utilities.
"""

from habitat_sim._ext.habitat_sim_bindings import OBB, BBox, Ray
from habitat_sim._ext.habitat_sim_bindings.geo import (
    BACK,
    FRONT,
    GRAVITY,
    LEFT,
    RIGHT,
    UP,
    build_catmull_rom_spline,
    compute_gravity_aligned_MOBB,
    get_transformed_bb,
)

__all__ = [
    "BBox",
    "OBB",
    "UP",
    "GRAVITY",
    "FRONT",
    "BACK",
    "LEFT",
    "RIGHT",
    "build_catmull_rom_spline",
    "compute_gravity_aligned_MOBB",
    "get_transformed_bb",
    "Ray",
]
