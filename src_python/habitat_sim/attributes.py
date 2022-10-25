# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Attributes objects store metadata relevant to a specific type of simulation objects for programmatic manipulation and instantiation (e.g. a blueprint).

Note: SceneDatasetAttributes and SceneInstanceAttributes are not publicly exposed.
"""


from habitat_sim._ext.habitat_sim_bindings import (
    CapsulePrimitiveAttributes,
    ConePrimitiveAttributes,
    CubePrimitiveAttributes,
    CylinderPrimitiveAttributes,
    IcospherePrimitiveAttributes,
    ObjectAttributes,
    PhysicsManagerAttributes,
    StageAttributes,
    UVSpherePrimitiveAttributes,
)

__all__ = [
    "CapsulePrimitiveAttributes",
    "ConePrimitiveAttributes",
    "CubePrimitiveAttributes",
    "CylinderPrimitiveAttributes",
    "IcospherePrimitiveAttributes",
    "ObjectAttributes",
    "PhysicsManagerAttributes",
    "StageAttributes",
    "UVSpherePrimitiveAttributes",
]
