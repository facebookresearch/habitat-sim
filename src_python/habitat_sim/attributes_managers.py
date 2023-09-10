# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Each AttributesManager acts as a library of Attributes objects of a specific type, governing access and supporting import from config files.

Notes: SceneDataset and SceneInstance managers can be accessed from MetadataMediator and Simulator APIs, but are not publicly exposed.
"""

from habitat_sim._ext.habitat_sim_bindings import (
    AOAttributesManager,
    AssetAttributesManager,
    ObjectAttributesManager,
    PbrShaderAttributesManager,
    PhysicsAttributesManager,
    StageAttributesManager,
)

__all__ = [
    "AOAttributesManager",
    "AssetAttributesManager",
    "ObjectAttributesManager",
    "PbrShaderAttributesManager",
    "PhysicsAttributesManager",
    "StageAttributesManager",
]
