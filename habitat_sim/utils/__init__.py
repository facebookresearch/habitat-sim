#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# quat_from_angle_axis and quat_rotate_vector imports
# added for backward compatibility with Habitat-API
# TODO @maksymets: remove after habitat-api/examples/new_actions.py will be
# fixed


from habitat_sim.utils import common
from habitat_sim.utils.common import quat_from_angle_axis, quat_rotate_vector

__all__ = ["quat_from_angle_axis", "quat_rotate_vector", "common"]
