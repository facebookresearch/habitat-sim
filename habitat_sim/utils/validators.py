#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# contains validators for attrs
import numpy as np


def all_is_finite(instance, attribute, value):
    if not np.all(np.isfinite(value)):
        raise ValueError(
            f"{value} contains NaN which are not valid in this context for the {attribute} of {instance}"
        )
