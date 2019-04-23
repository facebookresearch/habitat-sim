#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from habitat_sim.bindings.modules import modules
from habitat_sim.bindings.mode import use_dev_bindings

if use_dev_bindings:
    from .dev_bindings import *
else:
    from habitat_sim._ext.habitat_sim_bindings import Simulator as SimulatorBackend

    exec(
        "from habitat_sim._ext.habitat_sim_bindings import ({})".format(
            ", ".join(modules)
        )
    )

__all__ = ["SimulatorBackend"] + modules
