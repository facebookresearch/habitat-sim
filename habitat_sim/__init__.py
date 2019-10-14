#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import builtins

__version__ = "0.1.3"

if not getattr(builtins, "__HSIM_SETUP__", False):
    # TODO: all those import * should get removed, kept only for compatibility
    #   with existing code
    from habitat_sim.nav import *
    from habitat_sim.agent import *
    from habitat_sim.simulator import *
    from habitat_sim.bindings import *

    from habitat_sim import (
        agent,
        geo,
        gfx,
        logging,
        nav,
        scene,
        sensor,
        simulator,
        utils,
    )
    from habitat_sim._ext.habitat_sim_bindings import MapStringString
    from habitat_sim.registry import registry

    __all__ = [
        "agent",
        "nav",
        "sensors",
        "errors",
        "geo",
        "gfx",
        "logging",
        "nav",
        "scene",
        "sensor",
        "simulator",
        "utils",
        "MapStringString",
        "registry",
    ]
