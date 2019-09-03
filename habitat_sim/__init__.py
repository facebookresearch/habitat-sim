#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import builtins

__version__ = "0.1.2"

if not getattr(builtins, "__HSIM_SETUP__", False):
    # TODO: all those import * should get removed, kept only for compatibility
    #   with existing code
    from .nav import *
    from .agent import *
    from .simulator import *
    from .bindings import *

    from . import agent, geo, gfx, logging, nav, scene, sensor, simulator, utils
    from ._ext.habitat_sim_bindings import MapStringString

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
    ]
