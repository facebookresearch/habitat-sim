#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import sys
import os.path as osp
import os

_build_folder = osp.abspath(
    osp.join(osp.dirname(__file__), os.pardir, os.pardir, "build/esp/bindings/")
)
sys.path = [_build_folder] + sys.path


def _has_build_folder():
    if not osp.exists(_build_folder):
        return """Could not find the expected bindings build folder: \"{}\"
Make sure to not delete the build folder after building
        """.format(
            _build_folder
        )

    else:
        return ""


def _check_wrong_python():
    found_libs = []
    if osp.exists(_build_folder):
        for root, dirs, files in os.walk(_build_folder):
            for f in files:
                if "habitat_sim_bindings" in f:
                    found_libs.append(f)

    if len(found_libs) == 0:
        return ""

    import re

    version_matcher = re.compile(
        r"""
            habitat_sim_bindings\. # Match start
            \w+?- # this will probably just be cython
            (?P<v>\d+?)m # Match version
            .* # Rest of string
        """,
        re.VERBOSE,
    )

    msg = "Found the following libraries"
    msg += "\n"
    for lib in found_libs:
        matches = version_matcher.match(lib)
        if matches is not None:
            version = matches.group("v")
        else:
            version = "Unknown"

        msg += "\n Library: {} -- Supported Python Version: {}".format(lib, version)

    msg += "\n\nYour python interpreter is version {}{}".format(
        sys.version_info[0], sys.version_info[1]
    )
    msg += "\nPlease re-build habitat sim with this version of python"

    return msg


try:
    import habitat_sim_bindings
except ImportError:
    msg = """
Failed to to import habitat sim bindings in developer mode
----------------------------------------------------------

"""

    msg += _has_build_folder()
    msg += _check_wrong_python()

    raise ImportError(msg)

from habitat_sim.bindings.modules import modules

exec("from habitat_sim_bindings import ({})".format(", ".join(modules)))
from habitat_sim_bindings import Simulator as SimulatorBackend

__all__ = ["SimulatorBackend"] + modules
