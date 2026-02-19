#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Legacy setup.py shim for backward compatibility.

The build system has been migrated to scikit-build-core and is configured
entirely in pyproject.toml. Use pip to build and install:

    pip install .                    # Standard install (GUI + Bullet enabled)
    pip install -e .                 # Editable/development install
    pip install . -v                 # Verbose build output

Build options are controlled via environment variables (showing non-default overrides):

    HABITAT_BUILD_GUI_VIEWERS=OFF pip install .   # headless (no GUI)
    HABITAT_WITH_BULLET=OFF pip install .         # disable Bullet physics
    HABITAT_WITH_CUDA=ON pip install .            # enable CUDA
    HABITAT_WITH_AUDIO=ON pip install .           # enable audio sensor
"""

import sys
import warnings

warnings.warn(
    "setup.py is deprecated. Use 'pip install .' or 'pip install -e .' instead. "
    "See pyproject.toml for build configuration.",
    DeprecationWarning,
    stacklevel=1,
)

if __name__ == "__main__":
    print(
        "ERROR: Direct setup.py invocation is no longer supported.\n"
        "\n"
        "This project now uses scikit-build-core (configured in pyproject.toml).\n"
        "Please use one of the following commands instead:\n"
        "\n"
        "  pip install .              # Standard install (GUI + Bullet enabled)\n"
        "  pip install -e .           # Editable/development install\n"
        "  pip install . -v           # Verbose output\n"
        "\n"
        "Build options (environment variables, showing non-default overrides):\n"
        "\n"
        "  HABITAT_BUILD_GUI_VIEWERS=OFF pip install .   # headless (no GUI)\n"
        "  HABITAT_WITH_BULLET=OFF pip install .         # disable Bullet physics\n"
        "  HABITAT_WITH_CUDA=ON pip install .            # enable CUDA\n"
        "  HABITAT_WITH_AUDIO=ON pip install .           # enable audio sensor\n",
        file=sys.stderr,
    )
    sys.exit(1)
