#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

"""
Legacy setup.py shim for backward compatibility.

The build system has been migrated to scikit-build-core and is configured
entirely in pyproject.toml. Use pip to build and install:

    pip install .                    # Standard install
    pip install -e .                 # Editable/development install
    pip install . -v                 # Verbose build output

Build options are controlled via environment variables:

    HABITAT_WITH_BULLET=ON pip install .
    HABITAT_WITH_CUDA=ON pip install .
    HABITAT_BUILD_GUI_VIEWERS=ON pip install .
    HABITAT_WITH_AUDIO=ON pip install .

Or via pip's --config-settings:

    pip install . --config-settings=cmake.define.BUILD_WITH_BULLET=ON
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
        "  pip install .              # Standard install\n"
        "  pip install -e .           # Editable/development install\n"
        "  pip install . -v           # Verbose output\n"
        "\n"
        "Build options (environment variables):\n"
        "\n"
        "  HABITAT_WITH_BULLET=ON pip install .\n"
        "  HABITAT_WITH_CUDA=ON pip install .\n"
        "  HABITAT_BUILD_GUI_VIEWERS=ON pip install .\n"
        "  HABITAT_WITH_AUDIO=ON pip install .\n",
        file=sys.stderr,
    )
    sys.exit(1)
