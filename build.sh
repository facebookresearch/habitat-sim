#!/usr/bin/env bash

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# Build habitat-sim using scikit-build-core.
#
# Usage:
#   ./build.sh                      # Default build (GUI + Bullet)
#   ./build.sh --with-bullet        # Build with Bullet physics
#   ./build.sh --with-cuda          # Build with CUDA support
#   ./build.sh --with-audio         # Build with audio support
#   ./build.sh --run-tests          # Build and run tests
#   ./build.sh --debug              # Debug build
#   ./build.sh --lto                # Link Time Optimization
#
# Build options are passed via environment variables to scikit-build-core.
# You can also set them directly:
#   HABITAT_WITH_BULLET=ON HABITAT_WITH_CUDA=ON ./build.sh

set -e

# Default build options (can be overridden by env vars)
: "${HABITAT_BUILD_GUI_VIEWERS:=ON}"
: "${HABITAT_WITH_BULLET:=ON}"
: "${HABITAT_WITH_CUDA:=OFF}"
: "${HABITAT_WITH_AUDIO:=OFF}"
: "${HABITAT_BUILD_TEST:=OFF}"
: "${HABITAT_LTO:=OFF}"

RUN_TESTS=false
PIP_ARGS=()

# Parse arguments
for arg in "$@"; do
    case $arg in
        --headless)
            export HABITAT_BUILD_GUI_VIEWERS=OFF
            ;;
        --gui|--with-gui)
            export HABITAT_BUILD_GUI_VIEWERS=ON
            ;;
        --no-bullet)
            export HABITAT_WITH_BULLET=OFF
            ;;
        --with-bullet|--bullet)
            export HABITAT_WITH_BULLET=ON
            ;;
        --with-cuda)
            export HABITAT_WITH_CUDA=ON
            ;;
        --with-audio|--audio)
            export HABITAT_WITH_AUDIO=ON
            ;;
        --run-tests|--build-tests)
            export HABITAT_BUILD_TEST=ON
            RUN_TESTS=true
            ;;
        --debug)
            PIP_ARGS+=("--config-settings=cmake.build-type=Debug")
            ;;
        --lto)
            export HABITAT_LTO=ON
            ;;
        -v|--verbose)
            PIP_ARGS+=("-v")
            ;;
        *)
            echo "Unknown argument: $arg"
            echo "Usage: ./build.sh [--headless] [--gui] [--no-bullet] [--with-bullet] [--with-cuda] [--with-audio] [--run-tests] [--debug] [--lto] [-v]"
            exit 1
            ;;
    esac
done

echo "=== habitat-sim build ==="
echo "  GUI viewers: ${HABITAT_BUILD_GUI_VIEWERS}"
echo "  Bullet:      ${HABITAT_WITH_BULLET}"
echo "  CUDA:        ${HABITAT_WITH_CUDA}"
echo "  Audio:       ${HABITAT_WITH_AUDIO}"
echo "  Tests:       ${HABITAT_BUILD_TEST}"
echo "  LTO:         ${HABITAT_LTO}"
echo ""

pip install -e . --no-build-isolation "${PIP_ARGS[@]}"

if [ "$RUN_TESTS" = true ]; then
    echo ""
    echo "=== Running tests ==="
    # Find the scikit-build-core build directory
    BUILD_DIR=$(find build -maxdepth 1 -name "cp*" -type d | head -1)
    if [ -n "$BUILD_DIR" ]; then
        cd "$BUILD_DIR"
        ctest -V
    else
        echo "Warning: Could not find build directory for ctest."
        echo "Running Python tests instead..."
        python -m pytest tests/ -v --timeout=120
    fi
fi
