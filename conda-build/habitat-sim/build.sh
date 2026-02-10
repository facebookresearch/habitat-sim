#!/bin/bash

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# Conda build script for habitat-sim (scikit-build-core).
#
# Build options are passed via environment variables that map to CMake defines
# in pyproject.toml's [tool.scikit-build.cmake.define] section.

set -e

# Work around CMake 4.x removing support for cmake_minimum_required < 3.5
# Some bundled dependencies (e.g., zlib in openexr) have old CMakeLists.txt
export CMAKE_POLICY_VERSION_MINIMUM=3.5

# ---- Map conda variant variables to scikit-build-core env vars ----

if [ "${HEADLESS}" == "1" ]; then
  export HABITAT_BUILD_GUI_VIEWERS=OFF
else
  export HABITAT_BUILD_GUI_VIEWERS=ON
fi

if [ "${WITH_BULLET}" == "1" ]; then
  export HABITAT_WITH_BULLET=ON
fi

if [ "${WITH_CUDA}" == "1" ]; then
  export HABITAT_WITH_CUDA=ON
  export CUDA_HOME=/public/apps/cuda/${CUDA_VER}
  export PATH=${CUDA_HOME}/bin:${PATH}
fi

if [ "${LTO}" == "1" ]; then
  export HABITAT_LTO=ON
fi

# ---- Platform-specific setup ----

if [ "$(uname)" == "Linux" ]; then
  cp -r /usr/include/EGL "${PREFIX}"/include/.
  cp -r /usr/include/X11 "${PREFIX}"/include/.
  export CMAKE_PREFIX_PATH=${PREFIX}:${CMAKE_PREFIX_PATH}
fi

# ---- Build and install via pip (scikit-build-core handles CMake) ----
# --no-build-isolation: use the conda host environment's packages
# Magnum Python bindings are now installed by CMake install() targets,
# so the separate `pip install build/deps/magnum-bindings/src/python` step
# is no longer needed.

${PYTHON} -m pip install . -v --no-build-isolation

# ---- RPATH fixups for conda relocatability ----
# Conda packages must be relocatable, so we set RPATHs to be relative
# ($ORIGIN on Linux, @loader_path on macOS).

pushd "${PREFIX}" || exit

corrade_bindings=$(find . -name "*_corrade*so" | head -1)
magnum_bindings=$(find . -name "*_magnum*so" | head -1)
hsim_bindings=$(find . -name "*habitat_sim_bindings*so" | head -1)

if [ -z "${hsim_bindings}" ]; then
  echo "ERROR: Could not find habitat_sim_bindings.so in ${PREFIX}"
  exit 1
fi

ext_folder=$(dirname "${hsim_bindings}")

echo "Found bindings:"
echo "  corrade: ${corrade_bindings}"
echo "  magnum:  ${magnum_bindings}"
echo "  hsim:    ${hsim_bindings}"
echo "  ext_dir: ${ext_folder}"

if [ "$(uname)" == "Darwin" ]; then
  if [ -n "${corrade_bindings}" ]; then
    install_name_tool -add_rpath @loader_path/habitat_sim/_ext "${corrade_bindings}"
  fi
  if [ -n "${magnum_bindings}" ]; then
    install_name_tool -add_rpath @loader_path/habitat_sim/_ext "${magnum_bindings}"
  fi

  if [ -f bin/viewer ]; then
    install_name_tool -add_rpath @loader_path/../"${ext_folder}" bin/viewer
  fi

  corrade_pkg_dir=$(dirname "${corrade_bindings}")/corrade
  if [ -d "${corrade_pkg_dir}" ]; then
    pushd "${corrade_pkg_dir}" || exit
      find . -name "*so" -print0 | xargs -I {} install_name_tool -add_rpath @loader_path/../habitat_sim/_ext {}
    popd || exit
  fi

elif [ "$(uname)" == "Linux" ]; then
  # Set RPATHs relative to each .so's location for relocatability
  if [ -n "${corrade_bindings}" ]; then
    patchelf --set-rpath "\$ORIGIN/habitat_sim/_ext:\$ORIGIN/../.." --force-rpath "${corrade_bindings}"
  fi
  if [ -n "${magnum_bindings}" ]; then
    patchelf --set-rpath "\$ORIGIN/habitat_sim/_ext:\$ORIGIN/../.." --force-rpath "${magnum_bindings}"
  fi

  patchelf --set-rpath "\$ORIGIN:\$ORIGIN/../../../.." --force-rpath "${hsim_bindings}"

  if [ -f 'bin/viewer' ]; then
    patchelf --set-rpath "\$ORIGIN/../${ext_folder}:\$ORIGIN/../lib" --force-rpath bin/viewer
  fi

  find "$(dirname "${hsim_bindings}")" -name "*Corrade*so" -print0 | xargs -I {} patchelf --set-rpath "\$ORIGIN:\$ORIGIN/../../../.." --force-rpath {}

  corrade_pkg_dir=$(dirname "${corrade_bindings}")/corrade
  if [ -d "${corrade_pkg_dir}" ]; then
    pushd "${corrade_pkg_dir}" || exit
      find . -name "*so" -print0 | xargs -I {} patchelf --set-rpath "\$ORIGIN/../habitat_sim/_ext:\$ORIGIN/../../.." --force-rpath {}
    popd || exit
  fi
fi

popd || exit
