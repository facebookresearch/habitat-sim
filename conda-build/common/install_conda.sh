#!/bin/bash

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

set -ex

# Anaconda — detect architecture for the correct installer
ARCH=$(uname -m)
if [ "$ARCH" = "aarch64" ]; then
    MINICONDA_INSTALLER="Miniconda3-latest-Linux-aarch64.sh"
elif [ "$ARCH" = "x86_64" ]; then
    MINICONDA_INSTALLER="Miniconda3-latest-Linux-x86_64.sh"
else
    echo "Unsupported architecture: $ARCH" >&2
    exit 1
fi

wget -q "https://repo.continuum.io/miniconda/${MINICONDA_INSTALLER}"
chmod +x "${MINICONDA_INSTALLER}"
./"${MINICONDA_INSTALLER}" -b -p /opt/conda
rm "${MINICONDA_INSTALLER}"
export PATH=/opt/conda/bin:$PATH
conda install -y anaconda-client git gitpython ninja conda-build # conda-build=3.18.9 # last version that works with our setup
conda remove -y --force patchelf
