#!/bin/bash

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

set -ex

# Anaconda
wget -q https://repo.continuum.io/miniconda/Miniconda3-latest-Linux-x86_64.sh
chmod +x  Miniconda3-latest-Linux-x86_64.sh
./Miniconda3-latest-Linux-x86_64.sh -b -p /opt/conda
rm Miniconda3-latest-Linux-x86_64.sh
export PATH=/opt/conda/bin:$PATH
# Configure conda to use conda-forge channel to avoid Anaconda ToS requirement
# Anaconda now requires explicit Terms of Service acceptance for default channels
# (pkgs/main, pkgs/r) in non-interactive CI environments. Use conda-forge instead.
# Must remove specific default channels explicitly - removing 'defaults' alone is insufficient
conda config --remove channels defaults || true
conda config --remove channels pkgs/main || true
conda config --remove channels pkgs/r || true
conda config --add channels conda-forge
conda config --set channel_priority flexible
conda install -y anaconda-client git gitpython ninja conda-build # conda-build=3.18.9 # last version that works with our setup
conda remove -y --force patchelf
