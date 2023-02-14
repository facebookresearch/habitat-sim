#!/bin/bash

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

set -ex

# MKL
mkdir -p /opt/intel/lib
pushd /tmp
wget -q https://anaconda.org/intel/mkl-static/2019.4/download/linux-64/mkl-static-2019.4-intel_243.tar.bz2
tar -xvf mkl-static-2019.4-intel_243.tar.bz2
cp lib/* /opt/intel/lib/
rm -rf ./*
wget -q https://anaconda.org/intel/mkl-include/2019.4/download/linux-64/mkl-include-2019.4-intel_243.tar.bz2
tar -xvf mkl-include-2019.4-intel_243.tar.bz2
mv include /opt/intel/
rm -rf ./*
