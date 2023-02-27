#!/bin/bash

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

set -ex

git clone https://github.com/NixOS/patchelf
cd patchelf
git checkout 0.17.2
sed -i 's/serial/parallel/g' configure.ac
./bootstrap.sh
./configure
make
make install
cd ..
rm -rf patchelf
