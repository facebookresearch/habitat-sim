#!/usr/bin/env bash

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# Detect the platform
echo "Installing dependencies. You might be prompted for your sudo password."
OS="$(uname)"
case $OS in
  'Linux')
    sudo apt-get install -y libomp-dev libx11-dev
    ;;
  'WindowsNT')
    OS='Windows'
    ;;
  'Darwin')
    brew install libomp
    ;;
  *) ;;
esac
