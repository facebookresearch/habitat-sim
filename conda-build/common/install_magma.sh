#!/bin/bash

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

function install_92 {
    # Install MAGMA for CUDA 9.2
    pushd /tmp || exit
    wget -q https://anaconda.org/pytorch/magma-cuda92/2.5.1/download/linux-64/magma-cuda92-2.5.1-1.tar.bz2
    tar -xvf magma-cuda92-2.5.1-1.tar.bz2
    mkdir -p /usr/local/cuda-9.2/magma
    mv include /usr/local/cuda-9.2/magma/include
    mv lib /usr/local/cuda-9.2/magma/lib
    rm -rf info lib include magma-cuda92-2.5.1-1.tar.bz2
}

function install_100 {
    # Install MAGMA for CUDA 10.0
    pushd /tmp || exit
    wget -q https://anaconda.org/pytorch/magma-cuda100/2.5.1/download/linux-64/magma-cuda100-2.5.1-1.tar.bz2
    tar -xvf magma-cuda100-2.5.1-1.tar.bz2
    mkdir -p /usr/local/cuda-10.0/magma
    mv include /usr/local/cuda-10.0/magma/include
    mv lib /usr/local/cuda-10.0/magma/lib
    rm -rf info lib include magma-cuda100-2.5.1-1.tar.bz2
}

function install_101 {
    # Install MAGMA for CUDA 10.1
    pushd /tmp || exit
    wget -q https://anaconda.org/pytorch/magma-cuda101/2.5.1/download/linux-64/magma-cuda101-2.5.1-1.tar.bz2
    tar -xvf magma-cuda101-2.5.1-1.tar.bz2
    mkdir -p /usr/local/cuda-10.1/magma
    mv include /usr/local/cuda-10.1/magma/include
    mv lib /usr/local/cuda-10.1/magma/lib
    rm -rf info lib include magma-cuda101-2.5.1-1.tar.bz2
}

# idiomatic parameter and option handling in sh
while test $# -gt 0
do
    case "$1" in
       9.2) install_92
          ;;
       10.0) install_100
          ;;
       10.1) install_101
          ;;
       *) echo "bad argument $1"; exit 1
          ;;
    esac
    shift
done
