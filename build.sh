#!/usr/bin/env bash

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# Propagate failures properly
set -e

builder_args=()
while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    --run-tests)
      shift
      RUN_TESTS=true
      builder_args+=("--build-tests")
      ;;
    *)    # Forward unknown args to builder
      shift
      builder_args+=("$key")
      ;;
esac
done


# devfair/learnfair custom stuff: EGL path, and module loads
my_hostname=$(hostname)
if [[ $my_hostname =~ "fair" ]]; then
  module purge
  module load cuda/10.0
  module load cudnn/v7.4-cuda.10.0
  module load cmake/3.15.3/gcc.7.3.0
fi

python setup.py build_ext --inplace "${builder_args[@]}"

if [ "$RUN_TESTS" = true ] ; then
  cd build
  ctest -V
fi
