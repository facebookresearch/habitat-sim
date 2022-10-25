#!/usr/bin/env bash

# Copyright (c) Meta Platforms, Inc. and its affiliates.
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

here=$(pwd)
if [ "$RUN_TESTS" = true ] ; then
  cd build
  PYTHONPATH=${here}/src_python ctest -V
fi

# Check if src_python has been added to python path, otherwise remind user
if [[ "$PYTHONPATH" == *"src_python"* ]]; then
  echo "\`src_python\` subdir found in PYTHONPATH : \`$PYTHONPATH\`"
else
  echo "Add src_python to PYTHONPATH, i.e. \`export PYTHONPATH=${here}/src_python:\${PYTHONPATH}\`"
fi
