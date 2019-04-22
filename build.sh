#!/usr/bin/env bash

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

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
my_hostname=`hostname`
if [[ $my_hostname =~ "fair" ]]; then
  module purge
  module load anaconda3/5.0.1
  module load cuda/9.0
  module load cudnn/v7.0-cuda.9.0
  module load gcc/7.1.0
  module load cmake/3.10.1/gcc.5.4.0
fi

python setup.py build_ext --inplace "${builder_args[@]}"

if [ "$RUN_TESTS" = true ] ; then
  cd ..
  echo "Running tests..."
  TEST_SCRIPTS=$(find "build/tests" -type f -perm +111)
  declare -i RET_VAL=0
  for test_script in $TEST_SCRIPTS ; do
    echo "Running $test_script"
    $test_script
    RET_VAL+=$?
  done
  if [ "$RET_VAL" -ne 0 ] ; then
    echo "Some tests failed."
  else
    echo "All tests passed."
  fi
fi
