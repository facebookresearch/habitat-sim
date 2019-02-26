#!/usr/bin/env bash

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

git submodule update --recursive --init

if [ -f "build/CMakeCache.txt" ] ; then
  RUN_CMAKE=false
else
  RUN_CMAKE=true
fi


BUILD_GUI_VIEWERS=ON

builder_args=()
while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    --force-cmake)
      shift # past argument
      RUN_CMAKE=true
      ;;
    --headless)
      shift
      BUILD_GUI_VIEWERS=OFF
      ;;
    --run-tests)
      shift
      RUN_TESTS=true
      ;;
    *)    # Forward unknown args to builder
      shift
      builder_args+=("$key")
      ;;
esac
done

if [ -f "build/CMakeCache.txt" ] ; then
  current_value=$(cat "build/CMakeCache.txt" | grep BUILD_GUI_VIEWERS:BOOL)
  if ! [ "$current_value" = "BUILD_GUI_VIEWERS:BOOL=${BUILD_GUI_VIEWERS}" ] ; then
    RUN_CMAKE=true
  fi
fi



mkdir -p build
cd build

cmake_args=()
cmake_args+=('-DBUILD_TEST=1')
cmake_args+=('-DCMAKE_EXPORT_COMPILE_COMMANDS=ON')
cmake_args+=('-DBUILD_GUI_VIEWERS='${BUILD_GUI_VIEWERS})

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

# use ninja build if ninja is available
if hash ninja 2>/dev/null; then
  cmake_args+=('-GNinja')
fi

if [ "$RUN_CMAKE" = true ] ; then
  cmake "${cmake_args[@]}" \
    ../src
fi

if hash ninja 2>/dev/null; then
  ninja "${builder_args[@]}"
else
  make "${builder_args[@]}"
fi

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
