#!/bin/bash

export HSIM_SOURCE_PATH=$(pwd)/..

if [ $(uname) = "Darwin" ]; then
  export CC=clang
  export CXX=clang++
fi

export HEADLESS=0
export WITH_CUDA=0
export WITH_BULLET=0

py_ver="3.6"
if [ $(uname) = "Darwin" ]; then
  output_folder="py${py_ver}_osx"
fi


conda build \
  --python ${py_ver} \
  --variants "{python: [${py_ver}]}" \
  --channel conda-forge \
  --no-test \
  --no-anaconda-upload \
  --output-folder "$output_folder" \
  habitat-sim

