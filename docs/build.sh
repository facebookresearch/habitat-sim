#!/usr/bin/env bash

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# Propagate failures properly
set -e

if [[ $# -eq 1 ]]; then
  export mcss_path=$1
elif [[ $# -ne 0 ]]; then
  echo "usage: ./build.sh [path-to-m.css]"
  exit 1
else
  if [ ! -d m.css ]; then
    echo "m.css submodule not found, please run git submodule update --init or specify the path to it"
    exit 1
  fi
  mcss_path=./m.css
fi

# Regenerate the compiled CSS file
$mcss_path/css/postprocess.py \
  theme.css \
  $mcss_path/css/m-grid.css \
  $mcss_path/css/m-components.css \
  $mcss_path/css/m-layout.css \
  pygments-pastie.css \
  $mcss_path/css/pygments-console.css \
  $mcss_path/css/m-documentation.css \
  -o theme.compiled.css

# Build C++ docs first so the Python docs can make use of the tag file
$mcss_path/documentation/doxygen.py Doxyfile-mcss

mv ../build/docs/habitat-sim/annotated.html ../build/docs/habitat-sim/cpp.html

$mcss_path/documentation/python.py conf.py

# The file:// URLs are usually clickable in the terminal, directly opening a
# browser
echo "------------------------------------------------------------------------"
echo "Docs were successfully generated. Open the following link to view them:"
echo
echo "file://$(pwd)/../build/docs/habitat-sim/index.html"
