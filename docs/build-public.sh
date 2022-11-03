#!/usr/bin/env bash

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# Propagate failures properly
set -e

mcss_path=./m.css

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
$mcss_path/documentation/doxygen.py Doxyfile-public

mv ../build/docs-public/habitat-sim/annotated.html ../build/docs-public/habitat-sim/cpp.html

$mcss_path/documentation/python.py conf-public.py

# The file:// URLs are usually clickable in the terminal, directly opening a
# browser
echo "------------------------------------------------------------------------"
echo "Public docs were successfully generated to the following location. Note"
echo "that the search functionality requires a web server in this case."
echo
echo "file://$(pwd)/../build/docs-public/habitat-sim/index.html"
