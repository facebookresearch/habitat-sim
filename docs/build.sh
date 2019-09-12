#!/usr/bin/env bash

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

# Build C++ docs first so the Python docs can make use of the tag file
$mcss_path/documentation/doxygen.py Doxyfile-mcss

$mcss_path/documentation/python.py conf.py
