#!/usr/bin/env bash

# Propagate failures properly
set -e

# Build C++ docs first so the Python docs can make use of the tag file
./m.css/documentation/doxygen.py docs/Doxyfile-mcss

./m.css/documentation/python.py docs/conf.py
