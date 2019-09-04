#!/usr/bin/env bash

# Propagate failures properly
set -e

# Build C++ docs first so the Python docs can make use of the tag file
./docs/m.css/documentation/doxygen.py docs/Doxyfile-mcss

./docs/m.css/documentation/python.py docs/conf.py
