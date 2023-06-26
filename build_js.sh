#!/usr/bin/env bash

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# Propagate failures properly
set -e

BULLET=false
WEB_APPS=true

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --bullet) BULLET=true ;;
        --no-web-apps) WEB_APPS=false ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done
git submodule update --init --recursive

mkdir -p build_corrade-rc
pushd build_corrade-rc
cmake ../src \
    -DBUILD_GUI_VIEWERS=OFF \
    -DBUILD_PYTHON_BINDINGS=OFF \
    -DBUILD_ASSIMP_SUPPORT=OFF \
    -DBUILD_DATATOOL=OFF
cmake --build . --target corrade-rc --
popd

mkdir -p build_js
cd build_js


EXE_LINKER_FLAGS="-s USE_WEBGL2=1"
cmake ../src \
    -DCORRADE_RC_EXECUTABLE=../build_corrade-rc/RelWithDebInfo/bin/corrade-rc \
    -DBUILD_GUI_VIEWERS="$( if ${WEB_APPS} ; then echo ON ; else echo OFF; fi )" \
    -DBUILD_PYTHON_BINDINGS=OFF \
    -DBUILD_ASSIMP_SUPPORT=OFF \
    -DBUILD_DATATOOL=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$EMSCRIPTEN" \
    -DCMAKE_TOOLCHAIN_FILE="../src/deps/corrade/toolchains/generic/Emscripten-wasm.cmake" \
    -DCMAKE_INSTALL_PREFIX="." \
    -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=ON \
    -DCMAKE_CXX_FLAGS="-s FORCE_FILESYSTEM=1 -s ALLOW_MEMORY_GROWTH=1 -s ASSERTIONS=0" \
    -DCMAKE_EXE_LINKER_FLAGS="${EXE_LINKER_FLAGS}" \
    -DBUILD_WITH_BULLET="$( if ${BULLET} ; then echo ON ; else echo OFF; fi )" \
    -DBUILD_WEB_APPS="$( if ${WEB_APPS} ; then echo ON ; else echo OFF; fi )"

cmake --build . -- -j 8 #TODO: Set to 8 cores only on CirelcCI
echo "Done building."

if [ -o ${WEB_APPS} ]
  then
    cmake --build . --target install -- -j 8
    echo "Run:"
    echo "python2 -m SimpleHTTPServer 8000"
    echo "Or:"
    echo "python3 -m http.server"
    echo "Then open in a browser:"
    echo "http://0.0.0.0:8000/build_js/esp/bindings_js/bindings.html?scene=skokloster-castle.glb"
fi
