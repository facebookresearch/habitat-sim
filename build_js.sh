#!/usr/bin/env bash

# Propagate failures properly
set -e

git submodule update --init --recursive

DATA_DIR="$(pwd)/data/"

mkdir -p build_js
cd build_js

cmake ../src \
    -DBUILD_GUI_VIEWERS=ON \
    -DBUILD_PYTHON_BINDINGS=OFF \
    -DBUILD_ASSIMP_SUPPORT=OFF \
    -DBUILD_DATATOOL=OFF \
    -DBUILD_PTEX_SUPPORT=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$EMSCRIPTEN" \
    -DCMAKE_TOOLCHAIN_FILE="../src/deps/corrade/toolchains/generic/Emscripten-wasm.cmake" \
    -DCMAKE_INSTALL_PREFIX="." \
    -DCMAKE_CXX_FLAGS="-s FORCE_FILESYSTEM=1 -s ALLOW_MEMORY_GROWTH=1 --preload-file $DATA_DIR/scene_datasets/habitat-test-scenes@/" \

cmake --build . -- -j 4
cmake --build . --target install -- -j 4

echo "Done building."
echo "Run:"
echo "python2 -m SimpleHTTPServer 8000"
echo "Or:"
echo "python3 -m http.server"
echo "Then open in browser:"
echo "http://0.0.0.0:8000/build_js/utils/viewer/viewer.html?scene=skokloster-castle.glb"
