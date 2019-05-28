#!/usr/bin/env bash

git submodule update --init --recursive

DATA_DIR="$(pwd)/data/"

mkdir -p build_js
cd build_js

cmake ../src \
    -DBUILD_WEBGL=ON \
    -DBUILD_GUI_VIEWERS=ON \
    -DBUILD_PYTHON_BINDINGS=OFF \
    -DBUILD_DATATOOL=OFF \
    -DBUILD_PTEX_SUPPORT=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$EMSCRIPTEN" \
    -DTARGET_GLES2=OFF \
    -DTARGET_WEBGL=ON \
    -DCMAKE_TOOLCHAIN_FILE="../src/deps/corrade/toolchains/generic/Emscripten-wasm.cmake" \
    -DCMAKE_INSTALL_PREFIX="." \
    -DCMAKE_CXX_FLAGS="-s FORCE_FILESYSTEM=1 -s TOTAL_MEMORY=1677721600 --preload-file $DATA_DIR/scene_datasets/habitat-test-scenes@/" \

cmake --build . -- -j 4
cmake --build . --target install -- -j 4

echo "Done building."
echo "Run:"
echo "python -m http.server"
echo "Then open in browser:"
echo "http://0.0.0.0:8000/build_js/utils/viewer/viewer.html?scene=skokloster-castle.glb"
