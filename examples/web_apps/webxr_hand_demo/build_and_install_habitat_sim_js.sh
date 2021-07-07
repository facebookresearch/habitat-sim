#!/bin/sh
../../../build_js.sh --bullet
mkdir -p lib/habitat-sim-js/
cp ../../../build_js/esp/bindings_js/hsim_bindings.js lib/habitat-sim-js/
cp ../../../build_js/esp/bindings_js/hsim_bindings.wasm lib/habitat-sim-js/
cp ../../../src/esp/bindings_js/modules/utils.js lib/habitat-sim-js/
cp ../../../src/esp/bindings_js/modules/vr_utils.js lib/habitat-sim-js/
