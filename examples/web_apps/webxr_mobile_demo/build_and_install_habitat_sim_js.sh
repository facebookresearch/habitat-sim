#!/bin/sh
cd ../../.. || { echo "Failed to navigate to habitat_sim directory"; exit 1; }
./build_js.sh --bullet
mkdir -p examples/web_apps/webxr_mobile_demo/lib/habitat-sim-js/
cp build_js/esp/bindings_js/hsim_bindings.js examples/web_apps/webxr_mobile_demo/lib/habitat-sim-js/
cp build_js/esp/bindings_js/hsim_bindings.wasm examples/web_apps/webxr_mobile_demo/lib/habitat-sim-js/
cp src/esp/bindings_js/modules/utils.js examples/web_apps/webxr_mobile_demo/lib/habitat-sim-js/
cp src/esp/bindings_js/modules/xr_utils.js examples/web_apps/webxr_mobile_demo/lib/habitat-sim-js/
cp src/esp/bindings_js/modules/ar_utils.js examples/web_apps/webxr_mobile_demo/lib/habitat-sim-js/
cd examples/web_apps/webxr_mobile_demo || { echo "Failed to navigate back to examples/web_apps/webxr_mobile_demo"; exit 1; }
