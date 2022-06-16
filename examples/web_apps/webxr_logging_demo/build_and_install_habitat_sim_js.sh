#!/bin/sh
cd ../../.. || { echo "Failed to navigate to habitat_sim directory"; exit 1; }
./build_js.sh --no-web-apps
cp build_js/esp/bindings_js/hsim_bindings.js examples/web_apps/webxr_logging_demo/task/server_files/habitat_vr_app/lib/habitat-sim-js/
cp build_js/esp/bindings_js/hsim_bindings.wasm examples/web_apps/webxr_logging_demo/task/server_files/habitat_vr_app/lib/habitat-sim-js/
cd examples/web_apps/webxr_logging_demo || { echo "Failed to navigate back to examples/web_apps/webxr_logging_demo"; exit 1; }
