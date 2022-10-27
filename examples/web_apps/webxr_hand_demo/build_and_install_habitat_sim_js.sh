#!/bin/sh

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

cd ../../.. || { echo "Failed to navigate to habitat_sim directory"; exit 1; }
./build_js.sh --bullet
mkdir -p examples/web_apps/webxr_hand_demo/lib/habitat-sim-js/
cp build_js/esp/bindings_js/hsim_bindings.js examples/web_apps/webxr_hand_demo/js/
cp build_js/esp/bindings_js/hsim_bindings.wasm examples/web_apps/webxr_hand_demo/js/
cp src/esp/bindings_js/modules/utils.js examples/web_apps/webxr_hand_demo/lib/habitat-sim-js/
cp src/esp/bindings_js/modules/vr_utils.js examples/web_apps/webxr_hand_demo/lib/habitat-sim-js/
cd examples/web_apps/webxr_hand_demo || { echo "Failed to navigate back to examples/web_apps/webxr_hand_demo"; exit 1; }
