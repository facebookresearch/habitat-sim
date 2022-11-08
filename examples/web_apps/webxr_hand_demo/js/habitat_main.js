// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/* global Module */

import {
  checkWebAssemblySupport,
  checkWebgl2Support,
  buildConfigFromURLParameters,
  preload
} from "../lib/habitat-sim-js/utils.js";
import { VRDemo } from "./vr_demo.js";
import { isWebXRSupported } from "../lib/habitat-sim-js/vr_utils.js";

function checkSupport() {
  const webgl2Support = checkWebgl2Support();
  let message = "";

  if (!webgl2Support) {
    message = "WebGL2 is not supported on your browser. ";
  } else if (webgl2Support === 1) {
    message = "WebGL2 is supported on your browser, but not enabled. ";
  }

  const webasmSupport = checkWebAssemblySupport();

  if (!webasmSupport) {
    message += "Web Assembly is not supported in your browser";
  }

  if (message.length > 0) {
    const warningElement = document.getElementById("warning");
    warningElement.innerHTML = message;
    // Remove the default hidden class
    warningElement.className = "";
  }
}

checkSupport();

let demo = new VRDemo();

function doPreloading() {
  demo.preloadFiles(preload);
}

Module.preRun.push(() => {
  let config = {};
  buildConfigFromURLParameters(config);
  Module.stageName =
    config.stage === undefined ? "remake_v0_JustBigStuff_00" : config.stage;
  doPreloading();
});

async function doRun() {
  if (isWebXRSupported()) {
    demo.start();
  } else {
    console.log(
      "WebXR not supported. Make sure you have the WebXR API Emulator chrome extension if you are not on a headset."
    );
  }
}

Module.onRuntimeInitialized = async function() {
  doRun();
};
