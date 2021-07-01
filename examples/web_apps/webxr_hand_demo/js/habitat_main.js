// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/* global FS, Module */

import {
  checkWebAssemblySupport,
  checkWebgl2Support,
  buildConfigFromURLParameters
} from "../lib/utils/utils.js";
import { VRDemo } from "./vr_demo.js";

console.log("index.js loading...");

// todo: move to habitat utils.js
function preload(url) {
  let file_parents_str = "";
  const splits = url.split("/");
  let file = splits[splits.length - 1];
  if (url.indexOf("http") === -1) {
    let file_parents = splits.slice(0, splits.length - 1);
    for (let i = 0; i < splits.length - 1; i += 1) {
      file_parents_str += file_parents[i];
      if (!FS.analyzePath(file_parents_str).exists) {
        FS.mkdir(file_parents_str, 777);
      }
      file_parents_str += "/";
    }
  }
  FS.createPreloadedFile(file_parents_str, file, url, true, false);
  return file_parents_str + file;
}

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

function doPreloading() {
  VRDemo.preloadFiles(preload);
}

Module.preRun.push(() => {
  console.log("preRun");
  let config = {};
  buildConfigFromURLParameters(config);
  Module.stageName = (config.stage === undefined) ? "remake_v0_JustBigStuff_00" : config.stage;
  doPreloading();
});

async function doRun() {
  console.log("hsim_bindings initialized");
  let demo;
  // todo: move this vr-specific logic out of here
  const supported = await navigator.xr.isSessionSupported("immersive-vr");
  if (supported) {
    console.log("WebXR is supported");
    demo = new VRDemo();
    demo.display();
  } else {
    // todo: better error message
    console.log("WebXR not supported");
  }
}

Module.onRuntimeInitialized = async function() {
  console.log("onRuntimeInitialized");
  doRun();
};
