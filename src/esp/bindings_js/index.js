// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/* global FS, Module */

import WebDemo from "./modules/web_demo";
import VRDemo from "./modules/vr_demo";
import ViewerDemo from "./modules/viewer_demo";
import { defaultScene, defaultPhysicsConfigFilepath } from "./modules/defaults";
import "./bindings.css";
import {
  checkWebAssemblySupport,
  checkWebgl2Support,
  getInfoSemanticUrl,
  buildConfigFromURLParameters
} from "./modules/utils";
import TestPage from "./modules/test_page";

function preload(url) {
  let file_parents_str = "/";
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

Module.preRun.push(() => {
  if (window.isTestPage) {
    Module.testPage = new TestPage();
    Module.testPage.preRun(preload);
    return;
  }

  let config = {};
  config.scene = defaultScene;
  buildConfigFromURLParameters(config);
  window.config = config;
  const scene = config.scene;
  Module.scene = preload(scene);

  Module.physicsConfigFile = preload(defaultPhysicsConfigFilepath);

  const fileNoExtension = scene.substr(0, scene.lastIndexOf("."));

  preload(fileNoExtension + ".navmesh");
  if (config.semantic === "mp3d") {
    preload(fileNoExtension + ".house");
    preload(fileNoExtension + "_semantic.ply");
  } else if (config.semantic === "replica") {
    preload(getInfoSemanticUrl(config.scene));
  }
});

Module.onRuntimeInitialized = async function() {
  console.log("hsim_bindings initialized");

  if (window.isTestPage) {
    Module.testPage.onRuntimeInitialized();
    return;
  }

  let demo;
  if (window.vrEnabled) {
    const supported = await navigator.xr.isSessionSupported("immersive-vr");
    if (supported) {
      console.log("WebXR is supported");
      demo = new VRDemo();
    }
  } else if (window.viewerEnabled) {
    demo = new ViewerDemo();
  }

  if (!demo) {
    demo = new WebDemo();
  }

  demo.display();
};

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
