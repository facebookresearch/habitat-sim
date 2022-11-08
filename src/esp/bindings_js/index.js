// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/* global Module */

import WebDemo from "./modules/web_demo";
import ViewerDemo from "./modules/viewer_demo";
import {
  defaultScene,
  defaultPhysicsConfigFilepath,
  infoSemanticFileName
} from "./modules/defaults";
import "./bindings.css";
import {
  checkWebAssemblySupport,
  checkWebgl2Support,
  getInfoSemanticUrl,
  buildConfigFromURLParameters,
  preload
} from "./modules/utils";
import TestPage from "./modules/test_page";

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
    preload(getInfoSemanticUrl(config.scene, infoSemanticFileName));
  }
});

Module.onRuntimeInitialized = async function() {
  console.log("hsim_bindings initialized");

  if (window.isTestPage) {
    Module.testPage.onRuntimeInitialized();
    return;
  }

  let demo;
  if (window.viewerEnabled) {
    demo = new ViewerDemo();
  } else {
    demo = new WebDemo();
  }

  if (demo) {
    demo.display();
  }
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
