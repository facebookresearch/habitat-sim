// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/* global FS, Module */

import WebDemo from "./modules/web_demo";
import VRDemo from "./modules/vr_demo";
import ViewerDemo from "./modules/viewer_demo";
import { defaultScene } from "./modules/defaults";
import "./bindings.css";
import {
  checkWebAssemblySupport,
  checkWebgl2Support,
  getInfoSemanticUrl,
  buildConfigFromURLParameters
} from "./modules/utils";

function cacheUrlBlob(fs, url, blob) {
  fs.root.getFile(url, { create: true }, fileEntry => {
    fileEntry.createWriter(function(fileWriter) {
      fileWriter.write(blob);
    });
  });
}

function cacheUrl(fs, url) {
  fetch(url)
    .then(response => response.blob())
    .then(blob => {
      cacheUrlBlob(fs, url, blob);
    });
}

function preload(url, file = null) {
  if (!file) {
    file = url;
  }
  if (url.indexOf("http") === 0) {
    const splits = url.split("/");
    file = splits[splits.length - 1];
  }
  FS.createPreloadedFile("/", file, url, true, false);
  return file;
}

function resolveAfter2Seconds() {
  return new Promise(resolve => {
    setTimeout(() => {
      resolve("resolved");
    }, 2000);
  });
}

async function asyncCall() {
  console.log("calling");
  var result = await resolveAfter2Seconds();
  console.log(result);
  // expected output: 'resolved'
}

let cachedScene;

async function onInitFs(fs) {
  Module.preRun.push(() => {
    console.log("preload time");
  });

  let dirEntries = await new Promise((resolve, reject) => {
    fs.root.createReader().readEntries(resolve, reject);
  });

  console.log(dirEntries);

  if (!window.config.scene) {
    cacheUrl(fs, window.config.scene);
  }

  let blob = await new Promise((resolve, reject) => {
    dirEntries[0].file(resolve, reject);
  });

  const url = window.URL.createObjectURL(blob);
  cachedScene = url;
  console.log("Opened file system: " + fs.name);
  Module.preRun.push(preloadFiles);
}

asyncCall();
window.config = {};
window.config.scene = defaultScene;
buildConfigFromURLParameters(window.config);

// https://developer.chrome.com/apps/offline_storage#query
var requestedBytes = 1024 * 1024 * 3000; // 300MB
navigator.webkitPersistentStorage.requestQuota(
  requestedBytes,
  function(grantedBytes) {
    window.webkitRequestFileSystem(window.PERSISTENT, grantedBytes, onInitFs);
  },
  function(e) {
    console.log("Error", e);
  }
);

function preloadFiles() {
  const config = window.config;
  const scene = config.scene;
  if (cachedScene) {
    console.log(cachedScene);
    Module.scene = preload(cachedScene, "mesh_semantic.ply");
  } else {
    Module.scene = preload(scene);
  }
  const fileNoExtension = scene.substr(0, scene.lastIndexOf("."));

  preload(fileNoExtension + ".navmesh");
  if (config.semantic === "mp3d") {
    preload(fileNoExtension + ".house");
    preload(fileNoExtension + "_semantic.ply");
  } else if (config.semantic === "replica") {
    preload(getInfoSemanticUrl(config.scene));
  }
}

Module.onRuntimeInitialized = () => {
  console.log("hsim_bindings initialized");
  let demo;
  if (window.vrEnabled) {
    if (navigator && navigator.getVRDisplays) {
      console.log("Web VR is supported");
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
