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

// https://developer.mozilla.org/en-US/docs/Web/API/SubtleCrypto/digest
async function digestMessage(message) {
  const msgUint8 = new TextEncoder().encode(message);
  const hashBuffer = await crypto.subtle.digest("SHA-256", msgUint8);
  const hashArray = Array.from(new Uint8Array(hashBuffer));
  return hashArray.map(b => b.toString(16).padStart(2, "0")).join("");
}

async function cacheBlob(fs, filename, blob) {
  fs.root.getFile(filename, { create: true }, fileEntry => {
    fileEntry.createWriter(function(fileWriter) {
      fileWriter.write(blob);
    });
  });
}

async function cacheUrl(fs, url) {
  const dirEntries = await new Promise((resolve, reject) => {
    fs.root.createReader().readEntries(resolve, reject);
  });

  let blob;
  const urlDigest = await digestMessage(url);
  let entry = dirEntries.find(e => e.name === urlDigest);
  if (entry) {
    blob = await new Promise((resolve, reject) => {
      entry.file(resolve, reject);
    });
  } else {
    blob = await fetch(url).then(response => response.blob());
    cacheBlob(fs, urlDigest, blob);
  }

  return window.URL.createObjectURL(blob);
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

Module.preRun.push(async () => {
  Module.addRunDependency("initFs");
  // https://developer.chrome.com/apps/offline_storage#query
  const requestedBytes = 1024 * 1024 * 400; // 300MB
  const grantedBytes = await new Promise((resolve, reject) => {
    navigator.webkitPersistentStorage.requestQuota(
      requestedBytes,
      resolve,
      reject
    );
  });
  const fs = await new Promise((resolve, reject) => {
    window.webkitRequestFileSystem(
      window.PERSISTENT,
      grantedBytes,
      resolve,
      reject
    );
  });

  let config = {};
  config.scene = defaultScene;
  buildConfigFromURLParameters(config);

  const cachedScene = await cacheUrl(fs, config.scene);
  preloadFiles(config, cachedScene);
  window.config = config;
  Module.removeRunDependency("initFs");
});

function preloadFiles(config, cachedScene) {
  const scene = config.scene;
  if (cachedScene) {
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
