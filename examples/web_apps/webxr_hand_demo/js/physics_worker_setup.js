// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/* global preload, importScripts, PhysicsWorker */

// Sets up the Module object, which is used for all the bindings and
// interactions with Habitat.
function createMagnumModule(init) {
  const module = Object.assign({}, Module);
  Object.assign(module, {
    preRun: [],
    postRun: [],
    arguments: [],
    canvas: null,
    status: null,
    statusDescription: null,
    log: null,

    // These dictate what to do with various outputs from the wasm. We
    // currently just forward them to the JS console.
    printErr: function(msg) {
      console.log("physics_worker_setup.js - printErr:", msg);
    },
    print: function(msg) {
      console.log("physics_worker_setup.js - print:", msg);
    },
    setStatus: function(msg) {
      console.log("physics_worker_setup.js - setStatus:", msg);
    },
    setStatusDescription: function(msg) {
      console.log("physics_worker_setup.js - setStatusDescription:", msg);
    },

    totalDependencies: 0,
    monitorRunDependencies: function(left) {
      this.totalDependencies = Math.max(this.totalDependencies, left);
      if (left) {
        module.setStatus("Downloading...");
        module.setStatusDescription(
          this.totalDependencies - left + " / " + this.totalDependencies
        );
      } else {
        module.setStatus("Download complete");
        module.setStatusDescription("");
      }
    }
  });
  Object.assign(module, init);
  module.setStatus("Downloading...");
  return module;
}

let stageFilepath, physicsConfigFilepath, objectBaseFilepath;
var Module;

// First, wait for the main thread to send us the files to preload.
onmessage = function(e) {
  if (e.data.type == "preloadInfo") {
    // Get preloadInfo from the main thread.
    let preloadInfo = e.data.value;
    // Hack: the 'preload' function was passed here as a string containing JS
    // code.
    eval(preloadInfo.preloadFunc);

    Module = createMagnumModule();

    // Indicate that the first thing to do is to preload all the files.
    Module.preRun.push(() => {
      physicsConfigFilepath = preloadInfo.physicsConfigFilepath;
      stageFilepath = preloadInfo.stageFilepath;
      objectBaseFilepath = preloadInfo.objectBaseFilepath;
      for (const file of preloadInfo.preloadedFiles) {
        preload(file);
      }
    });

    // Once the functions in Module.preRun are done, we can run start().
    Module.onRuntimeInitialized = async function() {
      start();
    };

    // When we import this script, it will automatically execute the stuff in
    // preRun, then onRuntimeInitialized. That is why we needed to wait before
    // importing this.
    importScripts("hsim_bindings.js");
  } else {
    console.assert(false); // this should be unreachable
  }
};

importScripts("physics_worker.js");

// Main entry point of all the physics logic. This function is called from
// Module.onRuntimeInitialized after all the preloading is done.
function start() {
  // Tell the main thread that we are ready to start stepping.
  postMessage({ type: "ready", value: null });

  // Redefine onmessage to wait for the main thread to give us the start signal.
  onmessage = function(e) {
    if (e.data.type == "start") {
      let startData = e.data.value;
      let physicsWorker = new PhysicsWorker(
        physicsConfigFilepath,
        stageFilepath,
        objectBaseFilepath,
        startData.handFilepathsByHandIndex,
        startData.objectSpawnOrder
      );
      physicsWorker.start();
    } else {
      console.assert(false); // this should be unreachable
    }
  };
}
