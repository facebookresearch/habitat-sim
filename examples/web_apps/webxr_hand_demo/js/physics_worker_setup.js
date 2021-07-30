// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/* global FS, importScripts, PhysicsWorker */

function preloadFunc(url) {
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
    printErr: function() {},
    print: function() {},
    setStatus: function() {},
    setStatusDescription: function() {},
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
  if (module.log) {
    module.log.style.display = "none";
  }
  return module;
}

let stageFilepath, physicsConfigFilepath, objectBaseFilepath;
var Module;

onmessage = function(e) {
  console.assert(e.data.type == "preloadInfo");
  let preloadInfo = e.data.value;

  Module = createMagnumModule();

  Module.preRun.push(() => {
    physicsConfigFilepath = preloadInfo.physicsConfigFilepath;
    stageFilepath = preloadInfo.stageFilepath;
    objectBaseFilepath = preloadInfo.objectBaseFilepath;
    for (const file of preloadInfo.preloadedFiles) {
      preloadFunc(file);
    }
  });

  Module.onRuntimeInitialized = async function() {
    start();
  };

  importScripts("hsim_bindings.js");
};

importScripts("physics_worker.js");

function start() {
  postMessage({ type: "ready", value: null });
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
    }
  };
}
