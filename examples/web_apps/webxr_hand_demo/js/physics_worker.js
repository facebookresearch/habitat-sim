// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/* global FS, importScripts */

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
    stageFilepath = preloadInfo.stageFilepath;
    physicsConfigFilepath = preloadInfo.physicsConfigFilepath;
    objectBaseFilepath = preloadInfo.objectBaseFilepath;
    for (const file of preloadInfo.preloadedFiles) {
      preloadFunc(file);
    }
  });

  Module.onRuntimeInitialized = async function() {
    start();
  };

  importScripts("../lib/habitat-sim-js/hsim_bindings.js");
};

// ----------------------------------

function start() {
  postMessage({ type: "ready", value: null });
  onmessage = function(e) {
    // todo: get a bunch of variables here rather than hard code into this file
    if (e.data.type == "start") {
      let physicsWorker = new PhysicsWorker();
      physicsWorker.start();
    }
  };
}

class PhysicsWorker {
  constructor() {
    // initialize stuff
    this.config = new Module.SimulatorConfiguration();
    this.config.scene_id = stageFilepath;
    this.config.enablePhysics = true;
    this.config.physicsConfigFile = physicsConfigFilepath;
    this.config.createRenderer = false;
    this.config.enableGfxReplaySave = true;
    this.sim = new Module.Simulator(this.config);

    Module.loadAllObjectConfigsFromPath(this.sim, objectBaseFilepath);

    this.recorder = this.sim.getGfxReplayManager().getRecorder();
  }

  start() {
    let spawn = this.spawn.bind(this);
    let deleteAllObjects = this.deleteAllObjects.bind(this);
    onmessage = function(e) {
      if (e.data.type == "spawn") {
        let spawnInfo = e.data.value;
        let handle = spawnInfo.handle;
        let pos = new Module.Vector3(...spawnInfo.pos);
        let vel = new Module.Vector3(...spawnInfo.vel);
        spawn(handle, pos, vel);
      } else if (e.data.type == "delete") {
        deleteAllObjects();
      }
    };

    // physics step
    this.physicsStepFunction = setInterval(() => {
      this.sim.stepWorld(0.003);

      this.recorder.saveKeyframe();
      let keyframe = this.recorder.getLatestKeyframe();
      let jsonKeyframe = this.recorder.keyframeToString(keyframe);
      postMessage({ type: "keyframe", value: jsonKeyframe });
    }, 1000.0 / 60);
  }

  curSpawned = [];

  spawn(handle, pos, vel) {
    const objId = this.sim.addObjectByHandle(handle, null, "", 0);
    this.sim.setTranslation(pos, objId, 0);
    this.sim.setLinearVelocity(vel, objId, 0);
    this.curSpawned.push(objId);
  }

  deleteAllObjects() {
    for (const objId of this.curSpawned) {
      this.sim.removeObject(objId, true, true, 0);
    }
    this.curSpawned = [];
  }
}
