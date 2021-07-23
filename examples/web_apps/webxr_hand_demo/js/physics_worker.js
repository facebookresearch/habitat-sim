// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/* global FS, importScripts */

class DataUtils {
  static getDataDir() {
    return "data/webxr_hand_demo_dataset/";
  }

  static getPhysicsConfigFilepath() {
    return this.getDataDir() + "default.physics_config.json";
  }

  static getObjectBaseFilepath() {
    return this.getDataDir() + "objects/";
  }

  static getStageBaseFilepath() {
    return this.getDataDir() + "stages/";
  }

  static getObjectFilepath(name) {
    return this.getObjectBaseFilepath() + name + ".glb";
  }

  static getObjectConfigFilepath(name) {
    return this.getObjectBaseFilepath() + name + ".object_config.json";
  }

  static getObjectCollisionGlbFilepath(name) {
    return this.getObjectBaseFilepath() + name + "_cv_decomp.glb";
  }

  static getStageFilepath(name) {
    return this.getStageBaseFilepath() + name + ".glb";
  }

  static getStageConfigFilepath(name) {
    return this.getStageBaseFilepath() + name + ".stage_config.json";
  }
}

const objectSpawnOrder = [
  "frl_apartment_vase_02", // gray
  "frl_apartment_plate_02", // double-layer
  "frl_apartment_pan_01", // blue, with handle

  "frl_apartment_kitchen_utensil_05", // serving tray
  "banana_fixed",
  "banana_fixed",
  "banana_fixed",

  "frl_apartment_plate_01",
  "frl_apartment_plate_01",

  "frl_apartment_kitchen_utensil_06", // white handleless cup
  "frl_apartment_kitchen_utensil_06", // white handleless cup

  "frl_apartment_bowl_06", // small white
  "frl_apartment_bowl_06", // small white

  "frl_apartment_kitchen_utensil_02", // green spice shaker
  "frl_apartment_kitchen_utensil_03" // orange spice shaker
];

function workerlog(message) {
  console.log("WORKER:\n" + message);
}

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
    canvas: null, // document.getElementById('canvas'),
    status: null, // document.getElementById('status'),
    statusDescription: null, // document.getElementById('status-description'),
    log: null, // document.getElementById('log'),
    printErr: function(_message) {
      workerlog(_message);
    },
    print: function(_message) {
      workerlog(_message);
    },
    setStatus: function(message) {
      workerlog("setStatus: " + message);
    },
    setStatusDescription: function(message) {
      workerlog("setStatusDescription: " + message);
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
        // module.log.style.display = 'block';
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

let stageName;

function doPreloading() {
  preloadFunc(DataUtils.getPhysicsConfigFilepath());

  preloadFunc(DataUtils.getStageFilepath(stageName));
  preloadFunc(DataUtils.getStageConfigFilepath(stageName));

  preloadFunc(DataUtils.getObjectFilepath("hand_r_open"));
  preloadFunc(DataUtils.getObjectConfigFilepath("hand_r_open"));
  preloadFunc(DataUtils.getObjectFilepath("hand_r_closed"));
  preloadFunc(DataUtils.getObjectConfigFilepath("hand_r_closed"));

  preloadFunc(DataUtils.getObjectFilepath("hand_l_open"));
  preloadFunc(DataUtils.getObjectConfigFilepath("hand_l_open"));
  preloadFunc(DataUtils.getObjectFilepath("hand_l_closed"));
  preloadFunc(DataUtils.getObjectConfigFilepath("hand_l_closed"));

  const replicaCadObjectNames = new Set();
  for (const object of objectSpawnOrder) {
    replicaCadObjectNames.add(object);
  }

  for (const name of replicaCadObjectNames) {
    preloadFunc(DataUtils.getObjectFilepath(name));
    preloadFunc(DataUtils.getObjectCollisionGlbFilepath(name));
    preloadFunc(DataUtils.getObjectConfigFilepath(name));
  }
}

var Module = createMagnumModule();
Module.preRun.push(() => {
  // todo: get the stage name from onmessage
  stageName = "remake_v0_JustBigStuff_00";
  doPreloading();
});
Module.onRuntimeInitialized = async function() {
  start();
};
importScripts("hsim_bindings.js");

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
    this.config.scene_id = DataUtils.getStageFilepath(stageName);
    this.config.enablePhysics = true;
    this.config.physicsConfigFile = DataUtils.getPhysicsConfigFilepath();
    this.config.createRenderer = false;
    this.config.enableGfxReplaySave = true;
    this.sim = new Module.Simulator(this.config);

    Module.loadAllObjectConfigsFromPath(
      this.sim,
      DataUtils.getObjectBaseFilepath()
    );

    this.recorder = this.sim.getGfxReplayManager().getRecorder();
  }

  start() {
    onmessage = function(e) {
      if (e.data.type == "spawn") {
        console.log("spawn");
      }
    };

    // physics step
    this.physicsStepFunction = setInterval(() => {
      this.sim.stepWorld(1.0 / 60);

      this.recorder.saveKeyframe();
      let keyframe = this.recorder.getLatestKeyframe();
      let jsonKeyframe = this.recorder.keyframeToString(keyframe);
      postMessage({ type: "keyframe", value: jsonKeyframe });
    }, 1000.0 / 60);
  }
}
