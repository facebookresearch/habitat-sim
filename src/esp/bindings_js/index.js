/*global Module, FS */

import SimEnv from "./simenv_embind";
import NavigateTask from "./navigate";
import TopDownMap from "./topdown";
import "./bindings.css";

Module["onRuntimeInitialized"] = function() {
  console.log("hsim_bindings initialized");

  const sensorConfigs = [
    {
      uuid: "rgb",
      sensorType: Module.SensorType.COLOR,
      resolution: [480, 640]
    },
    {
      uuid: "semantic",
      sensorType: Module.SensorType.SEMANTIC,
      resolution: [480, 640]
    }
  ];

  const agentConfig = {
    height: 1.5,
    radius: 0.1,
    mass: 32.0,
    linearAcceleration: 20.0,
    angularAcceleration: 4 * Math.PI,
    linearFriction: 0.5,
    angularFriction: 1.0,
    coefficientOfRestitution: 0.0,
    sensorSpecifications: sensorConfigs
  };

  const startState = {
    position: [-1.2676633596420288, 0.2047852873802185, 12.595427513122559],
    rotation: [0, 0.4536385088584658, 0, 0.8911857849408661]
  };

  const goal = {
    position: [2.2896811962127686, 0.11950381100177765, 16.97636604309082]
  };

  const episode = {
    startState: startState,
    goal: goal
  };

  let sceneConfig = new Module.SceneConfiguration();
  try {
    FS.stat("17DRP5sb8fy.glb");
    sceneConfig.id = "17DRP5sb8fy.glb";
  } catch (err) {
    console.log(
      "Can't find 17DRP5sb8fy.glb. Falling back to skokloster-castle.glb which doesn't have semantic information."
    );
    sceneConfig.id = "skokloster-castle.glb";
  }
  let config = new Module.SimulatorConfiguration();
  config.scene = sceneConfig;

  const simenv = new SimEnv(config, episode, 0);
  simenv.addAgent(agentConfig);
  const topdown = new TopDownMap(
    simenv.getPathFinder(),
    document.getElementById("topdown")
  );
  const task = new NavigateTask(simenv, topdown, {
    canvas: document.getElementById("canvas"),
    semantic: document.getElementById("semantic"),
    radar: document.getElementById("radar"),
    status: document.getElementById("status")
  });
  task.init();
  task.reset();

  window.config = config;
  window.sim = simenv;
};
