/* global Module */
import { defaultAgentConfig, defaultEpisode } from "./defaults";
import SimEnv from "./simenv_embind";
import TopDownMap from "./topdown";
import NavigateTask from "./navigate";

class WebDemo {
  initializeModules(
    agentConfig = defaultAgentConfig,
    episode = defaultEpisode
  ) {
    this.sceneConfig = new Module.SceneConfiguration();
    this.sceneConfig.id = Module.scene;
    this.config = new Module.SimulatorConfiguration();
    this.config.scene = this.sceneConfig;

    this.simenv = new SimEnv(this.config, episode, 0);

    agentConfig = this.updateAgentConfigWithSensors({ ...agentConfig });

    this.simenv.addAgent(agentConfig);
    this.topdown = new TopDownMap(
      this.simenv.getPathFinder(),
      document.getElementById("topdown")
    );

    this.task = new NavigateTask(this.simenv, this.topdown, {
      canvas: document.getElementById("canvas"),
      semantic: document.getElementById("semantic"),
      radar: document.getElementById("radar"),
      status: document.getElementById("status")
    });
  }

  updateAgentConfigWithSensors(agentConfig = defaultAgentConfig) {
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

    agentConfig.sensorSpecifications = sensorConfigs;
    return agentConfig;
  }

  display() {
    this.initializeModules(defaultAgentConfig, defaultEpisode);

    this.task.init();
    this.task.reset();
  }
}

export default WebDemo;
