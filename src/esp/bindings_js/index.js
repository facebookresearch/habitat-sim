/* global FS, Module */

import WebDemo from "./modules/web_demo";
import VRDemo from "./modules/vr_demo";
import { defaultScene } from "./modules/defaults";
import "./bindings.css";

function preload(file) {
  FS.createPreloadedFile("/", file, file, true, false);
}

Module.preRun.push(() => {
  let config = {};
  config.scene = defaultScene;
  for (let arg of window.location.search.substr(1).split("&")) {
    let [key, value] = arg.split("=");
    if (key && value) {
      config[key] = value;
    }
  }
  const scene = config.scene;
  preload(scene);
  Module.scene = scene;
  const fileNoExtension = scene.substr(0, scene.lastIndexOf("."));
  preload(fileNoExtension + ".navmesh");
  if (config.semantic === "mp3d") {
    preload(fileNoExtension + ".house");
    preload(fileNoExtension + "_semantic.ply");
  }
});

Module.onRuntimeInitialized = () => {
  console.log("hsim_bindings initialized");
  let demo;
  if (window.vrEnabled) {
    if (navigator && navigator.getVRDisplays) {
      console.log("Web VR is supported");
      demo = new VRDemo();
    }
  }

  if (!demo) {
    demo = new WebDemo();
  }
  demo.display();
};
