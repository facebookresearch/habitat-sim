/* global FS, Module */

import WebDemo from "./modules/web_demo";
import VRDemo from "./modules/vr_demo";
import "./bindings.css";

Module.preRun.push(() => {
  let scene = "skokloster-castle.glb";
  for (let arg of window.location.search.substr(1).split("&")) {
    let [key, value] = arg.split("=");
    if (key === "scene" && value) {
      scene = value;
    }
  }
  FS.createPreloadedFile("/", scene, scene, true, false);
  Module.scene = scene;
  const navmesh = scene.substr(0, scene.lastIndexOf(".")) + ".navmesh";
  FS.createPreloadedFile("/", navmesh, navmesh, true, false);
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
