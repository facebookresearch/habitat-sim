/* global FS, Module */

import WebDemo from "./modules/web_demo";
import VRDemo from "./modules/vr_demo";
import "./bindings.css";

Module.preRun.push(() => {
  let arg;
  let scene;
  for (arg of window.location.search.substr(1).split("&")) {
    let [key, value] = arg.split("=");
    if (key == "scene") {
      scene = value;
    }
  }
  FS.createPreloadedFile("/", scene, scene, true, false);
  Module.scene = scene;
  let navmesh = scene.replace(/\..*/, ".navmesh");
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
