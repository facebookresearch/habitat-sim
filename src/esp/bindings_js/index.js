/*global Module */

import WebDemo from "./modules/web_demo";
import VRDemo from "./modules/vr_demo";
import "./bindings.css";

Module["onRuntimeInitialized"] = function() {
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
