/*global Module */

import WebDemo from "./modules/web_demo";
import "./bindings.css";

Module["onRuntimeInitialized"] = function() {
  console.log("hsim_bindings initialized");

  const demo = new WebDemo();
  demo.display();
};
