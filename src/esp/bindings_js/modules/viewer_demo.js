// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

import WebDemo from "./web_demo";
import "../viewer.css";

class ViewerDemo extends WebDemo {
  constructor(canvasId = "canvas") {
    super(canvasId);
    this.canvasElement = document.getElementById(canvasId);
    window.addEventListener("resize", this.windowResizeEvent.bind(this));
  }

  initializeModules(...args) {
    if (args.length === 3) {
      args[2] = false;
    } else if (args.length == 2) {
      args.push(false);
    }
    super.initializeModules(...args);
  }
  resetCanvasToCurrent() {
    this.currentResolution = {
      height: this.canvasElement.offsetHeight,
      width: this.canvasElement.offsetWidth
    };
    // Make sure width for canvas is set properly
    this.resetCanvas(this.currentResolution);
  }

  display() {
    this.resetCanvasToCurrent();
    super.display();
  }

  windowResizeEvent() {
    // TODO: Later add resize handling
  }
}

export default ViewerDemo;
