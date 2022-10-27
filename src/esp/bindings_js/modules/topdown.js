// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

import { throttle } from "./utils";

/**
 * TopDownMap class
 */
class TopDownMap {
  // PUBLIC methods.

  /**
   * Create top-down map.
   * @param {PathFinder} pathFinder - pathFinder used to create map
   * @param {HTMLCanvasElement} canvas - canvas element for drawing map
   */
  constructor(pathFinder, canvas) {
    this.pathFinder = pathFinder;
    this.bounds = pathFinder.bounds;
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.widthComponent = 0;
    this.heightComponent = 2;
    this.scale = 1.0;
    this.ctx.strokeStyle = "blue";
    this.ctx.lineWidth = 3;
    this.currentY = -Infinity;
  }

  /**
   * Draw the topdown map centered in the canvas.
   */
  draw() {
    const ctx = this.ctx;
    const canvas = this.canvas;
    const [x, y] = this.mapCenterInset();
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.putImageData(this.imageData, x, y);
  }

  /**
   * Set the start position of the trajectory.
   * @param {vec3f} position - start position
   */
  start(position) {
    this.currentY = position[1];
    this.imageData = this.createMap();
    this.draw();
    const [x, y] = this.convertPosition(position);
    this.ctx.beginPath();
    this.ctx.moveTo(x, y);
  }

  /**
   * Update trajectory with new position
   * @param {vec3f} position - new position
   * @param {int} throttleMs - time gap for throttle
   */
  moveTo(position, throttleMs = 0) {
    if (throttleMs !== 0) {
      if (!this.throttledMoveTo) {
        this.throttledMoveTo = throttle(this.moveTo.bind(this), throttleMs);
      }
      this.throttledMoveTo(position);
    } else {
      /*
       * If we've gone up or down a floor, we need to create a new map.
       * A change in Y of 0.25 should be sufficient as a test.
       */
      if (
        position[1] < this.currentY - 0.25 ||
        position[1] > this.currentY + 0.25
      ) {
        this.start(position);
      } else {
        let [x, y] = this.convertPosition(position);
        this.ctx.lineTo(x, y);
        this.ctx.stroke();
      }
    }
  }

  // PRIVATE methods.

  /*
   * We inset the map in order to center it in the canvas.
   */
  mapCenterInset() {
    let x = (this.canvas.width - this.imageData.width) / 2;
    let y = (this.canvas.height - this.imageData.height) / 2;
    return [x, y];
  }

  convertPosition(position) {
    const i = this.widthComponent;
    const j = this.heightComponent;
    let [x, y] = this.mapCenterInset();
    x += (position[i] - this.bounds.min[i]) * this.scale;
    y += (position[j] - this.bounds.min[j]) * this.scale;
    return [x, y];
  }

  /*
   * Produces map using a regular grid sampling of navigability.
   */
  createMap() {
    const canvas = this.canvas;
    let width = this.bounds.max[0] - this.bounds.min[0];
    let height = this.bounds.max[2] - this.bounds.min[2];
    let heightInPixels = 0;
    let widthInPixels = 0;

    // Best-Fit: orient and scale for tightest fit
    if (
      (width > height && canvas.width < canvas.height) ||
      (width < height && canvas.width > canvas.height)
    ) {
      this.widthComponent = 2;
      this.heightComponent = 0;
      let tmp = width;
      width = height;
      height = tmp;
    }
    if (height / width > canvas.height / canvas.width) {
      // Fit height
      this.scale = canvas.height / height;
      heightInPixels = canvas.height;
      widthInPixels = Math.round(width * this.scale - 0.5);
    } else {
      // Fit width
      this.scale = canvas.width / width;
      widthInPixels = canvas.width;
      heightInPixels = Math.round(height * this.scale - 0.5);
    }

    let imageData = new ImageData(widthInPixels, heightInPixels);

    // Create map by painting navigable locations white
    const stride = 4 * canvas.width;
    const increment = 1 / this.scale;
    const color = 0xffffffff;
    let y = this.bounds.min[this.heightComponent] + increment / 2;
    let i, j;
    for (i = 0; i < heightInPixels; i++) {
      let x = this.bounds.min[this.widthComponent] + increment / 2;
      for (j = 0; j < widthInPixels; j++) {
        let point = [0, this.currentY, 0];
        point[this.widthComponent] = x;
        point[this.heightComponent] = y;
        let isNavigable = this.pathFinder.isNavigable(point, 0.5);
        if (isNavigable) {
          imageData.data[i * stride + j * 4] = (color >> 24) & 0xff;
          imageData.data[i * stride + j * 4 + 1] = (color >> 16) & 0xff;
          imageData.data[i * stride + j * 4 + 2] = (color >> 8) & 0xff;
          imageData.data[i * stride + j * 4 + 3] = color & 0xff;
        }
        x += increment;
      }
      y += increment;
    }
    return imageData;
  }
}

export default TopDownMap;
