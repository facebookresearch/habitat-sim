// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/**
 * ObjectSensor class
 */
class ObjectSensor {
  // PUBLIC methods.

  /**
   * Create a scope for object navigation task.
   * @param {Object} searchRect - rect for Intersection-over-Union
   * @param {Object} semanticShape - shape of semantic observation
   * @param {SemanticScene} semanticScene - semantic information for scene
   * @param {String} objectClass - class name of object
   */
  constructor(searchRect, semanticShape, semanticScene, objectClass) {
    this.rect = searchRect;
    this.semanticShape = semanticShape;
    this.semanticScene = semanticScene;
    this.objectIds = [];

    const categories = this.semanticScene.categories;
    const objects = this.semanticScene.objects;

    let categoryIndex = -1;
    for (let i = 0; i < categories.size(); i++) {
      const category = categories.get(i);
      if (category && category.getName("") === objectClass) {
        categoryIndex = category.getIndex("");
        break;
      }
    }
    for (let i = 0; i < objects.size(); i++) {
      const object = objects.get(i) || {};
      if (object.category && object.category.getIndex("") === categoryIndex) {
        this.objectIds.push(i);
      }
    }
  }

  /**
   * Compute IOU for for a given class.
   * @param {Observation} semanticObservation - observation to search
   * @param {String} className - object class to search for
   */
  computeIOU(observation) {
    const width = this.semanticShape.get(1);
    const height = this.semanticShape.get(0);
    const left = this.rect.left;
    const top = this.rect.top;
    const right = this.rect.right;
    const bottom = this.rect.bottom;

    let maskPixelsIn = 0;
    let maskPixelsOut = 0;
    let inSearchArea = false;
    for (let j = 0; j < height; j++) {
      inSearchArea = j >= top && j < bottom;
      for (let i = 0; i < width; i++) {
        if (this.objectIds.includes(observation[j * width + i])) {
          if (inSearchArea && i >= left && i < right) {
            maskPixelsIn++;
          } else {
            maskPixelsOut++;
          }
        }
      }
    }
    if (maskPixelsIn === 0) {
      return 0.0;
    }
    const rectArea = (bottom - top) * (right - left);
    return maskPixelsIn / (maskPixelsOut + rectArea);
  }
}

export default ObjectSensor;
