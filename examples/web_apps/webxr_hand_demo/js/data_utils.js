// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

export class DataUtils {
  static getDataDir() {
    return "data/webxr_hand_demo_dataset/";
  }

  static getPhysicsConfigFilepath() {
    return this.getDataDir() + "default.physics_config.json";
  }

  static getObjectBaseFilepath() {
    return this.getDataDir() + "objects/";
  }

  static getStageBaseFilepath() {
    return this.getDataDir() + "stages/";
  }

  static getObjectFilepath(name) {
    return this.getObjectBaseFilepath() + name + ".glb";
  }

  static getObjectConfigFilepath(name) {
    return this.getObjectBaseFilepath() + name + ".object_config.json";
  }

  static getObjectCollisionGlbFilepath(name) {
    return this.getObjectBaseFilepath() + name + "_cv_decomp.glb";
  }

  static getStageFilepath(name) {
    return this.getStageBaseFilepath() + name + ".glb";
  }

  static getStageConfigFilepath(name) {
    return this.getStageBaseFilepath() + name + ".stage_config.json";
  }
}
