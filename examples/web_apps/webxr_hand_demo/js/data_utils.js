// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

export class DataUtils {
  static dataDir = "data/";

  static getPhysicsConfigFilepath() {
    return this.dataDir + "default.physics_config.json";
  }

  static getObjectBaseFilepath() {
    return this.dataDir + "objects/";
  }

  static getStageBaseFilepath() {
    return this.dataDir + "stages/";
  }
}
