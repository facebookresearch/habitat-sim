// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

export const defaultAgentConfig = {
  height: 1.5,
  radius: 0.1,
  mass: 32.0,
  linearAcceleration: 20.0,
  angularAcceleration: 4 * Math.PI,
  linearFriction: 0.5,
  angularFriction: 1.0,
  coefficientOfRestitution: 0.0
};

export const defaultStartState = {
  position: [-1.2676633596420288, 0.2047852873802185, 12.595427513122559],
  rotation: [0, 0.4536385088584658, 0, 0.8911857849408661]
};

export const defaultGoal = {
  position: [2.2896811962127686, 0.11950381100177765, 16.97636604309082]
};

export const defaultEpisode = {
  startState: defaultStartState,
  goal: defaultGoal
};

export const defaultResolution = { height: 480, width: 640 };

export const defaultScene = "skokloster-castle.glb";

export const infoSemanticFileName = "info_semantic.json";

export const defaultPhysicsConfigFilepath = "data/default.physics_config.json";
