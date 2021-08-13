// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/* global Module */

export const VIEW_SENSOR = "eye";

export function getEyeSensorSpecs(resolutionWidth, resolutionHeight) {
  const specs = new Module.VectorSensorSpec();
  const spec = new Module.CameraSensorSpec();
  spec.uuid = "eye";
  spec.sensorType = Module.SensorType.COLOR;
  spec.sensorSubType = Module.SensorSubType.PINHOLE;
  spec.resolution = [resolutionWidth, resolutionHeight];
  specs.push_back(spec);
  return specs;
}
