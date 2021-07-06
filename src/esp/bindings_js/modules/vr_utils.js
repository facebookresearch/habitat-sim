// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/* global Module */

export const VIEW_SENSORS = ["left_eye", "right_eye"];
export const pointToArray = p => [p.x, p.y, p.z, p.w];

export function getEyeSensorSpecs() {
  const resWidth = 1024;
  const resHeight = 1024;
  const specs = new Module.VectorSensorSpec();
  {
    const spec = new Module.CameraSensorSpec();
    spec.uuid = "left_eye";
    spec.sensorType = Module.SensorType.COLOR;
    spec.sensorSubType = Module.SensorSubType.PINHOLE;
    spec.resolution = [resWidth, resHeight];
    specs.push_back(spec);
  }
  {
    const spec = new Module.CameraSensorSpec();
    spec.uuid = "right_eye";
    spec.sensorType = Module.SensorType.COLOR;
    spec.sensorSubType = Module.SensorSubType.PINHOLE;
    spec.resolution = [resWidth, resHeight];
    specs.push_back(spec);
  }
  return specs;
}

// Given the WebXR viewer pose, update the positions/orientations of the view
// sensors
export function updateHeadPose(pose, agent) {
  const FWD = Module.Vector3.zAxis(1);
  const FWD_ANGLE = Math.atan2(FWD.z(), FWD.x());
  const DOWN = Module.Vector3.yAxis(-1);
  const viewYawOffset = 0;

  const headRotation = Module.toQuaternion(
    pointToArray(pose.transform.orientation)
  );
  const pointingVec = headRotation.transformVector(FWD);
  const pointingAngle =
    Math.atan2(pointingVec.z(), pointingVec.x()) - FWD_ANGLE;

  const agentQuat = Module.Quaternion.rotation(
    new Module.Rad(pointingAngle + viewYawOffset),
    DOWN
  );
  const inverseAgentRot = Module.Quaternion.rotation(
    new Module.Rad(-pointingAngle),
    DOWN
  );

  let state = new Module.AgentState();
  agent.getState(state);
  state.rotation = Module.toVec4f(agentQuat);
  agent.setState(state, false);

  for (var iView = 0; iView < pose.views.length; ++iView) {
    const view = pose.views[iView];

    const sensor = agent.getSubtreeSensors().get(VIEW_SENSORS[iView]);

    const pos = pointToArray(view.transform.position).slice(0, -1); // don't need w for position
    sensor.setLocalTransform(
      Module.toVec3f(
        inverseAgentRot.transformVector(new Module.Vector3(...pos))
      ),
      Module.toVec4f(
        Module.Quaternion.mul(
          inverseAgentRot,
          Module.toQuaternion(pointToArray(view.transform.orientation))
        )
      )
    );
  }
}
