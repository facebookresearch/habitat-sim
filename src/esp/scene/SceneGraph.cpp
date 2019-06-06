// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneGraph.h"
#include <Magnum/Math/Algorithms/GramSchmidt.h>

namespace esp {
namespace scene {

SceneGraph::SceneGraph()
    : rootNode_{world_},
      defaultRenderCameraNode_{rootNode_},
      defaultRenderCamera_{defaultRenderCameraNode_} {}

// set transformation and projection matrix to the default camera
void SceneGraph::setDefaultRenderCamera(sensor::Sensor& sensor) {
  ASSERT(sensor.isVisualSensor());

  Magnum::Matrix4 T = sensor.object().absoluteTransformation();
  Magnum::Matrix3 R = T.rotationScaling();
  Magnum::Math::Algorithms::gramSchmidtOrthonormalizeInPlace(R);

  VLOG(1) << "||R - GS(R)|| = "
          << Eigen::Map<mat3f>((R - T.rotationShear()).data()).norm();

  T = Magnum::Matrix4::from(R, T.translation()) *
      Magnum::Matrix4::scaling(T.scaling());

  // set the transformation to the default camera
  // so that the camera has the correct modelview matrix for rendering;
  // to do it,
  // obtain the *absolute* transformation from the sensor node,
  // apply it as the *relative* transformation between the default camera and
  // its parent, which is rootNode_.
  defaultRenderCameraNode_.setTransformation(T);

  // set the projection matrix to the default camera
  sensor.setProjectionMatrix(defaultRenderCamera_);
}

}  // namespace scene
}  // namespace esp
