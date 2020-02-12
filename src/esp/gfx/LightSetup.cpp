// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "LightSetup.h"

namespace esp {
namespace gfx {

Magnum::Vector3 getLightPositionRelativeToCamera(
    const LightInfo& light,
    const Magnum::Matrix4& transformationMatrix,
    const Magnum::Matrix4& cameraMatrix) {
  if (light.model == LightPositionModel::OBJECT)
    return transformationMatrix.transformPoint(light.position);

  if (light.model == LightPositionModel::GLOBAL)
    return cameraMatrix.transformPoint(light.position);

  // LightPositionModel::CAMERA
  return light.position;
}

LightSetup getLightsAtSceneCorners(scene::SceneGraph& sceneGraph, Magnum::Float lightIntensity) {
  const Magnum::Range3D& sceneBB =
      sceneGraph.getRootNode().computeCumulativeBB();

  // NOLINTNEXTLINE(google-build-using-namespace)
  using namespace Magnum::Math::Literals;

  return LightSetup{{sceneBB.frontTopLeft(), 0xffffff_rgbf * lightIntensity},
                    {sceneBB.frontTopRight(), 0xffffff_rgbf * lightIntensity},
                    {sceneBB.frontBottomLeft(), 0xffffff_rgbf * lightIntensity},
                    {sceneBB.frontBottomRight(), 0xffffff_rgbf * lightIntensity},
                    {sceneBB.backTopLeft(), 0xffffff_rgbf * lightIntensity},
                    {sceneBB.backTopRight(), 0xffffff_rgbf * lightIntensity},
                    {sceneBB.backBottomLeft(), 0xffffff_rgbf * lightIntensity},
                    {sceneBB.backBottomRight(), 0xffffff_rgbf * lightIntensity}};
}

}  // namespace gfx
}  // namespace esp
