// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <Magnum/Magnum.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Math/Vector3.h>

#include "esp/scene/SceneGraph.h"

namespace esp {
namespace gfx {

enum class LightPositionModel {
  /** @brief Light position is relative to the camera */
  CAMERA = 0,
  /** @brief Light position is relative to scene */
  GLOBAL = 1,
  /** @brief Light position is relative to the object being rendered */
  OBJECT = 2,
};

/** @brief Contains a single light's information */
struct LightInfo {
  Magnum::Vector3 position;
  Magnum::Color4 color;
  LightPositionModel model = LightPositionModel::GLOBAL;
};

bool operator==(const LightInfo& a, const LightInfo& b);
bool operator!=(const LightInfo& a, const LightInfo& b);

using LightSetup = std::vector<LightInfo>;

/**
 * @brief Get position relative to a camera for a @ref LightInfo and a
 * rendered object
 *
 * @param transformationMatrix Describes object position relative to camera
 * @param cameraMatrix Describes world position relative to camera
 * @return Magnum::Vector3 Light position relative to camera
 */
Magnum::Vector3 getLightPositionRelativeToCamera(
    const LightInfo& light,
    const Magnum::Matrix4& transformationMatrix,
    const Magnum::Matrix4& cameraMatrix);

/**
 * @brief Get a @ref LightSetup with lights at the corners of a box
 */
LightSetup getLightsAtBoxCorners(
    const Magnum::Range3D& box,
    const Magnum::Color4& lightColor = Magnum::Color4{0.4f});

}  // namespace gfx
}  // namespace esp
