// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_LIGHTSETUP_H_
#define ESP_GFX_LIGHTSETUP_H_

#include <Magnum/Magnum.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Math/Vector3.h>

#include "esp/scene/SceneGraph.h"

namespace esp {
namespace gfx {

enum class LightPositionModel {
  /** @brief Light position is relative to the camera */
  Camera = 0,
  /** @brief Light position is relative to scene */
  Global = 1,
  /** @brief Light position is relative to the object being rendered */
  Object = 2,
};

enum class LightType {
  /** @brief The type of light described by a light info*/
  Point = 0,
  Directional = 1,
  Spot = 2,

};

/** @brief Contains a single light's information. */
struct LightInfo {
  // Vector4 homogeneous-coordinate position
  // Use a Vector3 position and w == 1 to specify a point light with distance
  // attenuation. Or, use a Vector3 direction and w == 0 to specify a
  // directional light with no distance attenuation.
  Magnum::Vector4 vector;
  Magnum::Color3 color{1};
  LightPositionModel model = LightPositionModel::Global;
};

bool operator==(const LightInfo& a, const LightInfo& b);
bool operator!=(const LightInfo& a, const LightInfo& b);

//! A set of LightInfos.
using LightSetup = std::vector<LightInfo>;

/**
 * @brief Get position relative to a camera for a @ref LightInfo and a
 * rendered object. light.position and the return value are Vector4, with
 * w == 1 for positions and w == 0 for directions
 *
 * @param transformationMatrix Describes object position relative to camera
 * @param cameraMatrix Describes world position relative to camera
 * @return Magnum::Vector4 Light position relative to camera
 */
Magnum::Vector4 getLightPositionRelativeToCamera(
    const LightInfo& light,
    const Magnum::Matrix4& transformationMatrix,
    const Magnum::Matrix4& cameraMatrix);

/**
 * @brief Get light position in world space for a @ref LightInfo and a
 * rendered object. light.position and the return value are Vector4, with
 * w == 1 for positions and w == 0 for directions
 *
 * @param transformationMatrix Describes object position relative to camera
 * @param cameraMatrix Describes world position relative
 * @return Magnum::Vector4 Light position in world space
 */
Magnum::Vector4 getLightPositionRelativeToWorld(
    const LightInfo& light,
    const Magnum::Matrix4& transformationMatrix,
    const Magnum::Matrix4& cameraMatrix);

/**
 * @brief Get a @ref LightSetup with lights at the corners of a box
 */
LightSetup getLightsAtBoxCorners(
    const Magnum::Range3D& box,
    const Magnum::Color3& lightColor = Magnum::Color3{10.0f});

/**
 * @brief Get a @ref LightSetup with some directional lights approximating
 * daylight
 */
LightSetup getDefaultLights();

/**
 * @brief Get a single, combined ambient light color for use with the Phong
 * lighting model.
 */
Magnum::Color3 getAmbientLightColor(const LightSetup& lightSetup);

}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_LIGHTSETUP_H_
