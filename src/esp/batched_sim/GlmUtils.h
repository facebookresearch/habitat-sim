// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_BATCHEDSIM_GLMUTILS_H_
#define ESP_BATCHEDSIM_GLMUTILS_H_

#include <glm/gtx/transform.hpp>

#include <Magnum/Magnum.h>
#include <Magnum/Math/Math.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Math/Quaternion.h>

// for inlines
#include "esp/batched_sim/BatchedSimAssert.h"
#include <Magnum/Math/Range.h>

namespace esp {
namespace batched_sim {

glm::mat4 toGlmMat4(const Magnum::Vector3& pos, const Magnum::Quaternion& rot);
glm::mat4 toGlmMat4(const Magnum::Matrix4& m);

glm::mat4x3 toGlmMat4x3(const Magnum::Matrix4& m);
Magnum::Vector3 getMagnumTranslation(const glm::mat4x3& glMat);

Magnum::Vector3 inverseTransformPoint(const glm::mat4x3& glMat, const Magnum::Vector3& pos);

Magnum::Vector3 getRangeCorner(const Magnum::Range3D& range, int cornerIdx);

bool sphereBoxContactTest(const Magnum::Vector3& sphereOrigin, float sphereRadiusSq, const Magnum::Range3D& aabb);

template<int maxTests, bool numTestsIsMaxTests>
bool batchSphereOrientedBoxContactTest(const glm::mat4x3** orientedBoxTransforms, 
  const Magnum::Vector3** positions,
  float sphereRadiusSq, const Magnum::Range3D** boxRanges, int numTests);



/*
*/

}  // namespace batched_sim
}  // namespace esp

#endif
