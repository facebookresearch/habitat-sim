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

namespace esp {
namespace batched_sim {

glm::mat4 toGlmMat4(const Magnum::Vector3& pos, const Magnum::Quaternion& rot);

glm::mat4x3 toGlmMat4x3(const Magnum::Matrix4& m);

}  // namespace batched_sim
}  // namespace esp

#endif
