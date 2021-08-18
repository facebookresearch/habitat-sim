// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "GlmUtils.h"

namespace Mn = Magnum;

namespace esp {
namespace batched_sim {

glm::mat4 toGlmMat4(const Mn::Vector3& pos, const Mn::Quaternion& rot) {
  Mn::Matrix3x3 r = rot.toMatrix();
  return glm::mat4(r[0][0], r[0][1], r[0][2], 0.f, r[1][0], r[1][1], r[1][2],
                   0.f, r[2][0], r[2][1], r[2][2], 0.f, pos.x(), pos.y(),
                   pos.z(), 1.f);
}

glm::mat4x3 toGlmMat4x3(const Magnum::Matrix4& m) {
  return glm::mat4x3(m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2],
                     m[2][0], m[2][1], m[2][2], m[3][0], m[3][1], m[3][2]);
  // return glm::mat4x3(
  //   m[0][0], m[1][0], m[2][0], m[3][0],
  //   m[0][1], m[1][1], m[2][1], m[3][1],
  //   m[0][2], m[1][2], m[2][2], m[3][2]);
}

}  // namespace batched_sim
}  // namespace esp
