// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "GlmUtils.h"
#include "esp/batched_sim/BatchedSimAssert.h"

#include <Magnum/Math/Range.h>

namespace Mn = Magnum;

namespace esp {
namespace batched_sim {

glm::mat4 toGlmMat4(const Mn::Vector3& pos, const Mn::Quaternion& rot) {
  Mn::Matrix3x3 r = rot.toMatrix();
  return glm::mat4(r[0][0], r[0][1], r[0][2], 0.f, r[1][0], r[1][1], r[1][2],
                   0.f, r[2][0], r[2][1], r[2][2], 0.f, pos.x(), pos.y(),
                   pos.z(), 1.f);
}

glm::mat4 toGlmMat4(const Magnum::Matrix4& m) {
  return glm::mat4(
    m[0][0], m[0][1], m[0][2], 0.f, 
    m[1][0], m[1][1], m[1][2], 0.f, 
    m[2][0], m[2][1], m[2][2], 0.f, 
    m[3][0], m[3][1], m[3][2], 1.f);
}

glm::mat4x3 toGlmMat4x3(const Magnum::Matrix4& m) {
  return glm::mat4x3(m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2],
                     m[2][0], m[2][1], m[2][2], m[3][0], m[3][1], m[3][2]);
  // return glm::mat4x3(
  //   m[0][0], m[1][0], m[2][0], m[3][0],
  //   m[0][1], m[1][1], m[2][1], m[3][1],
  //   m[0][2], m[1][2], m[2][2], m[3][2]);
}

Mn::Vector3 getMagnumTranslation(const glm::mat4x3& glMat) {
  return Mn::Vector3(glMat[3][0], glMat[3][1], glMat[3][2]);
}

Magnum::Vector3 inverseTransformPoint(const glm::mat4x3& glMat, const Magnum::Vector3& pos) {

  auto myDot = [](const Magnum::Vector3&a, const Magnum::Vector3& b) {
    return a.x() * b.x()
      + a.y() * b.y()
      + a.z() * b.z();
  };

  auto mySub = [](const Magnum::Vector3&a, const Magnum::Vector3& b) {
    return Magnum::Vector3(
      a.x() - b.x(),
      a.y() - b.y(),
      a.z() - b.z());
  };

  // Magnum::Vector3 offset = pos - Magnum::Vector3(glMat[3][0], glMat[3][1], glMat[3][2]);
  Magnum::Vector3 offset = mySub(pos, Magnum::Vector3(glMat[3][0], glMat[3][1], glMat[3][2]));

  auto basisX = Magnum::Vector3(glMat[0][0], glMat[0][1], glMat[0][2]);
  auto basisY = Magnum::Vector3(glMat[1][0], glMat[1][1], glMat[1][2]);
  auto basisZ = Magnum::Vector3(glMat[2][0], glMat[2][1], glMat[2][2]);

  BATCHED_SIM_ASSERT(basisX.isNormalized());
  BATCHED_SIM_ASSERT(basisY.isNormalized());
  BATCHED_SIM_ASSERT(basisZ.isNormalized());

  Magnum::Vector3 result(
    // Magnum::Math::dot(offset, basisX),
    // Magnum::Math::dot(offset, basisY),
    // Magnum::Math::dot(offset, basisZ));
    myDot(offset, basisX),
    myDot(offset, basisY),
    myDot(offset, basisZ));

  return result;
}


bool sphereBoxContactTest(const Magnum::Vector3& sphereOrigin, float sphereRadiusSq, const Magnum::Range3D& aabb) {

  const float& cx = sphereOrigin.x();
  const float& cy = sphereOrigin.y();
  const float& cz = sphereOrigin.z();

  const float& rx = aabb.min().x();
  const float& rx_plus_rw = aabb.max().x();
  const float& ry = aabb.min().y();
  const float& ry_plus_rh = aabb.max().y();
  const float& rz = aabb.min().z();
  const float& rz_plus_rd = aabb.max().z();

  // from http://jeffreythompson.org/collision-detection/circle-rect.php
  float testX = cx;
  float testY = cy;  
  float testZ = cz;

  if (cx < rx) {
    testX = rx;        // left edge
  } else if (cx > rx_plus_rw) {
    testX = rx_plus_rw;     // right edge
  }

  if (cy < ry) {
    testY = ry;        // top edge
  } else if (cy > ry_plus_rh) {
    testY = ry_plus_rh;     // bottom edge
  }

  if (cz < rz) {
    testZ = rz;        // forward edge
  } else if (cz > rz_plus_rd) {
    testZ = rz_plus_rd;     // back edge
  }

  float distX = cx-testX;
  float distY = cy-testY;
  float distZ = cz-testZ;
  float distanceSq = (distX*distX) + (distY*distY) + (distZ*distZ);  

  return (distanceSq < sphereRadiusSq);
}

template<int maxTests, bool numTestsIsMaxTests>
bool batchSphereOrientedBoxContactTest(const glm::mat4x3** orientedBoxTransforms, 
  const Magnum::Vector3** positions,
  float sphereRadiusSq, const Magnum::Range3D** boxRanges, int numTests) {

  //__asm__ __volatile__("int3");
  //__asm__ __volatile__("int3");

  if (numTestsIsMaxTests) {
    BATCHED_SIM_ASSERT(numTests == maxTests);
  } else {
    BATCHED_SIM_ASSERT(numTests <= maxTests);
  }

  BATCHED_SIM_ASSERT(maxTests <= 64);
  uint64_t resultBits = 0;

  if (numTestsIsMaxTests) {
    for (int i = 0; i < maxTests; i++) {
      const auto posLocal = inverseTransformPoint(*orientedBoxTransforms[i], *positions[i]);
      const bool hit = sphereBoxContactTest(posLocal, sphereRadiusSq, *boxRanges[i]);
      if (hit) {
        resultBits |= (1 << i);
      }
    }
  } else {
    for (int i = 0; i < maxTests; i++) {
      // todo: try to move test to the end
      if (i < numTests) {
        const auto posLocal = inverseTransformPoint(*orientedBoxTransforms[i], *positions[i]);
        const bool hit = sphereBoxContactTest(posLocal, sphereRadiusSq, *boxRanges[i]);
        if (hit) {
          resultBits |= (1 << i);
        }
      }
    }
  }

  // __asm__ __volatile__("int3");
  // __asm__ __volatile__("int3");

  return resultBits;
}

template bool batchSphereOrientedBoxContactTest<64, true>(const glm::mat4x3** orientedBoxTransforms, 
  const Magnum::Vector3** positions,
  float sphereRadiusSq, const Magnum::Range3D** boxRanges, int numTests);
template bool batchSphereOrientedBoxContactTest<64, false>(const glm::mat4x3** orientedBoxTransforms, 
  const Magnum::Vector3** positions,
  float sphereRadiusSq, const Magnum::Range3D** boxRanges, int numTests);

}  // namespace batched_sim
}  // namespace esp
