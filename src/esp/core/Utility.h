// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_CORE_UTILITY_H_
#define ESP_CORE_UTILITY_H_

/** @file */

#include <Magnum/Magnum.h>
#include <Magnum/Math/Algorithms/GramSchmidt.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Math/Quaternion.h>
#include <Magnum/Math/Vector3.h>
#include <cmath>
#include <cstdlib>

namespace esp {
namespace core {
/**
 * @brief generate a random rotation
 *
 * @return a unit quternion
 */
inline Magnum::Quaternion randomRotation() {
  // generate random rotation using:
  // http://planning.cs.uiuc.edu/node198.html
  double u1 = (rand() % 1000) / 1000.0;
  double u2 = (rand() % 1000) / 1000.0;
  double u3 = (rand() % 1000) / 1000.0;

  Magnum::Vector3 qAxis(sqrt(1 - u1) * cos(2 * M_PI * u2),
                        sqrt(u1) * sin(2 * M_PI * u3),
                        sqrt(u1) * cos(2 * M_PI * u3));

  return Magnum::Quaternion(qAxis, sqrt(1 - u1) * sin(2 * M_PI * u2));
}

template <typename T>
Magnum::Math::Matrix4<T> orthonormalizeRotationShear(
    const Magnum::Math::Matrix4<T>& transformation) {
  Magnum::Math::Matrix3<T> rotation = transformation.rotationShear();
  Magnum::Math::Algorithms::gramSchmidtOrthonormalizeInPlace(rotation);

  return Magnum::Math::Matrix4<T>::from(rotation,
                                        transformation.translation()) *
         Magnum::Math::Matrix4<T>::scaling(transformation.scaling());
}
}  // namespace core
}  // namespace esp

#endif  // ESP_CORE_UTILITY_H_
