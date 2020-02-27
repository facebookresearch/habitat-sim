// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/TestSuite/Compare/Numeric.h>
#include <Corrade/TestSuite/Tester.h>
#include <Magnum/Math/FunctionsBatch.h>
#include "esp/core/Utility.h"
#include "esp/geo/CoordinateFrame.h"
#include "esp/geo/OBB.h"
#include "esp/geo/geo.h"

// TODO: refactor the GeoTest to use Cr::TestSuite
namespace Cr = Corrade;

using namespace esp;
using namespace esp::geo;

// standard method
// transform the 8 corners, and extract the min and max
Magnum::Range3D getTransformedBB_standard(const Magnum::Range3D& range,
                                          const Magnum::Matrix4& xform) {
  std::vector<Magnum::Vector3> corners;
  corners.push_back(xform.transformPoint(range.frontBottomLeft()));
  corners.push_back(xform.transformPoint(range.frontBottomRight()));
  corners.push_back(xform.transformPoint(range.frontTopLeft()));
  corners.push_back(xform.transformPoint(range.frontTopRight()));

  corners.push_back(xform.transformPoint(range.backTopLeft()));
  corners.push_back(xform.transformPoint(range.backTopRight()));
  corners.push_back(xform.transformPoint(range.backBottomLeft()));
  corners.push_back(xform.transformPoint(range.backBottomRight()));

  Magnum::Range3D transformedBB{Magnum::Math::minmax<Magnum::Vector3>(corners)};

  return transformedBB;
}

// reference: https://github.com/facebookresearch/habitat-sim/pull/496/files
namespace {
struct GeoTest : Cr::TestSuite::Tester {
  explicit GeoTest();
  void aabb();
};

GeoTest::GeoTest() {
  // clang-format off
  addTests({&GeoTest::aabb});
  // clang-format on
}

void GeoTest::aabb() {
  // Generate N boxes in random position and orientation.
  // compute aabb for each box using standard method and library method
  // respectively.
  // compare the results, which should be identical

  Magnum::Range3D box = Magnum::Range3D{Magnum::Vector3{-10.0, -10.0, -10.0},
                                        Magnum::Vector3{10.0, 10.0, 10.0}};

  int numBoxes = 1000;
  for (int iBox = 0; iBox < numBoxes; ++iBox) {
    // generate random translation
    const Magnum::Vector3 translation{
        static_cast<float>(rand() % 1000) + (rand() % 1000) / 1000.0f,
        static_cast<float>(rand() % 1000) + (rand() % 1000) / 1000.0f,
        static_cast<float>(rand() % 1000) + (rand() % 1000) / 1000.0f};

    // assemble the transformation matrix
    Magnum::Matrix4 xform = Magnum::Matrix4::from(
        esp::core::randomRotation().toMatrix(), translation);

    Magnum::Range3D aabbControl = getTransformedBB_standard(box, xform);
    Magnum::Range3D aabbTest = esp::geo::getTransformedBB(box, xform);

    CORRADE_COMPARE(aabbTest, aabbControl);
  }  // for iBox
}

}  // namespace

CORRADE_TEST_MAIN(GeoTest)
