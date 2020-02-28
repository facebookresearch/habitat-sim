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
namespace Mn = Magnum;

using namespace esp;
using namespace esp::geo;

// reference: https://github.com/facebookresearch/habitat-sim/pull/496/files
namespace Test {
// standard method
// transform the 8 corners, and extract the min and max
Mn::Range3D getTransformedBB_standard(const Mn::Range3D& range,
                                      const Mn::Matrix4& xform) {
  std::vector<Mn::Vector3> corners;
  corners.push_back(xform.transformPoint(range.frontBottomLeft()));
  corners.push_back(xform.transformPoint(range.frontBottomRight()));
  corners.push_back(xform.transformPoint(range.frontTopLeft()));
  corners.push_back(xform.transformPoint(range.frontTopRight()));

  corners.push_back(xform.transformPoint(range.backTopLeft()));
  corners.push_back(xform.transformPoint(range.backTopRight()));
  corners.push_back(xform.transformPoint(range.backBottomLeft()));
  corners.push_back(xform.transformPoint(range.backBottomRight()));

  Mn::Range3D transformedBB{Mn::Math::minmax<Mn::Vector3>(corners)};

  return transformedBB;
}

struct GeoTest : Cr::TestSuite::Tester {
  explicit GeoTest();
  // tests
  void aabb();
  // benchmarks
  void getTransformedBB_standard();
  void getTransformedBB();

  std::vector<Mn::Matrix4> xforms_;
  // number of transformations
  const unsigned int numCases_ = 50000;
  // the batch size when running benchmarks
  const unsigned int iterations_ = 10;
  Mn::Range3D box_ = Mn::Range3D{Mn::Vector3{-10.0, -10.0, -10.0},
                                 Mn::Vector3{10.0, 10.0, 10.0}};
};

GeoTest::GeoTest() {
  // clang-format off
  addTests({&GeoTest::aabb});
  addBenchmarks({&GeoTest::getTransformedBB_standard,
                 &GeoTest::getTransformedBB}, 10);
  // clang-format on

  // Generate N transformations (random positions and orientations)
  xforms_.reserve(numCases_);
  for (unsigned int iTransform = 0; iTransform < numCases_; ++iTransform) {
    // generate random translation
    const Mn::Vector3 translation{
        static_cast<float>(rand() % 1000) + (rand() % 1000) / 1000.0f,
        static_cast<float>(rand() % 1000) + (rand() % 1000) / 1000.0f,
        static_cast<float>(rand() % 1000) + (rand() % 1000) / 1000.0f};

    // assemble the transformation matrix, and push it back
    xforms_.emplace_back(
        Mn::Matrix4::from(esp::core::randomRotation().toMatrix(), translation));
  }
}

void GeoTest::getTransformedBB_standard() {
  Mn::Range3D aabb;
  CORRADE_BENCHMARK(iterations_) for (auto& xform : xforms_) {
    aabb = Test::getTransformedBB_standard(box_, xform);
  }
}

void GeoTest::getTransformedBB() {
  Mn::Range3D aabb;
  CORRADE_BENCHMARK(iterations_) for (auto& xform : xforms_) {
    aabb = esp::geo::getTransformedBB(box_, xform);
  }
}

void GeoTest::aabb() {
  // compute aabb for each box using standard method and library method
  // respectively.
  // compare the results, which should be identical
  for (unsigned int iTransform = 0; iTransform < numCases_; ++iTransform) {
    Mn::Range3D aabbControl =
        Test::getTransformedBB_standard(box_, xforms_[iTransform]);
    Mn::Range3D aabbTest =
        esp::geo::getTransformedBB(box_, xforms_[iTransform]);

    float eps = 1e-8f;
    CORRADE_COMPARE_WITH((aabbTest.min() - aabbControl.min()).dot(), 0.0,
                         Cr::TestSuite::Compare::around(eps));
    CORRADE_COMPARE_WITH((aabbTest.max() - aabbControl.max()).dot(), 0.0,
                         Cr::TestSuite::Compare::around(eps));
  }
}
}  // namespace Test

CORRADE_TEST_MAIN(Test::GeoTest)
