// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Containers/ArrayViewStl.h>
#include <Corrade/TestSuite/Compare/Numeric.h>
#include <Corrade/TestSuite/Tester.h>
#include <Corrade/Utility/DebugStl.h>
#include <Magnum/EigenIntegration/Integration.h>
#include <Magnum/Math/FunctionsBatch.h>
#include "esp/core/Utility.h"
#include "esp/geo/CoordinateFrame.h"
#include "esp/geo/Geo.h"
#include "esp/geo/OBB.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

using namespace esp;
using namespace esp::geo;

// reference: https://github.com/facebookresearch/habitat-sim/pull/496/files
namespace {

struct GeoTest : Cr::TestSuite::Tester {
  explicit GeoTest();
  // tests
  void aabb();
  void obbConstruction();
  void obbFunctions();
  void coordinateFrame();
  // benchmarks
  void getTransformedBB_standard();
  void getTransformedBB();
  // standard method
  // transform the 8 corners, and extract the min and max
  Mn::Range3D getTransformedBB_standard(const Mn::Range3D& range,
                                        const Mn::Matrix4& xform);

  std::vector<Mn::Matrix4> xforms_;
  // number of transformations
  const unsigned int numCases_ = 50000;
  // the batch size when running benchmarks
  const unsigned int iterations_ = 10;
  Mn::Range3D box_{Mn::Vector3{-10.0f, -10.0f, -10.0f},
                   Mn::Vector3{10.0f, 10.0f, 10.0f}};
  esp::logging::LoggingContext loggingContext_;
};

GeoTest::GeoTest() {
  // clang-format off
  addTests({&GeoTest::aabb,
            &GeoTest::obbConstruction,
            &GeoTest::obbFunctions,
            &GeoTest::coordinateFrame});
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

// standard method
// transform the 8 corners, and extract the min and max
Mn::Range3D GeoTest::getTransformedBB_standard(const Mn::Range3D& range,
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

  Mn::Range3D transformedBB{Mn::Math::minmax(corners)};

  return transformedBB;
}

void GeoTest::getTransformedBB_standard() {
  Mn::Range3D aabb;
  CORRADE_BENCHMARK(iterations_) for (auto& xform : xforms_) {
    aabb = getTransformedBB_standard(box_, xform);
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
  for (auto& xform : xforms_) {
    Mn::Range3D aabbControl = getTransformedBB_standard(box_, xform);
    Mn::Range3D aabbTest = esp::geo::getTransformedBB(box_, xform);

    float eps = 1e-8f;
    CORRADE_COMPARE_WITH(aabbTest.min(), aabbControl.min(),
                         Cr::TestSuite::Compare::around(Mn::Vector3{eps}));
    CORRADE_COMPARE_WITH(aabbTest.max(), aabbControl.max(),
                         Cr::TestSuite::Compare::around(Mn::Vector3{eps}));
  }
}

void GeoTest::obbConstruction() {
  OBB obb1;
  const vec3f center(0, 0, 0);
  const vec3f dimensions(20, 2, 10);
  const quatf rot1 = quatf::FromTwoVectors(vec3f::UnitY(), vec3f::UnitZ());
  OBB obb2(center, dimensions, rot1);

  CORRADE_VERIFY(obb2.center().isApprox(center));
  CORRADE_VERIFY(obb2.sizes().isApprox(dimensions));
  CORRADE_VERIFY(obb2.halfExtents().isApprox(dimensions / 2));
  CORRADE_VERIFY(obb2.rotation().coeffs().isApprox(rot1.coeffs()));

  box3f aabb(vec3f(-1, -2, -3), vec3f(3, 2, 1));
  OBB obb3(aabb);
  CORRADE_VERIFY(obb3.center().isApprox(aabb.center()));
  CORRADE_VERIFY(obb3.toAABB().isApprox(aabb));
}

void GeoTest::obbFunctions() {
  const vec3f center(0, 0, 0);
  const vec3f dimensions(20, 2, 10);
  const quatf rot1 = quatf::FromTwoVectors(vec3f::UnitY(), vec3f::UnitZ());
  OBB obb2(center, dimensions, rot1);
  CORRADE_VERIFY(obb2.contains(vec3f(0, 0, 0)));
  CORRADE_VERIFY(obb2.contains(vec3f(-5, -2, 0.5)));
  CORRADE_VERIFY(obb2.contains(vec3f(5, 0, -0.5)));
  CORRADE_VERIFY(!obb2.contains(vec3f(5, 0, 2)));
  CORRADE_VERIFY(!obb2.contains(vec3f(-10, 0.5, -2)));

  const box3f aabb = obb2.toAABB();

  CORRADE_COMPARE(Mn::Vector3{aabb.min()}, (Mn::Vector3{-10.0f, -5.0f, -1.0f}));
  CORRADE_COMPARE(Mn::Vector3{aabb.max()}, (Mn::Vector3{10.0f, 5.0f, 1.0f}));

  const Transform identity = obb2.worldToLocal() * obb2.localToWorld();
  CORRADE_VERIFY(identity.isApprox(Transform::Identity()));
  CORRADE_VERIFY(obb2.contains(obb2.localToWorld() * vec3f(0, 0, 0)));
  CORRADE_VERIFY(obb2.contains(obb2.localToWorld() * vec3f(1, -1, 1)));
  CORRADE_VERIFY(obb2.contains(obb2.localToWorld() * vec3f(-1, -1, -1)));
  CORRADE_VERIFY(!obb2.contains(obb2.localToWorld() * vec3f(-1, -1.5, -1)));
  CORRADE_COMPARE_AS(obb2.distance(vec3f(-20, 0, 0)), 10, float);
  CORRADE_COMPARE_AS(obb2.distance(vec3f(-10, -5, 2)), 1, float);
}

void GeoTest::coordinateFrame() {
  const vec3f origin(1, -2, 3);
  const vec3f up(0, 0, 1);
  const vec3f front(-1, 0, 0);
  quatf rotation = quatf::FromTwoVectors(ESP_UP, up) *
                   quatf::FromTwoVectors(ESP_FRONT, front);
  Transform xform;
  xform.rotate(rotation);
  xform.translate(origin);

  CoordinateFrame c1(up, front, origin);
  CORRADE_VERIFY(c1.up().isApprox(up));
  CORRADE_VERIFY(c1.gravity().isApprox(-up));
  CORRADE_VERIFY(c1.front().isApprox(front));
  CORRADE_VERIFY(c1.back().isApprox(-front));
  CORRADE_VERIFY(c1.up().isApprox(rotation * ESP_UP));
  CORRADE_VERIFY(c1.front().isApprox(rotation * ESP_FRONT));
  CORRADE_VERIFY(c1.origin().isApprox(origin));
  CORRADE_VERIFY(c1.rotationWorldToFrame().isApprox(rotation));

  CoordinateFrame c2(rotation, origin);
  CORRADE_VERIFY(c1 == c2);
  CORRADE_VERIFY(c2.up().isApprox(up));
  CORRADE_VERIFY(c2.gravity().isApprox(-up));
  CORRADE_VERIFY(c2.front().isApprox(front));
  CORRADE_VERIFY(c2.back().isApprox(-front));
  CORRADE_VERIFY(c2.up().isApprox(rotation * ESP_UP));
  CORRADE_VERIFY(c2.front().isApprox(rotation * ESP_FRONT));
  CORRADE_VERIFY(c2.origin().isApprox(origin));
  CORRADE_VERIFY(c2.rotationWorldToFrame().isApprox(rotation));

  const std::string j = R"({"up":[0,0,1],"front":[-1,0,0],"origin":[1,-2,3]})";
  CORRADE_COMPARE(c1.toString(), j);
}

}  // namespace

CORRADE_TEST_MAIN(GeoTest)
