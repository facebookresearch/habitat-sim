// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Containers/Array.h>
#include <Corrade/TestSuite/Compare/Numeric.h>
#include <Corrade/TestSuite/Tester.h>
#include <Magnum/Math/FunctionsBatch.h>
#include <Magnum/Math/Matrix4.h>

#include "esp/gfx/DepthUnprojection.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace gfx {
namespace test {
namespace {

struct DepthUnprojectionTest : Cr::TestSuite::Tester {
  explicit DepthUnprojectionTest();

  void testCpu();

  void benchmarkBaseline();
  void benchmarkCpu();
};

using namespace Mn::Math::Literals;

const struct {
  const char* name;
  float depth, expected;
  Mn::Matrix4 projection;
} TestData[]{
    {"z=4, near=0.01, far=100", 4.0f, 4.0f,
     Mn::Matrix4::perspectiveProjection(60.0_degf, 1.0f, 0.01f, 100.0f)},
    {"z=4, near=1.0, far=100", 4.0f, 4.0f,
     Mn::Matrix4::perspectiveProjection(60.0_degf, 1.0f, 1.0f, 100.0f)},
    {"z=4, near=0.01, far=10", 4.0f, 4.0f,
     Mn::Matrix4::perspectiveProjection(60.0_degf, 1.0f, 0.01f, 10.0f)},
    {"z=0.015, near=0.015, far=100", 0.015f, 0.015f,
     Mn::Matrix4::perspectiveProjection(60.0_degf, 1.0f, 0.01f, 100.0f)},
    {"z=10, near=0.01, far=100", 10.0f, 10.0f,
     Mn::Matrix4::perspectiveProjection(60.0_degf, 1.0f, 0.01f, 100.0f)},
    {"z=95, near=0.01, far=100", 95.0f, 95.0f,
     Mn::Matrix4::perspectiveProjection(60.0_degf, 1.0f, 0.01f, 100.0f)},
    {"z=near", 0.01f, 0.01f,
     Mn::Matrix4::perspectiveProjection(60.0_degf, 1.0f, 0.01f, 100.0f)},
    {"z=far", 100.0f, 0.0f,
     Mn::Matrix4::perspectiveProjection(60.0_degf, 1.0f, 0.01f, 100.0f)},
};

CORRADE_NEVER_INLINE void unprojectBaseline(
    Cr::Containers::ArrayView<float> depth,
    const Mn::Matrix4& unprojection) {
  for (float& d : depth) {
    if (d == 1.0f) {
      d = 0.0f;
      continue;
    }
    d = -unprojection.transformPoint(Mn::Vector3::zAxis(-d)).z();
  }
}

CORRADE_NEVER_INLINE void unprojectBaselineNoBranch(
    Cr::Containers::ArrayView<float> depth,
    const Mn::Matrix4& unprojection) {
  for (float& d : depth) {
    d = -unprojection.transformPoint(Mn::Vector3::zAxis(-d)).z();
  }
}

const struct {
  const char* name;
  void (*unprojector)(Cr::Containers::ArrayView<float>, const Mn::Matrix4&);
} UnprojectBenchmarkData[]{
    {"", unprojectBaseline},
    {"no branch", unprojectBaselineNoBranch},
};

DepthUnprojectionTest::DepthUnprojectionTest() {
  addInstancedTests({&DepthUnprojectionTest::testCpu},
                    Cr::Containers::arraySize(TestData));

  addInstancedBenchmarks({&DepthUnprojectionTest::benchmarkBaseline}, 10,
                         Cr::Containers::arraySize(UnprojectBenchmarkData));

  addBenchmarks({&DepthUnprojectionTest::benchmarkCpu}, 10);
}

void DepthUnprojectionTest::testCpu() {
  auto&& data = TestData[testCaseInstanceId()];
  setTestCaseDescription(data.name);

  Mn::Vector3 projected =
      data.projection.transformPoint({0.95f, -0.34f, -data.depth});
  CORRADE_COMPARE_WITH(
      -data.projection.inverted().transformPoint(projected).z(), data.depth,
      Cr::TestSuite::Compare::around(data.depth * 0.0002f));

  float depth[] = {Mn::Math::lerpInverted(-1.0f, 1.0f, projected.z())};
  unprojectDepth(calculateDepthUnprojection(data.projection), depth);
  CORRADE_COMPARE_WITH(depth[0], data.expected,
                       Cr::TestSuite::Compare::around(data.depth * 0.0002f));
}

constexpr Mn::Vector2i BenchmarkSize{1536};

void DepthUnprojectionTest::benchmarkBaseline() {
  auto&& data = UnprojectBenchmarkData[testCaseInstanceId()];
  setTestCaseDescription(data.name);

  Mn::Matrix4 unprojection =
      Mn::Matrix4::perspectiveProjection(60.0_degf, 1.0f, 0.001f, 100.0f)
          .inverted();

  Cr::Containers::Array<float> depth{Cr::Containers::NoInit,
                                     std::size_t(BenchmarkSize.product())};
  for (std::size_t i = 0; i != depth.size(); ++i)
    depth[i] = float(2 * (i % 10000)) / float(10000) - 1.0f;

  CORRADE_BENCHMARK(1) { data.unprojector(depth, unprojection); }

  CORRADE_COMPARE_AS(Mn::Math::max<float>(depth), 9.0f,
                     Cr::TestSuite::Compare::Greater);
}

void DepthUnprojectionTest::benchmarkCpu() {
  Mn::Vector2 unprojection = calculateDepthUnprojection(
      Mn::Matrix4::perspectiveProjection(60.0_degf, 1.0f, 0.001f, 100.0f));

  Cr::Containers::Array<float> depth{Cr::Containers::NoInit,
                                     std::size_t(BenchmarkSize.product())};
  for (std::size_t i = 0; i != depth.size(); ++i)
    depth[i] = float(i % 10000) / float(10000);

  CORRADE_BENCHMARK(1) { unprojectDepth(unprojection, depth); }

  CORRADE_COMPARE_AS(Mn::Math::max<float>(depth), 9.0f,
                     Cr::TestSuite::Compare::Greater);
}

}  // namespace
}  // namespace test
}  // namespace gfx
}  // namespace esp

CORRADE_TEST_MAIN(esp::gfx::test::DepthUnprojectionTest)
