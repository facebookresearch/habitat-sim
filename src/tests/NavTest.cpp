// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/TestSuite/Compare/Numeric.h>
#include <Corrade/TestSuite/Tester.h>
#include <Corrade/Utility/Path.h>

#include "esp/assets/MeshData.h"
#include "esp/core/Esp.h"
#include "esp/core/Random.h"
#include "esp/nav/PathFinder.h"
#include "esp/scene/SceneGraph.h"

#include "configure.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace {

void printPathPoint(int run, int step, const Mn::Vector3& p, float distance) {
  ESP_VERY_VERBOSE() << run << "," << step << "," << Mn::Vector3{p} << ","
                     << distance;
}

struct NavTest : Cr::TestSuite::Tester {
  explicit NavTest();

  void PathFinderLoadTest();
  void PathFinderTestCases();

  void PathFinderTestNonNavigable();
  void PathFinderTestSeed();

  void PathFinderTestMeshData();
  esp::logging::LoggingContext loggingContext;
};

NavTest::NavTest() {
  addTests({&NavTest::PathFinderLoadTest, &NavTest::PathFinderTestCases,
            &NavTest::PathFinderTestNonNavigable, &NavTest::PathFinderTestSeed,
            &NavTest::PathFinderTestMeshData});
}  // namespace Test

void NavTest::PathFinderLoadTest() {
  esp::nav::PathFinder pf;
  pf.loadNavMesh(Cr::Utility::Path::join(
      SCENE_DATASETS, "habitat-test-scenes/skokloster-castle.navmesh"));
  for (int i = 0; i < 100000; ++i) {
    esp::nav::ShortestPath path;
    path.requestedStart = pf.getRandomNavigablePoint();
    path.requestedEnd = pf.getRandomNavigablePoint();
    const bool foundPath = pf.findPath(path);
    if (foundPath) {
      const float islandSize = pf.islandRadius(path.requestedStart);
      CORRADE_COMPARE_AS(islandSize, 0.0, Cr::TestSuite::Compare::Greater);
      for (int j = 0; j < path.points.size(); ++j) {
        printPathPoint(i, j, path.points[j], path.geodesicDistance);
        CORRADE_COMPARE(pf.islandRadius(path.points[j]), islandSize);
      }
      CORRADE_COMPARE(pf.islandRadius(path.requestedEnd), islandSize);
      const Mn::Vector3& pathStart = path.points.front();
      const Mn::Vector3& pathEnd = path.points.back();
      const Mn::Vector3 end = pf.tryStep(pathStart, pathEnd);
      ESP_DEBUG() << "tryStep initial end=" << pathEnd << ", final end=" << end;
      CORRADE_COMPARE_AS(path.geodesicDistance,
                         std::numeric_limits<float>::infinity(),
                         Cr::TestSuite::Compare::Less);
    }
  }
}

void printRandomizedPathSet(esp::nav::PathFinder& pf) {
  esp::core::Random random;
  esp::nav::ShortestPath path;
  path.requestedStart = pf.getRandomNavigablePoint();
  path.requestedEnd = pf.getRandomNavigablePoint();
  ESP_VERY_VERBOSE() << "run,step,x,y,z,geodesicDistance";
  for (int i = 0; i < 100; ++i) {
    const float r = 0.1;
    Mn::Vector3 rv(random.uniform_float(-r, r), 0, random.uniform_float(-r, r));
    Mn::Vector3 rv2(random.uniform_float(-r, r), 0,
                    random.uniform_float(-r, r));
    path.requestedStart += rv;
    path.requestedEnd += rv2;
    const bool foundPath = pf.findPath(path);

    if (foundPath) {
      printPathPoint(i, 0, path.requestedStart, path.geodesicDistance);
      for (int j = 0; j < path.points.size(); ++j) {
        printPathPoint(i, j + 1, path.points[j], path.geodesicDistance);
      }
      printPathPoint(i, path.points.size() + 1, path.requestedEnd,
                     path.geodesicDistance);
    } else {
      ESP_WARNING() << "Failed to find shortest path between start="
                    << path.requestedStart << "and end=" << path.requestedEnd;
    }
  }
}

void NavTest::PathFinderTestCases() {
  esp::nav::PathFinder pf;
  pf.loadNavMesh(Cr::Utility::Path::join(
      SCENE_DATASETS, "habitat-test-scenes/skokloster-castle.navmesh"));
  esp::nav::ShortestPath testPath;
  testPath.requestedStart = Mn::Vector3(-6.493, 0.072, -3.292);
  testPath.requestedEnd = Mn::Vector3(-8.98, 0.072, -0.62);
  pf.findPath(testPath);
  CORRADE_COMPARE(testPath.points.size(), 0);
  CORRADE_COMPARE(testPath.geodesicDistance,
                  std::numeric_limits<float>::infinity());

  testPath.requestedStart = pf.getRandomNavigablePoint();
  // Jitter the point just enough so that it isn't exactly the same
  testPath.requestedEnd =
      testPath.requestedStart + Mn::Vector3(0.01, 0.0, 0.01);
  pf.findPath(testPath);
  // There should be 2 points
  CORRADE_COMPARE(testPath.points.size(), 2);
  // The geodesicDistance should be almost exactly the L2 dist
  CORRADE_COMPARE_AS(
      std::abs(testPath.geodesicDistance -
               (testPath.requestedStart - testPath.requestedEnd).length()),
      0.001, Cr::TestSuite::Compare::LessOrEqual);
}

void NavTest::PathFinderTestNonNavigable() {
  esp::nav::PathFinder pf;
  pf.loadNavMesh(Cr::Utility::Path::join(
      SCENE_DATASETS, "habitat-test-scenes/skokloster-castle.navmesh"));

  const Mn::Vector3 nonNavigablePoint{1e2, 1e2, 1e2};

  const Mn::Vector3 resultPoint = pf.tryStep<Mn::Vector3>(
      nonNavigablePoint, nonNavigablePoint + Mn::Vector3{0.25, 0, 0});

  CORRADE_VERIFY(nonNavigablePoint == resultPoint);
}

void NavTest::PathFinderTestSeed() {
  esp::nav::PathFinder pf;
  pf.loadNavMesh(Cr::Utility::Path::join(
      SCENE_DATASETS, "habitat-test-scenes/skokloster-castle.navmesh"));

  // The same seed should produce the same point
  pf.seed(1);
  Mn::Vector3 firstPoint = pf.getRandomNavigablePoint();
  pf.seed(1);
  Mn::Vector3 secondPoint = pf.getRandomNavigablePoint();
  CORRADE_COMPARE(firstPoint, secondPoint);

  // Different seeds should produce different points
  pf.seed(2);
  Mn::Vector3 firstPoint2 = pf.getRandomNavigablePoint();
  pf.seed(3);
  Mn::Vector3 secondPoint2 = pf.getRandomNavigablePoint();
  CORRADE_COMPARE_AS(firstPoint2, secondPoint2,
                     Cr::TestSuite::Compare::NotEqual);

  // One seed should produce different points when sampled twice
  pf.seed(4);
  Mn::Vector3 firstPoint3 = pf.getRandomNavigablePoint();
  Mn::Vector3 secondPoint3 = pf.getRandomNavigablePoint();
  CORRADE_COMPARE_AS(firstPoint3, secondPoint3,
                     Cr::TestSuite::Compare::NotEqual);
}

void NavTest::PathFinderTestMeshData() {
  esp::nav::PathFinder pf;
  pf.loadNavMesh(Cr::Utility::Path::join(
      SCENE_DATASETS, "habitat-test-scenes/skokloster-castle.navmesh"));

  esp::assets::MeshData::ptr meshData = pf.getNavMeshData();

  // Will change if asset changes
  CORRADE_COMPARE(meshData->vbo.size(), 1155);
  CORRADE_COMPARE(meshData->ibo.size(), 1155);

  pf.loadNavMesh(Cr::Utility::Path::join(
      SCENE_DATASETS, "habitat-test-scenes/van-gogh-room.navmesh"));

  meshData = pf.getNavMeshData();

  // Will change if asset changes
  CORRADE_COMPARE(meshData->vbo.size(), 63);
  CORRADE_COMPARE(meshData->ibo.size(), 63);
}
}  // namespace

CORRADE_TEST_MAIN(NavTest)
