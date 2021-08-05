// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Utility/Directory.h>
#include <gtest/gtest.h>

#include "esp/assets/MeshData.h"
#include "esp/core/esp.h"
#include "esp/core/random.h"
#include "esp/nav/PathFinder.h"
#include "esp/scene/ObjectControls.h"
#include "esp/scene/SceneGraph.h"

#include "configure.h"

namespace Cr = Corrade;

namespace esp {
namespace nav {

void printPathPoint(int run, int step, const vec3f& p, float distance) {
  ESP_DEBUG() << run << "," << step << "," << p[0] << "," << p[1] << "," << p[2]
              << "," << distance;
}

void testPathFinder(PathFinder& pf) {
  for (int i = 0; i < 100000; i++) {
    ShortestPath path;
    path.requestedStart = pf.getRandomNavigablePoint();
    path.requestedEnd = pf.getRandomNavigablePoint();
    const bool foundPath = pf.findPath(path);
    if (foundPath) {
      const float islandSize = pf.islandRadius(path.requestedStart);
      CORRADE_INTERNAL_ASSERT(islandSize > 0.0);
      for (int j = 0; j < path.points.size(); j++) {
        printPathPoint(i, j, path.points[j], path.geodesicDistance);
        CORRADE_INTERNAL_ASSERT(pf.islandRadius(path.points[j]) == islandSize);
      }
      CORRADE_INTERNAL_ASSERT(pf.islandRadius(path.requestedEnd) == islandSize);
      const vec3f& pathStart = path.points.front();
      const vec3f& pathEnd = path.points.back();
      const vec3f end = pf.tryStep(pathStart, pathEnd);
      ESP_DEBUG() << "tryStep initial end=" << pathEnd.transpose()
                  << ", final end=" << end.transpose();
      CORRADE_INTERNAL_ASSERT(path.geodesicDistance <
                              std::numeric_limits<float>::infinity());
    }
  }
}

TEST(NavTest, PathFinderLoadTest) {
  logging::LoggingContext loggingContext;
  PathFinder pf;
  pf.loadNavMesh(Cr::Utility::Directory::join(
      SCENE_DATASETS, "habitat-test-scenes/skokloster-castle.navmesh"));
  testPathFinder(pf);
}

void printRandomizedPathSet(PathFinder& pf) {
  core::Random random;
  ShortestPath path;
  path.requestedStart = pf.getRandomNavigablePoint();
  path.requestedEnd = pf.getRandomNavigablePoint();
  std::cout << "run,step,x,y,z,geodesicDistance" << std::endl;
  for (int i = 0; i < 100; i++) {
    const float r = 0.1;
    vec3f rv(random.uniform_float(-r, r), 0, random.uniform_float(-r, r));
    vec3f rv2(random.uniform_float(-r, r), 0, random.uniform_float(-r, r));
    path.requestedStart += rv;
    path.requestedEnd += rv2;
    const bool foundPath = pf.findPath(path);

    if (foundPath) {
      printPathPoint(i, 0, path.requestedStart, path.geodesicDistance);
      for (int j = 0; j < path.points.size(); j++) {
        printPathPoint(i, j + 1, path.points[j], path.geodesicDistance);
      }
      printPathPoint(i, path.points.size() + 1, path.requestedEnd,
                     path.geodesicDistance);
    } else {
      ESP_WARNING() << "Failed to find shortest path between start="
                    << path.requestedStart.transpose()
                    << "and end=" << path.requestedEnd.transpose();
    }
  }
}

TEST(NavTest, PathFinderTestCases) {
  logging::LoggingContext loggingContext;
  PathFinder pf;
  pf.loadNavMesh(Cr::Utility::Directory::join(
      SCENE_DATASETS, "habitat-test-scenes/skokloster-castle.navmesh"));
  ShortestPath testPath;
  testPath.requestedStart = vec3f(-6.493, 0.072, -3.292);
  testPath.requestedEnd = vec3f(-8.98, 0.072, -0.62);
  ESP_DEBUG() << "TEST";
  pf.findPath(testPath);
  CORRADE_INTERNAL_ASSERT(testPath.points.size() == 0);
  ASSERT_EQ(testPath.geodesicDistance, std::numeric_limits<float>::infinity());

  testPath.requestedStart = pf.getRandomNavigablePoint();
  // Jitter the point just enough so that it isn't exactly the same
  testPath.requestedEnd = testPath.requestedStart + vec3f(0.01, 0.0, 0.01);
  pf.findPath(testPath);
  // There should be 2 points
  ASSERT_EQ(testPath.points.size(), 2);
  // The geodesicDistance should be almost exactly the L2 dist
  ASSERT_LE(std::abs(testPath.geodesicDistance -
                     (testPath.requestedStart - testPath.requestedEnd).norm()),
            0.001);
}

TEST(NavTest, PathFinderTestNonNavigable) {
  logging::LoggingContext loggingContext;
  PathFinder pf;
  pf.loadNavMesh(Cr::Utility::Directory::join(
      SCENE_DATASETS, "habitat-test-scenes/skokloster-castle.navmesh"));

  const vec3f nonNavigablePoint{1e2, 1e2, 1e2};

  const vec3f resultPoint = pf.tryStep<vec3f>(
      nonNavigablePoint, nonNavigablePoint + vec3f{0.25, 0, 0});

  CORRADE_INTERNAL_ASSERT(nonNavigablePoint.isApprox(resultPoint));
}

TEST(NavTest, PathFinderTestSeed) {
  logging::LoggingContext loggingContext;
  PathFinder pf;
  pf.loadNavMesh(Cr::Utility::Directory::join(
      SCENE_DATASETS, "habitat-test-scenes/skokloster-castle.navmesh"));

  // The same seed should produce the same point
  pf.seed(1);
  vec3f firstPoint = pf.getRandomNavigablePoint();
  pf.seed(1);
  vec3f secondPoint = pf.getRandomNavigablePoint();
  ASSERT_TRUE(firstPoint == secondPoint);

  // Different seeds should produce different points
  pf.seed(2);
  vec3f firstPoint2 = pf.getRandomNavigablePoint();
  pf.seed(3);
  vec3f secondPoint2 = pf.getRandomNavigablePoint();
  ASSERT_TRUE(firstPoint2 != secondPoint2);

  // One seed should produce different points when sampled twice
  pf.seed(4);
  vec3f firstPoint3 = pf.getRandomNavigablePoint();
  vec3f secondPoint3 = pf.getRandomNavigablePoint();
  ASSERT_TRUE(firstPoint3 != secondPoint3);
}

TEST(NavTest, PathFinderTestMeshData) {
  logging::LoggingContext loggingContext;
  PathFinder pf;
  pf.loadNavMesh(Cr::Utility::Directory::join(
      SCENE_DATASETS, "habitat-test-scenes/skokloster-castle.navmesh"));

  esp::assets::MeshData::ptr meshData = pf.getNavMeshData();

  // Will change if asset changes
  ASSERT_EQ(meshData->vbo.size(), 1155);
  ASSERT_EQ(meshData->ibo.size(), 1155);

  pf.loadNavMesh(Cr::Utility::Directory::join(
      SCENE_DATASETS, "habitat-test-scenes/van-gogh-room.navmesh"));

  meshData = pf.getNavMeshData();

  // Will change if asset changes
  ASSERT_EQ(meshData->vbo.size(), 63);
  ASSERT_EQ(meshData->ibo.size(), 63);
}
}  // namespace nav
}  // namespace esp
