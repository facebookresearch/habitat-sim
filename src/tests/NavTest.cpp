// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <gtest/gtest.h>
#include "esp/agent/Agent.h"
#include "esp/assets/SceneLoader.h"
#include "esp/core/esp.h"
#include "esp/core/random.h"
#include "esp/nav/ActionSpacePath.h"
#include "esp/nav/PathFinder.h"
#include "esp/scene/ObjectControls.h"
#include "esp/scene/SceneGraph.h"

using namespace esp;
using namespace esp::nav;

void printPathPoint(int run, int step, const vec3f& p, float distance) {
  LOG(INFO) << run << "," << step << "," << p[0] << "," << p[1] << "," << p[2]
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
      CHECK(islandSize > 0.0);
      for (int j = 0; j < path.points.size(); j++) {
        printPathPoint(i, j, path.points[j], path.geodesicDistance);
        CHECK(pf.islandRadius(path.points[j]) == islandSize);
      }
      CHECK(pf.islandRadius(path.requestedEnd) == islandSize);
      const vec3f& pathStart = path.points.front();
      const vec3f& pathEnd = path.points.back();
      const vec3f end = pf.tryStep(pathStart, pathEnd);
      LOG(INFO) << "tryStep initial end=" << pathEnd.transpose()
                << ", final end=" << end.transpose();
      CHECK(path.geodesicDistance < std::numeric_limits<float>::infinity());
    }
  }
}

TEST(NavTest, PathFinderLoadTest) {
  PathFinder pf;
  pf.loadNavMesh("test.navmesh");
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
      LOG(WARNING) << "Failed to find shortest path between start="
                   << path.requestedStart.transpose()
                   << " and end=" << path.requestedEnd.transpose();
    }
  }
}

TEST(NavTest, PathFinderTestCases) {
  PathFinder pf;
  pf.loadNavMesh("test.navmesh");
  ShortestPath testPath;
  testPath.requestedStart = vec3f(-6.493, 0.072, -3.292);
  testPath.requestedEnd = vec3f(-8.98, 0.072, -0.62);
  LOG(INFO) << "TEST";
  pf.findPath(testPath);
  CHECK(testPath.points.size() == 0);
  CHECK_EQ(testPath.geodesicDistance, std::numeric_limits<float>::infinity());

  testPath.requestedStart = pf.getRandomNavigablePoint();
  // Jitter the point just enough so that it isn't exactly the same
  testPath.requestedEnd = testPath.requestedStart + vec3f(0.01, 0.0, 0.01);
  pf.findPath(testPath);
  // There should be 2 points
  CHECK_EQ(testPath.points.size(), 2);
  // The geodesicDistance should be almost exactly the L2 dist
  CHECK_LE(std::abs(testPath.geodesicDistance -
                    (testPath.requestedStart - testPath.requestedEnd).norm()),
           0.001);
}

TEST(NavTest, BuildNavMeshFromMeshTest) {
  using namespace esp::assets;
  SceneLoader loader;
  const AssetInfo info = AssetInfo::fromPath("test.glb");
  const MeshData mesh = loader.load(info);
  NavMeshSettings bs;
  bs.setDefaults();
  PathFinder pf;
  pf.build(bs, mesh);
  testPathFinder(pf);
}

TEST(NavTest, ActionSpacePath) {
  PathFinder::ptr pf = PathFinder::create();
  pf->loadNavMesh("test.navmesh");
  pf->seed(0);
  agent::AgentConfiguration agentCfg;
  scene::ObjectControls controls_;
  const quatf startRotation(0.0, 0.0, 0.0, 1.0),
      goalRotation(0.0, 1.0, 0.0, 0.0);
  auto actPathfinder = nav::ActionSpacePathFinder::create_unique(
      pf, agentCfg, controls_, startRotation);

  constexpr int numTests = 5;
  for (int i = 0; i < numTests; ++i) {
    const vec3f goalPos = pf->getRandomNavigablePoint();
    ShortestPath testPath;
    do {
      testPath.requestedEnd = goalPos;
      testPath.requestedStart = pf->getRandomNavigablePoint();
      pf->findPath(testPath);
    } while (testPath.geodesicDistance > 10.0 ||
             testPath.geodesicDistance < 5.0);

    ActionSpaceShortestPath actTestPath, nextActTestPath;
    actTestPath.requestedStart->pos_ = testPath.requestedStart;
    actTestPath.requestedStart->rotation_ = startRotation.coeffs();
    actTestPath.requestedEnd->pos_ = testPath.requestedEnd;
    actTestPath.requestedEnd->rotation_ = goalRotation.coeffs();

    nextActTestPath.requestedStart->pos_ = testPath.requestedStart;
    nextActTestPath.requestedStart->rotation_ = startRotation.coeffs();
    nextActTestPath.requestedEnd->pos_ = testPath.requestedEnd;
    nextActTestPath.requestedEnd->rotation_ = goalRotation.coeffs();

    actPathfinder->paddingRadius(0.0);
    actPathfinder->findPath(actTestPath);

    LOG(INFO) << "testPath.geodesicDistance: " << testPath.geodesicDistance;
    LOG(INFO) << "actTestPath.geodesicDistance: "
              << actTestPath.geodesicDistance;
    CHECK(
        std::abs(actTestPath.geodesicDistance - testPath.geodesicDistance) /
            std::max(actTestPath.geodesicDistance, testPath.geodesicDistance) <
        0.1);

    actPathfinder->paddingRadius(0.2);
    actPathfinder->findPath(actTestPath);

    scene::SceneGraph dummyGraph;
    scene::SceneNode dummy(dummyGraph.getRootNode());
    dummy.setTranslation(testPath.requestedStart);
    dummy.setRotation(startRotation);
    for (const auto& act : actTestPath.actions) {
      nextActTestPath.requestedStart->pos_ = dummy.getAbsolutePosition();
      nextActTestPath.requestedStart->rotation_ = dummy.getRotation().coeffs();
      actPathfinder->findNextActionAlongPath(nextActTestPath);

      // Checks to make sure the predicted entire path is correct for each
      // individual loaction
      CHECK(act == nextActTestPath.actions[0]);

      const vec3f pos = dummy.getAbsolutePosition();
      const agent::ActionSpec::ptr& actionSpec = agentCfg.actionSpace[act];
      controls_.getMoveFuncMap().at(actionSpec->name)(
          dummy, actionSpec->actuation["amount"]);

      vec3f newPos = pf->tryStep(pos, dummy.getAbsolutePosition());
      dummy.setTranslation(newPos);
    }

    const double distToGoal =
        (actTestPath.requestedEnd->pos_ - dummy.getAbsolutePosition()).norm();
    const double angularDistToGoal =
        Eigen::Map<const quatf>(actTestPath.requestedEnd->rotation_.data())
            .angularDistance(dummy.getRotation());
    LOG(INFO) << "L2 Distance to goal after following path: " << distToGoal;
    LOG(INFO) << "Angular distance to goal after following path: "
              << angularDistToGoal;

    CHECK(distToGoal <
          agentCfg.actionSpace["moveForward"]->actuation["amount"]);
    CHECK(angularDistToGoal < 1e-3);
  }
};
