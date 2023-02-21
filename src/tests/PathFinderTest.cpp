// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Containers/ArrayView.h>
#include <Corrade/TestSuite/Compare/Numeric.h>
#include <Corrade/TestSuite/Tester.h>

#include <esp/nav/PathFinder.h>

#include <Corrade/Utility/Path.h>
#include <Magnum/EigenIntegration/Integration.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/Swizzle.h>
#include <Magnum/Math/Vector3.h>

#include "configure.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace {

const std::string skokloster =
    Cr::Utility::Path::join(SCENE_DATASETS,
                            "habitat-test-scenes/skokloster-castle.navmesh");

constexpr struct {
  const char* name;
  bool cache;
} MultiGoalBenchMarkData[]{{"path to closest of 1000", false},
                           {"cached path to closest of 1000", true}};

struct PathFinderTest : Cr::TestSuite::Tester {
  explicit PathFinderTest();

  void bounds();
  void tryStepNoSliding();
  void multiGoalPath();

  void benchmarkSingleGoal();
  void benchmarkMultiGoal();

  void testCaching();

  void navMeshSettingsTestJSON();

  esp::logging::LoggingContext loggingContext;
};

PathFinderTest::PathFinderTest() {
  addTests({&PathFinderTest::bounds, &PathFinderTest::tryStepNoSliding,
            &PathFinderTest::multiGoalPath, &PathFinderTest::testCaching,
            &PathFinderTest::navMeshSettingsTestJSON});

  addBenchmarks({&PathFinderTest::benchmarkSingleGoal}, 1000);
  addInstancedBenchmarks({&PathFinderTest::benchmarkMultiGoal}, 100,
                         Cr::Containers::arraySize(MultiGoalBenchMarkData));
}

void PathFinderTest::bounds() {
  esp::nav::PathFinder pathFinder;
  pathFinder.loadNavMesh(skokloster);
  CORRADE_VERIFY(pathFinder.isLoaded());

  Mn::Vector3 minExpected{-9.75916f, -0.390081f, 0.973853f};
  Mn::Vector3 maxExpected{8.56903f, 6.43441f, 25.5983f};

  std::pair<esp::vec3f, esp::vec3f> bounds = pathFinder.bounds();

  CORRADE_COMPARE(Mn::Vector3{bounds.first}, minExpected);
  CORRADE_COMPARE(Mn::Vector3{bounds.second}, maxExpected);
}

void PathFinderTest::tryStepNoSliding() {
  esp::nav::PathFinder pathFinder;
  pathFinder.loadNavMesh(skokloster);
  CORRADE_VERIFY(pathFinder.isLoaded());
  pathFinder.seed(0);

  Mn::Vector3 stepDir{0, 0, -1};
  for (int i = 0; i < 100; ++i) {
    Mn::Vector3 pos{pathFinder.getRandomNavigablePoint()};
    for (int j = 0; j < 10; ++j) {
      Mn::Vector3 targetPos = pos + stepDir;
      Mn::Vector3 actualEnd = pathFinder.tryStepNoSliding(pos, targetPos);
      CORRADE_VERIFY(pathFinder.isNavigable(
          Mn::EigenIntegration::cast<esp::vec3f>(actualEnd)));

      // The test becomes unreliable if we moved a very small distance
      if (Mn::Math::gather<'x', 'z'>(actualEnd - pos).dot() < 1e-5)
        break;

      Mn::Deg stepAngle = Mn::Math::angle(
          Mn::Math::gather<'x', 'z'>(stepDir),
          Mn::Math::gather<'x', 'z'>(actualEnd - pos).normalized());
      CORRADE_COMPARE_AS(stepAngle, Mn::Deg{1}, Cr::TestSuite::Compare::Less);

      pos = actualEnd;
    }
  }
}

void PathFinderTest::multiGoalPath() {
  esp::nav::PathFinder pathFinder;
  pathFinder.loadNavMesh(skokloster);
  CORRADE_VERIFY(pathFinder.isLoaded());
  pathFinder.seed(0);

  for (int __j = 0; __j < 1000; ++__j) {
    std::vector<esp::vec3f> points;
    points.reserve(10);
    for (int i = 0; i < 10; ++i) {
      points.emplace_back(pathFinder.getRandomNavigablePoint());
    }

    esp::nav::MultiGoalShortestPath multiPath;
    multiPath.requestedStart = points[0];
    multiPath.setRequestedEnds({points.begin() + 1, points.end()});

    CORRADE_VERIFY(pathFinder.findPath(multiPath));

    esp::nav::MultiGoalShortestPath path;
    path.requestedStart = points[0];
    float trueMinDist = 1e5;
    for (int i = 1; i < points.size(); ++i) {
      path.setRequestedEnds({points[i]});

      CORRADE_VERIFY(pathFinder.findPath(path));

      trueMinDist = std::min(trueMinDist, path.geodesicDistance);
    }

    CORRADE_COMPARE(multiPath.geodesicDistance, trueMinDist);
  }
}

void PathFinderTest::testCaching() {
  esp::nav::PathFinder pathFinder;
  pathFinder.loadNavMesh(skokloster);
  CORRADE_VERIFY(pathFinder.isLoaded());

  esp::nav::MultiGoalShortestPath cachePath;
  {
    std::vector<esp::vec3f> rqEnds;
    rqEnds.reserve(25);
    for (int i = 0; i < 25; ++i) {
      rqEnds.emplace_back(pathFinder.getRandomNavigablePoint());
    }
    cachePath.setRequestedEnds(rqEnds);
  }

  for (int i = 0; i < 1000; ++i) {
    CORRADE_ITERATION(i);

    cachePath.requestedStart = pathFinder.getRandomNavigablePoint();
    pathFinder.findPath(cachePath);

    esp::nav::MultiGoalShortestPath noCachePath;
    noCachePath.setRequestedEnds(cachePath.getRequestedEnds());
    noCachePath.requestedStart = cachePath.requestedStart;
    pathFinder.findPath(noCachePath);

    CORRADE_COMPARE(cachePath.geodesicDistance, noCachePath.geodesicDistance);
  }
}

void PathFinderTest::benchmarkSingleGoal() {
  esp::nav::PathFinder pathFinder;
  pathFinder.loadNavMesh(skokloster);
  CORRADE_VERIFY(pathFinder.isLoaded());

  esp::nav::ShortestPath path;
  path.requestedStart = pathFinder.getRandomNavigablePoint();
  do {
    path.requestedStart = pathFinder.getRandomNavigablePoint();
  } while (pathFinder.islandRadius(path.requestedStart) < 10.0);
  path.requestedEnd = pathFinder.getRandomNavigablePoint();

  bool status = false;
  CORRADE_BENCHMARK(5) { status = pathFinder.findPath(path); };
  CORRADE_VERIFY(status);
}

void PathFinderTest::benchmarkMultiGoal() {
  esp::nav::PathFinder pathFinder;
  pathFinder.loadNavMesh(skokloster);
  CORRADE_VERIFY(pathFinder.isLoaded());

  auto&& data = MultiGoalBenchMarkData[testCaseInstanceId()];
  setTestCaseDescription(data.name);

  esp::nav::MultiGoalShortestPath path;
  do {
    path.requestedStart = pathFinder.getRandomNavigablePoint();
  } while (pathFinder.islandRadius(path.requestedStart) < 10.0);

  std::vector<esp::vec3f> rqEnds;
  rqEnds.reserve(1000);
  for (int i = 0; i < 1000; ++i) {
    rqEnds.emplace_back(pathFinder.getRandomNavigablePoint());
  }
  path.setRequestedEnds(rqEnds);

  if (data.cache) {
    pathFinder.findPath(path);
  }

  bool status = false;
  CORRADE_BENCHMARK(1) { status = pathFinder.findPath(path); };
  CORRADE_VERIFY(status);
}

void PathFinderTest::navMeshSettingsTestJSON() {
  esp::nav::NavMeshSettings navmeshSettings;

  // load test settings
  navmeshSettings.readFromJSON(
      Cr::Utility::Path::join(TEST_ASSETS, "test_navmeshsettings.json"));

  // check against expected values
  esp::nav::NavMeshSettings cacheCheckSettings;
  cacheCheckSettings.cellSize = 0.0123;
  cacheCheckSettings.cellHeight = 0.234;
  cacheCheckSettings.agentHeight = 1.2345;
  cacheCheckSettings.agentRadius = 0.123;
  cacheCheckSettings.agentMaxClimb = 0.234;
  cacheCheckSettings.agentMaxSlope = 34.0;
  cacheCheckSettings.regionMinSize = 23.0;
  cacheCheckSettings.regionMergeSize = 25.0;
  cacheCheckSettings.edgeMaxLen = 23.0;
  cacheCheckSettings.edgeMaxError = 1.345;
  cacheCheckSettings.vertsPerPoly = 9.0;
  cacheCheckSettings.detailSampleDist = 9.0;
  cacheCheckSettings.detailSampleMaxError = 2.0;
  cacheCheckSettings.filterLowHangingObstacles = false;
  cacheCheckSettings.filterLedgeSpans = false;
  cacheCheckSettings.filterWalkableLowHeightSpans = false;

  CORRADE_VERIFY(cacheCheckSettings == navmeshSettings);

  // check saving settings
  esp::nav::NavMeshSettings defaultSettings;
  const auto testFilepath =
      Cr::Utility::Path::join(TEST_ASSETS, "test_navmeshsettings_reload.json");
  // save defaults to a file
  defaultSettings.writeToJSON(testFilepath);

  // reload and check against original
  navmeshSettings.readFromJSON(testFilepath);

  CORRADE_VERIFY(defaultSettings == navmeshSettings);

  // remove file created for this test
  bool success = Corrade::Utility::Path::remove(testFilepath);
  if (!success) {
    ESP_WARNING() << "Unable to remove temporary test JSON file"
                  << testFilepath;
  }
}

}  // namespace

CORRADE_TEST_MAIN(PathFinderTest)
