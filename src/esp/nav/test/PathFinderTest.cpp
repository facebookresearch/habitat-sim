#include <Corrade/TestSuite/Compare/Numeric.h>
#include <Corrade/TestSuite/Tester.h>

#include <esp/nav/PathFinder.h>

#include <Corrade/Utility/Directory.h>
#include <Magnum/EigenIntegration/Integration.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/Vector3.h>

#include "configure.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace {

const std::string skokloster = Cr::Utility::Directory::join(
    SCENE_DATASETS,
    "habitat-test-scenes/skokloster-castle.navmesh");

struct PathFinderTest : Cr::TestSuite::Tester {
  explicit PathFinderTest();

  void bounds();
  void tryStepNoSliding();
};

PathFinderTest::PathFinderTest() {
  addTests({&PathFinderTest::bounds, &PathFinderTest::tryStepNoSliding});
}

void PathFinderTest::bounds() {
  esp::nav::PathFinder pathFinder;
  Mn::Vector3 minExpected{-9.75916f, -0.390081f, 0.973853f};
  Mn::Vector3 maxExpected{8.56903f, 6.43441f, 25.5983f};

  pathFinder.loadNavMesh(skokloster);
  CORRADE_VERIFY(pathFinder.isLoaded());
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

}  // namespace

CORRADE_TEST_MAIN(PathFinderTest)
