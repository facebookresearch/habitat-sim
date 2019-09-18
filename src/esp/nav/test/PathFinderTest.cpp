#include <Corrade/TestSuite/Tester.h>

#include <esp/nav/PathFinder.h>

#include <Corrade/Utility/Directory.h>
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
};

PathFinderTest::PathFinderTest() {
  addTests({&PathFinderTest::bounds});
}

void PathFinderTest::bounds() {
  esp::nav::PathFinder pathFinder;
  Mn::Vector3 minExpected{-9.75916f, -0.390081f, 0.973853f};
  Mn::Vector3 maxExpected{8.56903f, 6.43441f, 25.5983f};

  pathFinder.loadNavMesh(skokloster);
  CORRADE_VERIFY(pathFinder.isLoaded());
  std::pair<esp::vec3f, esp::vec3f> bounds = pathFinder.bounds();

  CORRADE_COMPARE(Mn::Vector3::from(bounds.first.data()), minExpected);
  CORRADE_COMPARE(Mn::Vector3::from(bounds.second.data()), maxExpected);
}

}  // namespace

CORRADE_TEST_MAIN(PathFinderTest)
