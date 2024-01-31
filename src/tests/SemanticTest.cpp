// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/TestSuite/Tester.h>
#include <vector>

#include "esp/core/Esp.h"
#include "esp/metadata/MetadataMediator.h"
#include "esp/scene/SemanticScene.h"

#include "configure.h"

namespace Cr = Corrade;
namespace AttrMgrs = esp::metadata::managers;
namespace Attrs = esp::metadata::attributes;
using Attrs::SemanticAttributes;
using esp::metadata::MetadataMediator;

namespace {

struct SemanticTest : Cr::TestSuite::Tester {
  explicit SemanticTest();

  /**
   * @brief This test will validate semantic region creation and containment
   * checks.
   */
  void TestRegionCreation();

  esp::logging::LoggingContext loggingContext_;

  //
  std::shared_ptr<Attrs::SemanticAttributes> semanticAttr_ = nullptr;

};  // struct SemanticTest

SemanticTest::SemanticTest() {
  // set up a default simulation config to initialize MM
  auto cfg = esp::sim::SimulatorConfiguration{};
  esp::metadata::MetadataMediator::uptr MM =
      MetadataMediator::create_unique(cfg);

  const std::string semanticConfigFile = Cr::Utility::Path::join(
      TEST_ASSETS, "semantic/test_regions.semantic_config.json");

  // Build an attributes based on the json file
  semanticAttr_ = MM->getSemanticAttributesManager()->createObject(
      semanticConfigFile, true);

  addTests({&SemanticTest::TestRegionCreation});
}

void SemanticTest::TestRegionCreation() {
  std::shared_ptr<esp::scene::SemanticScene> semanticScene =
      esp::scene::SemanticScene::create();

  esp::scene::SemanticScene::loadSemanticSceneDescriptor(semanticAttr_,
                                                         *semanticScene);
  // Verify semantic scene values
  const auto& regions = semanticScene->regions();
  // Verify there are 2 regions
  CORRADE_COMPARE(regions.size(), 2);
  // The regions are symmetric, so both lists will work for both, if the signs
  // are flipped.
  // Build a list of points within the region
  std::vector<Mn::Vector3> hitTestPoints(6);
  hitTestPoints[0] = Mn::Vector3{-5.1, 0.0, 0.01};
  hitTestPoints[1] = Mn::Vector3{-5.1, 1.0, 0.01};
  hitTestPoints[2] = Mn::Vector3{-5.1, -1.0, 0.01};
  hitTestPoints[3] = Mn::Vector3{-24.9, 0.0, 0.01};
  hitTestPoints[4] = Mn::Vector3{-15.1, 0.0, 9.9};
  hitTestPoints[5] = Mn::Vector3{-15.1, 0.0, -9.9};
  // Build a list of points outside the region
  std::vector<Mn::Vector3> missTestPoints(6);
  missTestPoints[0] = Mn::Vector3{0.0, 0.0, 0.01};
  missTestPoints[1] = Mn::Vector3{-6.0, 0.0, 10.01};
  missTestPoints[2] = Mn::Vector3{-5.1, -2.1, 0.01};
  missTestPoints[3] = Mn::Vector3{-5.1, 2.1, 0.01};
  missTestPoints[4] = Mn::Vector3{-15.1, -1.5, 10.01};
  missTestPoints[5] = Mn::Vector3{-15.1, 1.5, -10.01};

  // Check region construction
  {  // negative X region
    const auto region = regions[0];
    CORRADE_COMPARE(region->id(), "test_region_negativeX");
    CORRADE_COMPARE(region->getPolyLoopPoints().size(), 6);
    CORRADE_COMPARE(region->getExtrusionHeight(), 4.0);
    CORRADE_COMPARE(region->getFloorHeight(), -2.0);

    const auto regionCat = region->category();
    CORRADE_COMPARE(regionCat->name(), "bedroom");
    CORRADE_COMPARE(regionCat->index(), 0);

    int idx = 0;
    for (const auto& pt : hitTestPoints) {
      CORRADE_ITERATION(idx++);
      CORRADE_VERIFY(region->contains(pt));
    }
    idx = 0;
    for (const auto& pt : missTestPoints) {
      CORRADE_ITERATION(idx++);
      CORRADE_VERIFY(!region->contains(pt));
    }
  }

  {  // positive X region
    const auto region = regions[1];
    CORRADE_COMPARE(region->id(), "test_region_positiveX");
    CORRADE_COMPARE(region->getPolyLoopPoints().size(), 6);
    CORRADE_COMPARE(region->getExtrusionHeight(), 4.0);
    CORRADE_COMPARE(region->getFloorHeight(), -2.0);
    const auto regionCat = region->category();
    CORRADE_COMPARE(regionCat->name(), "bathroom");
    CORRADE_COMPARE(regionCat->index(), 1);
    int idx = 0;
    for (const auto& pt : hitTestPoints) {
      CORRADE_ITERATION(idx++);
      CORRADE_VERIFY(region->contains(-pt));
    }
    idx = 0;
    for (const auto& pt : missTestPoints) {
      CORRADE_ITERATION(idx++);
      CORRADE_VERIFY(!region->contains(-pt));
    }
  }

}  // namespace

}  // namespace

CORRADE_TEST_MAIN(SemanticTest)
