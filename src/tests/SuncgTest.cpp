// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/TestSuite/Tester.h>
#include <Corrade/Utility/Directory.h>

#include "esp/scene/SemanticScene.h"

#include "configure.h"

namespace Cr = Corrade;

namespace Test {

struct SuncgTest : Cr::TestSuite::Tester {
  explicit SuncgTest();

  void testLoad();
  esp::logging::LoggingContext loggingContext;

};  // struct SuncgTest

SuncgTest::SuncgTest() {
  addTests({&SuncgTest::testLoad});
}

void SuncgTest::testLoad() {
  const std::string filename = Cr::Utility::Directory::join(
      SCENE_DATASETS, "suncg/0a0b9b45a1db29832dd84e80c1347854.json");
  if (!Cr::Utility::Directory::exists(filename)) {
    CORRADE_SKIP("SUNCG dataset not found.");
  }

  esp::scene::SemanticScene house;
  esp::scene::SemanticScene::loadSuncgHouse(filename, house);
  // ESP_DEBUG() << "House, bbox:" << house.aabb();

  CORRADE_VERIFY(house.levels().size() > 0);
  for (auto& level : house.levels()) {
    ESP_DEBUG() << "Level{id:"
                << level->id()
                // << ",aabb:" << level->aabb()
                << "}";
    for (auto& region : level->regions()) {
      ESP_DEBUG() << "Region{id:"
                  << region->id()
                  //<< ",aabb:" << region->aabb()
                  << ",category:" << region->category()->name() << "}";
      for (auto& object : region->objects()) {
        ESP_DEBUG() << "Object{id:"
                    << object->id()
                    //<< ",obb:" << object->obb()
                    << ",category:" << object->category()->name() << "}";
      }
    }
  }
}
}  // namespace Test

CORRADE_TEST_MAIN(Test::SuncgTest)
