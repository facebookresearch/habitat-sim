// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/TestSuite/Tester.h>
#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/Path.h>

#include "esp/scene/SemanticScene.h"

#include "configure.h"

namespace Cr = Corrade;

namespace {

struct Mp3dTest : Cr::TestSuite::Tester {
  explicit Mp3dTest();

  void testLoad();

  esp::logging::LoggingContext loggingContext;
};  // struct Mp3Test

Mp3dTest::Mp3dTest() {
  addTests({&Mp3dTest::testLoad});
}  // Mp3dTest ctor

void Mp3dTest::testLoad() {
  const std::string filename = Cr::Utility::Path::join(
      SCENE_DATASETS, "mp3d/17DRP5sb8fy/17DRP5sb8fy.house");
  if (!Cr::Utility::Path::exists(filename)) {
    CORRADE_SKIP("MP3D dataset not found.");
  }

  esp::scene::SemanticScene house;
  const esp::quatf alignGravity =
      esp::quatf::FromTwoVectors(-esp::vec3f::UnitZ(), esp::geo::ESP_GRAVITY);
  const esp::quatf alignFront =
      esp::quatf::FromTwoVectors(-esp::vec3f::UnitX(), esp::geo::ESP_FRONT);
  esp::scene::SemanticScene::loadMp3dHouse(filename, house,
                                           alignFront * alignGravity);
  ESP_DEBUG() << "House{nobjects:" << house.count("objects")
              << ",nlevels:" << house.count("levels")
              << ",nregions:" << house.count("regions")
              << ",ncategories:" << house.count("categories")
              << ",bbox:" << Mn::Range3D{house.aabb()} << "}";
  ESP_DEBUG() << "~~~~~~~~~~~ Categories : " << house.categories().size();
  CORRADE_COMPARE(house.categories().size(), 1659);
  for (auto& category : house.categories()) {
    ESP_DEBUG() << "SemanticCategory{i:" << category->index("raw")
                << ",name:" << category->name("raw")
                << ",mpcat40Name:" << category->name("") << "}";
  }
  ESP_DEBUG() << "~~~~~~~~~~~ Levels : " << house.levels().size();
  CORRADE_COMPARE(house.levels().size(), 1);
  for (auto& level : house.levels()) {
    ESP_DEBUG() << "Level{id:" << level->id()
                << ",aabb:" << Mn::Range3D{level->aabb()} << "}";
    for (auto& region : level->regions()) {
      ESP_DEBUG() << "Region{id:" << region->id()
                  << ",aabb:" << Mn::Range3D{region->aabb()}
                  << ",category:" << region->category()->name()
                  << ",index:" << region->category()->index() << "}";
      for (auto& object : region->objects()) {
        ESP_DEBUG() << "Object{id:" << object->id() << ",obb:" << object->obb()
                    << ",category:" << object->category()->name() << "}";
      }  // per object
    }    // per region
  }      // per level
}

}  // namespace

CORRADE_TEST_MAIN(Mp3dTest)
