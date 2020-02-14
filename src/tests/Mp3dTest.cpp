// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Utility/Directory.h>
#include <gtest/gtest.h>

#include "esp/scene/SemanticScene.h"

#include "configure.h"

namespace Cr = Corrade;

using namespace esp;
using namespace esp::geo;
using namespace esp::scene;

TEST(Mp3dTest, Load) {
  const std::string filename = Cr::Utility::Directory::join(
      SCENE_DATASETS, "mp3d/1LXtFkjw3qL/1LXtFkjw3qL.house");
  if (!Cr::Utility::Directory::exists(filename))
    GTEST_SKIP_("MP3D dataset not found.");

  SemanticScene house;
  const quatf alignGravity =
      quatf::FromTwoVectors(-vec3f::UnitZ(), ESP_GRAVITY);
  const quatf alignFront = quatf::FromTwoVectors(-vec3f::UnitX(), ESP_FRONT);
  SemanticScene::loadMp3dHouse(filename, house, alignFront * alignGravity);
  LOG(INFO) << "House{nobjects:" << house.count("objects")
            << ",nlevels:" << house.count("levels")
            << ",nregions:" << house.count("regions")
            << ",ncategories:" << house.count("categories")
            << ",bbox:" << house.aabb() << "}";
  for (auto& category : house.categories()) {
    LOG(INFO) << "SemanticCategory{i:" << category->index("raw")
              << ",name:" << category->name("raw")
              << ",mpcat40Name:" << category->name("") << "}";
  }
  for (auto& level : house.levels()) {
    LOG(INFO) << "Level{id:" << level->id() << ",aabb:" << level->aabb() << "}";
    for (auto& region : level->regions()) {
      LOG(INFO) << "Region{id:" << region->id() << ",aabb:" << region->aabb()
                << ",category:" << region->category()->name()
                << ",index:" << region->category()->index() << "}";
      for (auto& object : region->objects()) {
        LOG(INFO) << "Object{id:" << object->id() << ",obb:" << object->obb()
                  << ",category:" << object->category()->name() << "}";
      }
    }
  }
}
