// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "configure.h"

#include <gtest/gtest.h>

#include <Corrade/Utility/Directory.h>
#include <string>
#include "esp/io/io.h"
#include "esp/scene/SemanticScene.h"
#include "esp/sim/Simulator.h"

namespace Cr = Corrade;

using esp::scene::SemanticScene;
using esp::sim::Simulator;
using esp::sim::SimulatorConfiguration;

const std::string houseFilename = SCENE_DIR "/GibsonSceneTest/test.scn";

TEST(GibsonSceneTest, Basic) {
  esp::logging::LoggingContext loggingContext;
  SemanticScene semanticScene;
  ASSERT_EQ(semanticScene.objects().size(), 0);
  SemanticScene::loadGibsonHouse(houseFilename, semanticScene);
  ASSERT_NE(semanticScene.objects().size(), 2);
  auto object = semanticScene.objects()[1];
  ASSERT_EQ(object->category()->name(""), "microwave");
  CORRADE_INTERNAL_ASSERT(
      object->obb().center().isApprox(esp::vec3f(2.83999, 4.76085, 1.49223)));
  CORRADE_INTERNAL_ASSERT(
      object->obb().sizes().isApprox(esp::vec3f(0.406775, 1.28023, 0.454744)));
  object = semanticScene.objects()[2];
  ASSERT_EQ(object->category()->name(""), "oven");
  object = semanticScene.objects()[3];
  ASSERT_EQ(object->category()->name(""), "microwave");
}

const std::string gibsonSemanticFilename =
    Cr::Utility::Directory::join(SCENE_DATASETS, "gibson/Allensville.scn");

TEST(GibsonSemanticSimTest, Basic) {
  esp::logging::LoggingContext loggingContext;
  if (!esp::io::exists(gibsonSemanticFilename)) {
    std::string skip_message = "Gibson's semantic scene file \"" +
                               gibsonSemanticFilename + "\" wasn't found.";
    GTEST_SKIP_(skip_message.c_str());
  }
  SimulatorConfiguration cfg;
  cfg.activeSceneName =
      esp::io::changeExtension(gibsonSemanticFilename, ".glb");
  Simulator simulator(cfg);
  const auto& semanticScene = simulator.getSemanticScene();
  ASSERT_EQ(semanticScene->objects().size(), 34);
  const auto& microwave = semanticScene->objects()[1];
  ASSERT_EQ(microwave->category()->name(""), "microwave");
  CORRADE_INTERNAL_ASSERT(microwave->obb().center().isApprox(
      esp::vec3f(2.83999, 4.76085, 1.49223)));
  CORRADE_INTERNAL_ASSERT(microwave->obb().sizes().isApprox(
      esp::vec3f(0.406775, 1.28023, 0.454744)));
}
