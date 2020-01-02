// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "configure.h"

#include <gtest/gtest.h>

#include <Corrade/Utility/Directory.h>
#include <string>
#include "esp/io/io.h"
#include "esp/scene/SemanticScene.h"
#include "esp/sim/SimulatorWithAgents.h"

namespace Cr = Corrade;

using esp::gfx::SimulatorConfiguration;
using esp::scene::SceneConfiguration;
using esp::scene::SemanticScene;
using esp::sim::SimulatorWithAgents;

const std::string houseFilename = SCENE_DIR "/GibsonSceneTest/test.scn";

TEST(GibsonSceneTest, Basic) {
  SemanticScene semanticScene;
  ASSERT_EQ(semanticScene.objects().size(), 0);
  SemanticScene::loadGibsonHouse(houseFilename, semanticScene);
  ASSERT_NE(semanticScene.objects().size(), 2);
  auto object = semanticScene.objects()[1];
  ASSERT_EQ(object->category()->name(""), "microwave");
  object = semanticScene.objects()[2];
  ASSERT_EQ(object->category()->name(""), "oven");
  object = semanticScene.objects()[3];
  ASSERT_EQ(object->category()->name(""), "microwave");
}

const std::string gibsonSemanticFilename =
    Cr::Utility::Directory::join(SCENE_DATASETS, "gibson/Allensville.scn");

TEST(GibsonSemanticSimTest, Basic) {
  if (!esp::io::exists(gibsonSemanticFilename)) {
    std::string skip_message = "Gibson's semantic scene file \"" +
                               gibsonSemanticFilename + "\" wasn't found.";
    GTEST_SKIP_(skip_message.c_str());
  }
  SimulatorConfiguration cfg;
  cfg.scene.id = esp::io::changeExtension(gibsonSemanticFilename, ".glb");
  SimulatorWithAgents simulator(cfg);
  const auto& semanticScene = simulator.getSemanticScene();
  ASSERT_EQ(semanticScene->objects().size(), 34);
  const auto& microwave = semanticScene->objects()[1];
  ASSERT_EQ(microwave->category()->name(""), "microwave");
}
