// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#include <Corrade/TestSuite/Tester.h>
#include <Corrade/Utility/Path.h>
#include <string>
#include "esp/io/Io.h"
#include "esp/scene/SemanticScene.h"
#include "esp/sim/Simulator.h"

#include "configure.h"

namespace Cr = Corrade;

using esp::scene::SemanticScene;
using esp::sim::Simulator;
using esp::sim::SimulatorConfiguration;

namespace {

const std::string houseFilename = Cr::Utility::Path::join(
    DATA_DIR,
    "test_assets/dataset_tests/GibsonSceneTest/GibsonTestScene.scn");

const std::string gibsonSemanticFilename =
    Cr::Utility::Path::join(SCENE_DATASETS, "gibson/Allensville.scn");

struct GibsonSceneTest : Cr::TestSuite::Tester {
  explicit GibsonSceneTest();

  void testGibsonScene();

  void testGibsonSemanticScene();

  esp::logging::LoggingContext loggingContext;

};  // struct GibsonSceneTest

GibsonSceneTest::GibsonSceneTest() {
  addTests({&GibsonSceneTest::testGibsonScene,
            &GibsonSceneTest::testGibsonSemanticScene});
}  // ctor

void GibsonSceneTest::testGibsonScene() {
  esp::logging::LoggingContext loggingContext;
  SemanticScene semanticScene;
  CORRADE_COMPARE(semanticScene.objects().size(), 0);
  SemanticScene::loadGibsonHouse(houseFilename, semanticScene);
  CORRADE_VERIFY(semanticScene.objects().size() != 2);
  auto object = semanticScene.objects()[1];
  CORRADE_COMPARE(object->category()->name(""), "microwave");
  CORRADE_VERIFY(
      object->obb().center().isApprox(esp::vec3f(2.83999, 4.76085, 1.49223)));
  CORRADE_VERIFY(
      object->obb().sizes().isApprox(esp::vec3f(0.406775, 1.28023, 0.454744)));
  object = semanticScene.objects()[2];
  CORRADE_COMPARE(object->category()->name(""), "oven");
  object = semanticScene.objects()[3];
  CORRADE_COMPARE(object->category()->name(""), "microwave");
}

void GibsonSceneTest::testGibsonSemanticScene() {
  esp::logging::LoggingContext loggingContext;
  if (!Cr::Utility::Path::exists(gibsonSemanticFilename)) {
    std::string skip_message = "Gibson's semantic scene file \"" +
                               gibsonSemanticFilename + "\" wasn't found.";
    CORRADE_SKIP(skip_message.c_str());
  }
  SimulatorConfiguration cfg;
  cfg.activeSceneName =
      esp::io::changeExtension(gibsonSemanticFilename, ".glb");
  Simulator simulator(cfg);
  const auto& semanticScene = simulator.getSemanticScene();
  CORRADE_COMPARE(semanticScene->objects().size(), 34);
  const auto& microwave = semanticScene->objects()[1];
  CORRADE_COMPARE(microwave->category()->name(""), "microwave");
  CORRADE_VERIFY(microwave->obb().center().isApprox(
      esp::vec3f(2.83999, 4.76085, 1.49223)));
  CORRADE_VERIFY(microwave->obb().sizes().isApprox(
      esp::vec3f(0.406775, 1.28023, 0.454744)));
}

}  // namespace

CORRADE_TEST_MAIN(GibsonSceneTest)
