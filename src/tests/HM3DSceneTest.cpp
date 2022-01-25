// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#include <Corrade/TestSuite/Tester.h>
#include <Corrade/Utility/Directory.h>
#include <string>

#include "esp/scene/SemanticScene.h"

#include "configure.h"

namespace Cr = Corrade;

namespace {

struct HM3DSceneTest : Cr::TestSuite::Tester {
  explicit HM3DSceneTest();

  void testHM3DScene();

  void testHM3DSemanticScene();

  esp::logging::LoggingContext loggingContext;

};  // struct HM3DSceneTest
HM3DSceneTest::HM3DSceneTest() {
  addTests(
      {&HM3DSceneTest::testHM3DScene, &HM3DSceneTest::testHM3DSemanticScene});
}

void HM3DSceneTest::testHM3DScene() {
  CORRADE_COMPARE(0, 0);
}

void HM3DSceneTest::testHM3DSemanticScene() {
  CORRADE_COMPARE(0, 0);
}

}  // namespace

CORRADE_TEST_MAIN(HM3DSceneTest)
