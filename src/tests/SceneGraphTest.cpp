// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/TestSuite/Tester.h>

#include "esp/scene/SceneGraph.h"

using esp::gfx::DrawableGroup;
using esp::scene::SceneGraph;

namespace {
struct SceneGraphTest : Cr::TestSuite::Tester {
  explicit SceneGraphTest();
  void testGetDrawableGroup();
  void testDeleteDrawableGroup();
  esp::logging::LoggingContext loggingContext_;
  size_t numInitialGroups;
  SceneGraph g;
  const std::string groupName = "testGroup";
};
SceneGraphTest::SceneGraphTest() {
  numInitialGroups = g.getDrawableGroups().size();
  addTests({&SceneGraphTest::testGetDrawableGroup,
            &SceneGraphTest::testDeleteDrawableGroup});
}

void SceneGraphTest::testGetDrawableGroup() {
  CORRADE_VERIFY(g.getDrawableGroup("not created") == nullptr);

  DrawableGroup* group = g.createDrawableGroup(groupName);
  CORRADE_VERIFY(group);
  CORRADE_COMPARE(g.getDrawableGroups().size(), numInitialGroups + 1);
  CORRADE_COMPARE(g.getDrawableGroup(groupName), group);
}

void SceneGraphTest::testDeleteDrawableGroup() {
  CORRADE_VERIFY(!g.deleteDrawableGroup("not created"));

  CORRADE_VERIFY(g.deleteDrawableGroup(groupName));
  CORRADE_COMPARE(g.getDrawableGroups().size(), numInitialGroups);
  CORRADE_VERIFY(g.getDrawableGroup(groupName) == nullptr);
}
}  // namespace

CORRADE_TEST_MAIN(SceneGraphTest)
