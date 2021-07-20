// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <gtest/gtest.h>

#include "esp/scene/SceneGraph.h"

using esp::gfx::DrawableGroup;
using esp::scene::SceneGraph;

class SceneGraphTest : public ::testing::Test {
 protected:
  void SetUp() override { numInitialGroups = g.getDrawableGroups().size(); }

  esp::logging::LoggingContext loggingContext_;
  size_t numInitialGroups;
  SceneGraph g;
  const std::string groupName = "testGroup";
};

TEST_F(SceneGraphTest, GetDrawableGroup) {
  EXPECT_EQ(g.getDrawableGroup("not created"), nullptr);

  DrawableGroup* group = g.createDrawableGroup(groupName);
  ASSERT_NE(group, nullptr);
  EXPECT_EQ(g.getDrawableGroups().size(), numInitialGroups + 1);
  ASSERT_EQ(g.getDrawableGroup(groupName), group);
}

TEST_F(SceneGraphTest, DeleteDrawableGroup) {
  EXPECT_FALSE(g.deleteDrawableGroup("not created"));

  DrawableGroup* group = g.createDrawableGroup(groupName);
  ASSERT_NE(group, nullptr);
  ASSERT_EQ(g.getDrawableGroup(groupName), group);

  ASSERT_TRUE(g.deleteDrawableGroup(groupName));
  EXPECT_EQ(g.getDrawableGroups().size(), numInitialGroups);
  ASSERT_EQ(g.getDrawableGroup(groupName), nullptr);
}
