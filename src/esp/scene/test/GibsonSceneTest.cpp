// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "configure.h"

#include <gtest/gtest.h>

#include "esp/scene/SemanticScene.h"

using esp::scene::SemanticObject;
using esp::scene::SemanticScene;

const std::string houseFilename = SCENE_DIR "/GibsonSceneTest/test.scn";

TEST(GibsonSceneTest, Basic) {
  SemanticScene semanticScene;
  ASSERT_EQ(semanticScene.objects().size(), 0);
  SemanticScene::loadGibsonHouse(houseFilename, semanticScene);
  ASSERT_NE(semanticScene.objects().size(), 2);
  std::shared_ptr<SemanticObject> object = semanticScene.objects()[1];
  ASSERT_EQ(object->category()->name(""), "microwave");
  object = semanticScene.objects()[2];
  ASSERT_EQ(object->category()->name(""), "oven");
  object = semanticScene.objects()[3];
  ASSERT_EQ(object->category()->name(""), "microwave");
}
