// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <gtest/gtest.h>
#include "esp/assets/SceneLoader.h"
#include "esp/core/esp.h"

using namespace esp::assets;

TEST(AssetsTest, SceneLoaderTest) {
  SceneLoader sceneLoader;
  LOG(INFO) << "Loading mesh at test.obj";
  MeshData mesh = sceneLoader.load({AssetType::UNKNOWN, "test.obj"});
  LOG(INFO) << "Loaded mesh [numVerts: " << mesh.vbo.size() << " ]";
}
