// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Utility/Directory.h>
#include <Magnum/EigenIntegration/Integration.h>
#include <gtest/gtest.h>
#include <string>

#include "esp/assets/ResourceManager.h"
#include "esp/gfx/Renderer.h"
#include "esp/gfx/WindowlessContext.h"
#include "esp/scene/SceneManager.h"

#include "configure.h"

namespace Cr = Corrade;

using esp::assets::ResourceManager;
using esp::scene::SceneManager;

const std::string dataDir = Cr::Utility::Directory::join(SCENE_DATASETS, "../");

TEST(ResourceManagerTest, createJoinedCollisionMesh) {
  esp::gfx::WindowlessContext::uptr context_ =
      esp::gfx::WindowlessContext::create_unique(0);

  std::shared_ptr<esp::gfx::Renderer> renderer_ = esp::gfx::Renderer::create();

  // must declare these in this order due to avoid deallocation errors
  ResourceManager resourceManager;
  SceneManager sceneManager_;

  std::string boxFile = Cr::Utility::Directory::join(
      dataDir, "test_assets/objects/transform_box.glb");

  int sceneID = sceneManager_.initSceneGraph();
  auto& sceneGraph = sceneManager_.getSceneGraph(sceneID);
  esp::scene::SceneNode* navSceneNode = &sceneGraph.getRootNode().createChild();
  const esp::assets::AssetInfo info = esp::assets::AssetInfo::fromPath(boxFile);
  resourceManager.loadScene(info, navSceneNode, nullptr);

  esp::assets::MeshData::uptr joinedBox =
      resourceManager.createJoinedCollisionMesh(boxFile);

  // transform_box.glb is composed of 6 identical triangulated plane meshes
  // transformed into a cube via a transform heirarchy. Combined, the resulting
  // mesh should have 24 vertices and 36 indices with corners at the unit corner
  // coordinates as defined in the ground truth vectors below.
  int numVerts = joinedBox->vbo.size();
  int numIndices = joinedBox->ibo.size();

  ASSERT_EQ(numVerts, 24);
  ASSERT_EQ(numIndices, 36);

  std::vector<Magnum::Vector3> vertGroundTruth{
      {-1, 1, 1},  {-1, -1, -1}, {-1, 1, -1},  {-1, -1, 1}, {1, 1, 1},
      {1, -1, -1}, {1, 1, -1},   {1, -1, 1},   {1, 1, -1},  {-1, -1, -1},
      {-1, 1, -1}, {1, -1, -1},  {1, 1, 1},    {-1, 1, -1}, {-1, 1, 1},
      {1, 1, -1},  {1, -1, 1},   {-1, -1, -1}, {-1, -1, 1}, {1, -1, -1},
      {1, 1, 1},   {-1, -1, 1},  {-1, 1, 1},   {1, -1, 1}};

  std::vector<uint32_t> indexGroundTruth{
      0,  1,  2,  0,  3,  1,  4,  5,  6,  4,  7,  5,  8,  9,  10, 8,  11, 9,
      12, 13, 14, 12, 15, 13, 16, 17, 18, 16, 19, 17, 20, 21, 22, 20, 23, 21};

  for (size_t vix = 0; vix < joinedBox->vbo.size(); vix++) {
    // Cr::Utility::Debug() << joinedBox->vbo[vix] << " vs " <<
    // vertGroundTruth[vix];
    ASSERT_EQ(vertGroundTruth[vix], Magnum::Vector3(joinedBox->vbo[vix]));
  }

  for (size_t iix = 0; iix < joinedBox->ibo.size(); iix++) {
    // Cr::Utility::Debug() << joinedBox->ibo[iix] << " vs " <<
    // indexGroundTruth[iix];
    ASSERT_EQ(indexGroundTruth[iix], joinedBox->ibo[iix]);
  }
}
