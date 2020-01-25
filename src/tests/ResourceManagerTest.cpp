// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Containers/Optional.h>
#include <Corrade/Utility/Directory.h>
#include <Magnum/EigenIntegration/Integration.h>
#include <Magnum/Math/Range.h>
#include <gtest/gtest.h>
#include <string>

#include "esp/assets/ResourceManager.h"
#include "esp/gfx/Renderer.h"
#include "esp/gfx/WindowlessContext.h"
#include "esp/scene/SceneManager.h"

#include "configure.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

using esp::assets::ResourceManager;
using esp::scene::SceneManager;

TEST(ResourceManagerTest, createJoinedCollisionMesh) {
  esp::gfx::WindowlessContext::uptr context_ =
      esp::gfx::WindowlessContext::create_unique(0);

  std::shared_ptr<esp::gfx::Renderer> renderer_ = esp::gfx::Renderer::create();

  // must declare these in this order due to avoid deallocation errors
  ResourceManager resourceManager;
  SceneManager sceneManager_;

  std::string boxFile =
      Cr::Utility::Directory::join(TEST_ASSETS, "objects/transform_box.glb");

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

TEST(ResourceManagerTest, computeAbsoluteAABB) {
  // must create a GL context which will be used in the resource manager
  esp::gfx::WindowlessContext::uptr context_ =
      esp::gfx::WindowlessContext::create_unique(0);

  // must declare these in this order due to avoid deallocation errors
  ResourceManager resourceManager;
  SceneManager sceneManager;

  std::string sceneFile =
      Cr::Utility::Directory::join(TEST_ASSETS, "objects/5boxes.glb");

  int sceneID = sceneManager.initSceneGraph();
  auto& sceneGraph = sceneManager.getSceneGraph(sceneID);
  esp::scene::SceneNode& sceneRootNode = sceneGraph.getRootNode();
  auto& drawables = sceneGraph.getDrawables();
  const esp::assets::AssetInfo info =
      esp::assets::AssetInfo::fromPath(sceneFile);
  bool loadSuccess =
      resourceManager.loadScene(info, &sceneRootNode, &drawables);
  CHECK_EQ(loadSuccess, true);

  std::vector<Mn::Range3D> aabbs;
  for (size_t iDrawable = 0; iDrawable < drawables.size(); ++iDrawable) {
    Cr::Containers::Optional<Mn::Range3D> aabb =
        dynamic_cast<esp::scene::SceneNode&>(drawables[iDrawable].object())
            .getAbsoluteAABB();
    if (aabb) {
      aabbs.emplace_back(*aabb);
    }
  }

  /* ground truth
   *
   * Objects: (TODO: add more objects to the test, e.g., sphere, cylinder)
   *  a) a cube, with edge length 2.0
   *
   */
  std::vector<Mn::Range3D> aabbsGroundTruth;
  // Box 0: root (parent: null), object "a", centered at origin
  aabbsGroundTruth.emplace_back(Mn::Vector3{-1.0, -1.0, -1.0},
                                Mn::Vector3{1.0, 1.0, 1.0});
  // Box 1: (parent, Box 0), object "a", relative translation (0.0, -4.0, 0.0)
  aabbsGroundTruth.emplace_back(Mn::Vector3{-1.0, -5.0, -1.0},
                                Mn::Vector3{1.0, -3.0, 1.0});
  // Box 2: (parent, Box 1), object "a", relative translation (0.0, 0.0, 4.0)
  aabbsGroundTruth.emplace_back(Mn::Vector3{-1.0, -5.0, 3.0},
                                Mn::Vector3{1.0, -3.0, 5.0});
  // Box 3: (parent, Box 0), object "a", relative translation (-4.0, 0.0, 4.0),
  // relative rotation pi/4 (ccw) around local z-axis of Box 3
  aabbsGroundTruth.emplace_back(Mn::Vector3{-4.0 - sqrt(2.0), -sqrt(2.0), 3.0},
                                Mn::Vector3{-4.0 + sqrt(2.0), sqrt(2.0), 5.0});
  // Box 4: (parent, Box 3), object "a", relative translation (8.0, 0.0, 0.0),
  // relative rotation pi/4 (ccw) around local z-axis of Box 4
  aabbsGroundTruth.emplace_back(Mn::Vector3{3.0, -1.0, 3.0},
                                Mn::Vector3{5.0, 1.0, 5.0});

  // compare against the ground truth
  CHECK_EQ(aabbs.size(), aabbsGroundTruth.size());
  const float epsilon = 1e-6;
  for (size_t iBox = 0; iBox < aabbsGroundTruth.size(); ++iBox) {
    CHECK_LE(std::abs((aabbs[iBox].min() - aabbsGroundTruth[iBox].min()).dot()),
             epsilon);
    CHECK_LE(std::abs((aabbs[iBox].max() - aabbsGroundTruth[iBox].max()).dot()),
             epsilon);
  }
}
