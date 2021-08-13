// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Containers/Optional.h>
#include <Corrade/Utility/Directory.h>
#include <Magnum/EigenIntegration/Integration.h>
#include <Magnum/Math/Range.h>
#include <gtest/gtest.h>
#include <string>

#include "esp/assets/RenderAssetInstanceCreationInfo.h"
#include "esp/assets/ResourceManager.h"
#include "esp/gfx/Renderer.h"
#include "esp/gfx/WindowlessContext.h"
#include "esp/scene/SceneManager.h"

#include "configure.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

using esp::assets::ResourceManager;
using esp::metadata::MetadataMediator;
using esp::scene::SceneManager;

TEST(ResourceManagerTest, createJoinedCollisionMesh) {
  esp::logging::LoggingContext loggingContext;
  esp::gfx::WindowlessContext::uptr context_ =
      esp::gfx::WindowlessContext::create_unique(0);

  std::shared_ptr<esp::gfx::Renderer> renderer_ = esp::gfx::Renderer::create();

  // must declare these in this order due to avoid deallocation errors
  auto cfg = esp::sim::SimulatorConfiguration{};
  // setting values for stage load
  cfg.loadSemanticMesh = false;
  cfg.forceSeparateSemanticSceneGraph = false;
  auto MM = MetadataMediator::create(cfg);
  ResourceManager resourceManager(MM);
  SceneManager sceneManager_;
  auto stageAttributesMgr = MM->getStageAttributesManager();
  std::string boxFile =
      Cr::Utility::Directory::join(TEST_ASSETS, "objects/transform_box.glb");

  // create stage attributes file
  auto stageAttributes = stageAttributesMgr->createObject(boxFile, true);

  int sceneID = sceneManager_.initSceneGraph();
  auto& sceneGraph = sceneManager_.getSceneGraph(sceneID);
  const esp::assets::AssetInfo info = esp::assets::AssetInfo::fromPath(boxFile);

  std::vector<int> tempIDs{sceneID, esp::ID_UNDEFINED};
  bool result = resourceManager.loadStage(stageAttributes, nullptr, nullptr,
                                          &sceneManager_, tempIDs);

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
      {-1, 1, 1},  {-1, 1, -1}, {-1, -1, -1}, {-1, -1, 1},  {1, 1, 1},
      {1, -1, -1}, {1, 1, -1},  {1, -1, 1},   {1, 1, -1},   {-1, -1, -1},
      {-1, 1, -1}, {1, -1, -1}, {1, 1, 1},    {-1, 1, -1},  {-1, 1, 1},
      {1, 1, -1},  {1, -1, 1},  {-1, -1, 1},  {-1, -1, -1}, {1, -1, -1},
      {1, 1, 1},   {-1, 1, 1},  {-1, -1, 1},  {1, -1, 1}};

  std::vector<uint32_t> indexGroundTruth{
      0,  1,  2,  0,  2,  3,  4,  5,  6,  4,  7,  5,  8,  9,  10, 8,  11, 9,
      12, 13, 14, 12, 15, 13, 16, 17, 18, 16, 18, 19, 20, 21, 22, 20, 22, 23};

  for (size_t vix = 0; vix < joinedBox->vbo.size(); vix++) {
    // ESP_DEBUG() << joinedBox->vbo[vix]   << "vs" <<
    // vertGroundTruth[vix];
    ASSERT_EQ(vertGroundTruth[vix], Magnum::Vector3(joinedBox->vbo[vix]));
  }

  for (size_t iix = 0; iix < joinedBox->ibo.size(); iix++) {
    // ESP_DEBUG() << joinedBox->ibo[iix]   << "vs" <<
    // indexGroundTruth[iix];
    ASSERT_EQ(indexGroundTruth[iix], joinedBox->ibo[iix]);
  }
}

#ifdef ESP_BUILD_WITH_VHACD
TEST(ResourceManagerTest, VHACDUsageTest) {
  esp::logging::LoggingContext loggingContext;
  esp::gfx::WindowlessContext::uptr context_ =
      esp::gfx::WindowlessContext::create_unique(0);

  std::shared_ptr<esp::gfx::Renderer> renderer_ = esp::gfx::Renderer::create();

  // must declare these in this order due to avoid deallocation errors
  // must declare these in this order due to avoid deallocation errors
  auto cfg = esp::sim::SimulatorConfiguration{};
  // setting values for stage load
  cfg.loadSemanticMesh = false;
  cfg.forceSeparateSemanticSceneGraph = false;
  auto MM = MetadataMediator::create(cfg);
  ResourceManager resourceManager(MM);
  SceneManager sceneManager_;
  auto stageAttributesMgr = MM->getStageAttributesManager();
  std::string donutFile =
      Cr::Utility::Directory::join(TEST_ASSETS, "objects/donut.glb");
  std::string CHdonutFile =
      Cr::Utility::Directory::join(TEST_ASSETS, "objects/CHdonut.glb");

  // create stage attributes file
  auto stageAttributes = stageAttributesMgr->createObject(donutFile, true);

  int sceneID = sceneManager_.initSceneGraph();
  auto& sceneGraph = sceneManager_.getSceneGraph(sceneID);
  const esp::assets::AssetInfo info =
      esp::assets::AssetInfo::fromPath(donutFile);

  std::vector<int> tempIDs{sceneID, esp::ID_UNDEFINED};
  bool result = resourceManager.loadStage(stageAttributes, nullptr, nullptr,
                                          &sceneManager_, tempIDs);

  esp::assets::MeshData::uptr joinedBox =
      resourceManager.createJoinedCollisionMesh(donutFile);

  esp::assets::ResourceManager::VHACDParameters params;
  // params.setMaxNumVerticesPerCH(10);
  params.m_resolution = 1000000;
  CORRADE_INTERNAL_ASSERT(!resourceManager.isAssetDataRegistered(CHdonutFile));
  resourceManager.createConvexHullDecomposition(donutFile, CHdonutFile, params,
                                                true);

  CORRADE_INTERNAL_ASSERT(resourceManager.isAssetDataRegistered(CHdonutFile));
}
#endif

// Load and create a render asset instance and assert success
TEST(ResourceManagerTest, loadAndCreateRenderAssetInstance) {
  esp::logging::LoggingContext loggingContext;
  esp::gfx::WindowlessContext::uptr context_ =
      esp::gfx::WindowlessContext::create_unique(0);

  std::shared_ptr<esp::gfx::Renderer> renderer_ = esp::gfx::Renderer::create();

  // must declare these in this order due to avoid deallocation errors
  auto MM = MetadataMediator::create();
  ResourceManager resourceManager(MM);
  SceneManager sceneManager_;
  std::string boxFile =
      Cr::Utility::Directory::join(TEST_ASSETS, "objects/transform_box.glb");

  int sceneID = sceneManager_.initSceneGraph();
  auto& sceneGraph = sceneManager_.getSceneGraph(sceneID);
  const esp::assets::AssetInfo info = esp::assets::AssetInfo::fromPath(boxFile);

  const std::string lightSetupKey = "";
  esp::assets::RenderAssetInstanceCreationInfo::Flags flags;
  flags |= esp::assets::RenderAssetInstanceCreationInfo::Flag::IsRGBD;
  flags |= esp::assets::RenderAssetInstanceCreationInfo::Flag::IsSemantic;
  esp::assets::RenderAssetInstanceCreationInfo creation(
      boxFile, Corrade::Containers::NullOpt, flags, lightSetupKey);

  std::vector<int> tempIDs{sceneID, esp::ID_UNDEFINED};
  auto* node = resourceManager.loadAndCreateRenderAssetInstance(
      info, creation, &sceneManager_, tempIDs);
  CORRADE_INTERNAL_ASSERT(node);
}
