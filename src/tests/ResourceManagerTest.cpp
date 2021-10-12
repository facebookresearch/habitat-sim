// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Containers/ArrayView.h>
#include <Corrade/Containers/ArrayViewStl.h>
#include <Corrade/Containers/Optional.h>
#include <Corrade/TestSuite/Compare/Container.h>
#include <Corrade/TestSuite/Tester.h>
#include <Corrade/Utility/Directory.h>
#include <Magnum/EigenIntegration/Integration.h>
#include <Magnum/Math/Range.h>
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

namespace {

struct ResourceManagerTest : Cr::TestSuite::Tester {
  explicit ResourceManagerTest();
  void createJoinedCollisionMesh();

#ifdef ESP_BUILD_WITH_VHACD
  void VHACDUsageTest();
#endif

  void loadAndCreateRenderAssetInstance();

  void testShaderTypeSpecification();

  esp::logging::LoggingContext loggingContext;
};  // struct ResourceManagerTest
ResourceManagerTest::ResourceManagerTest() {
  addTests({&ResourceManagerTest::createJoinedCollisionMesh,
#ifdef ESP_BUILD_WITH_VHACD
            &ResourceManagerTest::VHACDUsageTest,
#endif
            &ResourceManagerTest::loadAndCreateRenderAssetInstance,
            &ResourceManagerTest::testShaderTypeSpecification});
}

void ResourceManagerTest::createJoinedCollisionMesh() {
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
  // transformed into a cube via a transform heirarchy. Combined, the
  // resulting mesh should have 24 vertices and 36 indices with corners at the
  // unit corner coordinates as defined in the ground truth vectors below.
  int numVerts = joinedBox->vbo.size();
  int numIndices = joinedBox->ibo.size();

  CORRADE_COMPARE(numVerts, 24);
  CORRADE_COMPARE(numIndices, 36);

  CORRADE_COMPARE_AS(
      Cr::Containers::arrayCast<const Mn::Vector3>(
          Cr::Containers::arrayView(joinedBox->vbo)),
      Cr::Containers::arrayView<Magnum::Vector3>(
          {{-1, 1, 1},  {-1, 1, -1}, {-1, -1, -1}, {-1, -1, 1},  {1, 1, 1},
           {1, -1, -1}, {1, 1, -1},  {1, -1, 1},   {1, 1, -1},   {-1, -1, -1},
           {-1, 1, -1}, {1, -1, -1}, {1, 1, 1},    {-1, 1, -1},  {-1, 1, 1},
           {1, 1, -1},  {1, -1, 1},  {-1, -1, 1},  {-1, -1, -1}, {1, -1, -1},
           {1, 1, 1},   {-1, 1, 1},  {-1, -1, 1},  {1, -1, 1}}),
      Cr::TestSuite::Compare::Container);

  CORRADE_COMPARE_AS(Cr::Containers::arrayView(joinedBox->ibo),
                     Cr::Containers::arrayView<uint32_t>(
                         {0,  1,  2,  0,  2,  3,  4,  5,  6,  4,  7,  5,
                          8,  9,  10, 8,  11, 9,  12, 13, 14, 12, 15, 13,
                          16, 17, 18, 16, 18, 19, 20, 21, 22, 20, 22, 23}),
                     Cr::TestSuite::Compare::Container);

}  // namespace Test

#ifdef ESP_BUILD_WITH_VHACD
void ResourceManagerTest::VHACDUsageTest() {
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
  CORRADE_VERIFY(!resourceManager.isAssetDataRegistered(CHdonutFile));
  resourceManager.createConvexHullDecomposition(donutFile, CHdonutFile, params,
                                                true);

  CORRADE_VERIFY(resourceManager.isAssetDataRegistered(CHdonutFile));
}
#endif

// Load and create a render asset instance and assert success
void ResourceManagerTest::loadAndCreateRenderAssetInstance() {
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
  CORRADE_VERIFY(node);
}

void ResourceManagerTest::testShaderTypeSpecification() {
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
  CORRADE_VERIFY(node);

}  // ResourceManagerTest::testShaderTypeSpecification

}  // namespace

CORRADE_TEST_MAIN(ResourceManagerTest)
