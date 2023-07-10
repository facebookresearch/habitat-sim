// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Containers/ArrayView.h>
#include <Corrade/Containers/ArrayViewStl.h>
#include <Corrade/Containers/Optional.h>
#include <Corrade/TestSuite/Compare/Container.h>
#include <Corrade/TestSuite/Tester.h>
#include <Corrade/Utility/Path.h>
#include <Magnum/EigenIntegration/Integration.h>
#include <Magnum/Math/Range.h>
#include <Magnum/Trade/MaterialData.h>
#include <string>

#include "esp/assets/MeshData.h"
#include "esp/assets/RenderAssetInstanceCreationInfo.h"
#include "esp/assets/ResourceManager.h"
#include "esp/gfx/Renderer.h"
#include "esp/gfx/WindowlessContext.h"
#include "esp/metadata/MetadataMediator.h"
#include "esp/metadata/attributes/AttributesBase.h"
#include "esp/scene/SceneManager.h"

#include "configure.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

using esp::assets::ResourceManager;
using esp::metadata::MetadataMediator;
using esp::metadata::attributes::ObjectInstanceShaderType;
using esp::scene::SceneManager;

namespace {

struct ResourceManagerTest : Cr::TestSuite::Tester {
  explicit ResourceManagerTest();
  void createJoinedCollisionMesh();

  void loadAndCreateRenderAssetInstance();

  void testShaderTypeSpecification();

  esp::logging::LoggingContext loggingContext;
};  // struct ResourceManagerTest
ResourceManagerTest::ResourceManagerTest() {
  addTests({
      &ResourceManagerTest::createJoinedCollisionMesh,
      &ResourceManagerTest::loadAndCreateRenderAssetInstance,
      &ResourceManagerTest::testShaderTypeSpecification,
  });
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
      Cr::Utility::Path::join(TEST_ASSETS, "objects/transform_box.glb");

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
      Cr::Utility::Path::join(TEST_ASSETS, "objects/transform_box.glb");

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

/**
 * @brief Recurse through Transform tree to find all material IDs
 * @param root MeshTransformNode that holds a material id and vector of children
 * @param matIDs Vector of string material ids to be populated.
 * @param margin Display aid, to visualize tree structure.
 */
void buildMaterialIDs(const esp::assets::MeshTransformNode& root,
                      std::set<std::string>& matIDs,
                      const std::string margin = "") {
  if (!Cr::Utility::String::trim(root.materialID).empty()) {
    matIDs.emplace(root.materialID);
  }
  ESP_DEBUG() << margin << "MMD material ID : " << root.materialID
              << " # children :" << root.children.size();
  for (const auto& child : root.children) {
    buildMaterialIDs(child, matIDs, margin + "\t");
  }
}

/**
 * @brief Test that the material being created is built based on the
 * info-specified shaderType.
 * @param specifiedAssetType integer cast of @ref
 * esp::metadata::attributes::ObjectInstanceShaderType that is expected in the
 * given material
 * @param info The AssetInfo describing the material to load
 * @param MM The @ref esp::metadata::MetadataMediator from which to build
 * ResourceManager.  We need to rebuild every test so that we force reload of
 * assets.
 */
void testAssetTypeMatch(ObjectInstanceShaderType specifiedAssetType,
                        const esp::assets::AssetInfo& info,
                        std::shared_ptr<esp::metadata::MetadataMediator> MM) {
  ResourceManager resourceManager(MM);
  // build a render asset based on the passed info specifications.
  bool success = resourceManager.loadRenderAsset(info);
  CORRADE_VERIFY(success);
  // get the MeshMetaData corresponding to the loaded asset
  const esp::assets::MeshMetaData& meshMetaData =
      resourceManager.getMeshMetaData(info.filepath);
  // vector to hold all material ids in mesh node hierarchy
  std::set<std::string> matIDs;
  // traverse mesh node hierarchy to scrap all material ids
  buildMaterialIDs(meshMetaData.root, matIDs);

  ESP_DEBUG() << "# Materials specified in asset:" << matIDs.size();
  CORRADE_COMPARE(matIDs.size(), 3);

  int specifiedAssetTypeInt = static_cast<int>(specifiedAssetType);
  // get shaderManager from RM and check material @ material ID
  auto& shaderManager = resourceManager.getShaderManager();
  // all materials specified in the hierarchy should match the material for the
  // desired shadertype.
  for (const std::string id : matIDs) {
    int shaderTypeSpec =
        shaderManager.get<Mn::Trade::MaterialData>(id)->attribute<int>(
            "shaderTypeToUse");
    ESP_DEBUG() << "mat ID : " << id << "type spec in mat :" << shaderTypeSpec
                << " | spec in asset :" << specifiedAssetTypeInt;
    CORRADE_COMPARE(shaderTypeSpec, specifiedAssetTypeInt);
  }
}  // testAssetTypeMatch

void ResourceManagerTest::testShaderTypeSpecification() {
  esp::gfx::WindowlessContext::uptr context_ =
      esp::gfx::WindowlessContext::create_unique(0);

  std::shared_ptr<esp::gfx::Renderer> renderer_ = esp::gfx::Renderer::create();

  // must declare these in this order due to avoid deallocation errors
  auto MM = MetadataMediator::create();
  std::string boxFile =
      Cr::Utility::Path::join(TEST_ASSETS, "objects/transform_box.glb");
  // bogus test for corrade
  CORRADE_VERIFY(true);

  esp::assets::AssetInfo info = esp::assets::AssetInfo::fromPath(boxFile);
  ESP_DEBUG() << "Testing flat via forceFlatShading == true";
  // force flat shading
  info.forceFlatShading = true;
  // object's material type is flat
  testAssetTypeMatch(ObjectInstanceShaderType::Flat, info, MM);
  ESP_DEBUG() << "Testing Material type, which is PBR for this asset.";

  // enable lightig and use material type
  info.forceFlatShading = false;
  info.shaderTypeToUse = ObjectInstanceShaderType::Material;
  // object's material type is PBR
  testAssetTypeMatch(ObjectInstanceShaderType::PBR, info, MM);

  ESP_DEBUG() << "Testing PBR explicitly being set.";
  // force pbr
  info.shaderTypeToUse = ObjectInstanceShaderType::PBR;
  testAssetTypeMatch(ObjectInstanceShaderType::PBR, info, MM);

  ESP_DEBUG() << "Testing Phong explicitly being set.";
  // force phong
  info.shaderTypeToUse = ObjectInstanceShaderType::Phong;
  testAssetTypeMatch(ObjectInstanceShaderType::Phong, info, MM);

  ESP_DEBUG() << "Testing Flat explicitly being set.";
  // force flat via shadertype
  info.shaderTypeToUse = ObjectInstanceShaderType::Flat;
  testAssetTypeMatch(ObjectInstanceShaderType::Flat, info, MM);

}  // ResourceManagerTest::testFlatShaderTypeSpecification

}  // namespace

CORRADE_TEST_MAIN(ResourceManagerTest)
