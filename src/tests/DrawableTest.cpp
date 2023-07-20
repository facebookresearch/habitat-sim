// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/TestSuite/Tester.h>
#include <Corrade/Utility/Path.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Shaders/FlatGL.h>
#include <Magnum/Trade/MeshData.h>
#include "esp/assets/ResourceManager.h"
#include "esp/gfx/GenericDrawable.h"
#include "esp/gfx/RenderTarget.h"
#include "esp/gfx/WindowlessContext.h"
#include "esp/metadata/MetadataMediator.h"
#include "esp/scene/SceneManager.h"

#include "configure.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

using esp::assets::ResourceManager;
using esp::metadata::MetadataMediator;
using esp::scene::SceneManager;

// on GCC and Clang, the following namespace causes useful warnings to be
// printed when you have accidentally unused variables or functions in the test
namespace {

struct DrawableTest : Cr::TestSuite::Tester {
  explicit DrawableTest();
  // tests
  void addRemoveDrawables();

 protected:
  esp::logging::LoggingContext loggingContext_;
  esp::gfx::WindowlessContext::uptr context_ =
      esp::gfx::WindowlessContext::create_unique(0);
  // must declare these in this order due to avoid deallocation errors
  std::unique_ptr<ResourceManager> resourceManager_ = nullptr;
  SceneManager sceneManager_;
  // must create a GL context which will be used in the resource manager
  int sceneID_ = -1;
  esp::gfx::DrawableGroup* drawableGroup_;
};

DrawableTest::DrawableTest() {
  auto cfg = esp::sim::SimulatorConfiguration{};
  // setting values for stage load
  cfg.loadSemanticMesh = false;
  cfg.forceSeparateSemanticSceneGraph = false;
  auto MM = MetadataMediator::create(cfg);
  resourceManager_ = std::make_unique<ResourceManager>(MM);
  //clang-format off
  addTests({&DrawableTest::addRemoveDrawables});
  //clang-format on
  auto stageAttributesMgr = MM->getStageAttributesManager();
  std::string stageFile =
      Cr::Utility::Path::join(TEST_ASSETS, "objects/5boxes.glb");
  auto stageAttributes = stageAttributesMgr->createObject(stageFile, true);

  sceneID_ = sceneManager_.initSceneGraph();
  auto& sceneGraph = sceneManager_.getSceneGraph(sceneID_);
  drawableGroup_ = &sceneGraph.getDrawables();

  std::vector<int> tempIDs{sceneID_, esp::ID_UNDEFINED};
  bool result = resourceManager_->loadStage(stageAttributes, nullptr, nullptr,
                                            &sceneManager_, tempIDs);
}

void DrawableTest::addRemoveDrawables() {
  Mn::GL::Mesh box;
  Mn::Shaders::FlatGL3D shader;

  Mn::Trade::MeshData cube = Mn::Primitives::cubeSolidStrip();
  box = Mn::MeshTools::compile(cube);
  auto& sceneGraph = sceneManager_.getSceneGraph(sceneID_);
  esp::scene::SceneNode& sceneRootNode = sceneGraph.getRootNode();

  esp::scene::SceneNode& node = sceneRootNode.createChild();

  esp::gfx::Drawable::Flags meshAttributeFlags{};
  esp::gfx::DrawableConfiguration cfg{
      esp::NO_LIGHT_KEY,
      esp::PER_VERTEX_OBJECT_ID_MATERIAL_KEY,
      esp::metadata::attributes::ObjectInstanceShaderType::Phong,
      drawableGroup_,  // DrawableGroup
      nullptr,         // Skin
      nullptr,         // PbrIBLHelper
      nullptr};        // PbrShaderAttributes
  // add a toy box here!
  node.addFeature<esp::gfx::GenericDrawable>(
      &box, meshAttributeFlags, resourceManager_->getShaderManager(), cfg);
  // we already had 5 boxes in the scene, so the id for the above toy box must
  // be 5
  int toyBoxId = 5;

  // step 1: basic tests
  esp::gfx::GenericDrawable* dr =
      new esp::gfx::GenericDrawable{node, &box, meshAttributeFlags,
                                    resourceManager_->getShaderManager(), cfg};

  // we already had 5 boxes in the scene, 1 toy box added before the current
  // one, so the id should be 6
  CORRADE_COMPARE(dr->getDrawableId(), 6);
  // verify this drawable has been added to drawable group
  CORRADE_COMPARE(dr->drawables(), drawableGroup_);

  esp::gfx::DrawableConfiguration cfg_nullgrp{
      esp::NO_LIGHT_KEY,
      esp::PER_VERTEX_OBJECT_ID_MATERIAL_KEY,
      esp::metadata::attributes::ObjectInstanceShaderType::Phong,
      nullptr,   // DrawableGroup
      nullptr,   // Skin
      nullptr,   // PbrIBLHelper
      nullptr};  // PbrShaderAttributes

  // step 2: add a single drawable to a group
  dr = new esp::gfx::GenericDrawable{node, &box, meshAttributeFlags,
                                     resourceManager_->getShaderManager(),
                                     cfg_nullgrp};
  // it has NOT been added to this group, so it should not find it!
  CORRADE_VERIFY(!drawableGroup_->hasDrawable(dr->getDrawableId()));
  drawableGroup_->add(*dr);
  // Now, it has been added to this group, check it again
  CORRADE_VERIFY(drawableGroup_->hasDrawable(dr->getDrawableId()));

  // step 3: delete the previous drawable, and it should be removed from the
  // drawable group
  int id = dr->getDrawableId();
  delete dr;
  CORRADE_VERIFY(!drawableGroup_->hasDrawable(id));

  // step 4: remove a drawable, that is in the drawable group
  auto* toyBoxDrawable = drawableGroup_->getDrawable(toyBoxId);
  CORRADE_VERIFY(toyBoxDrawable);
  drawableGroup_->remove(*toyBoxDrawable);
  CORRADE_VERIFY(!drawableGroup_->hasDrawable(toyBoxId));

  // step 5: remove a drawable, that is NOT in the group! should be fine!!
  dr = new esp::gfx::GenericDrawable{node, &box, meshAttributeFlags,
                                     resourceManager_->getShaderManager(),
                                     cfg_nullgrp};
  drawableGroup_->remove(*dr);
  CORRADE_VERIFY(!drawableGroup_->hasDrawable(dr->getDrawableId()));
}

}  // namespace

CORRADE_TEST_MAIN(DrawableTest)
