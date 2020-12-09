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
#include "esp/gfx/replay/Recorder.h"
#include "esp/scene/SceneManager.h"

#include "configure.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

using esp::assets::ResourceManager;
using esp::metadata::MetadataMediator;
using esp::scene::SceneManager;

// Manipulate the scene and save some keyframes using replay::Recorder
TEST(GfxReplayTest, recorder) {
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
  ASSERT(node);
  esp::gfx::replay::Recorder recorder;
  recorder.onLoadRenderAsset(info);
  recorder.onCreateRenderAssetInstance(node, creation);
  recorder.saveKeyframe();
  node->setTranslation(Mn::Vector3(1.f, 2.f, 3.f));
  node->setSemanticId(7);
  recorder.saveKeyframe();
  delete node;
  recorder.addUserTransformToKeyframe("my_user_transform",
                                      Mn::Vector3(4.f, 5.f, 6.f),
                                      Mn::Quaternion(Mn::Math::ZeroInit));
  recorder.saveKeyframe();

  // verify 3 saved keyframes
  const auto& keyframes = recorder.debugGetSavedKeyframes();
  ASSERT(keyframes.size() == 3);

  // verify frame #0 loads a render asset, creates an instance, and stores a
  // state update for the instance
  ASSERT(keyframes[0].loads.size() == 1);
  ASSERT(keyframes[0].loads[0] == info);
  ASSERT(keyframes[0].creations.size() == 1);
  ASSERT(keyframes[0].creations[0].second.filepath.find(
             "objects/transform_box.glb") != std::string::npos);
  ASSERT(keyframes[0].stateUpdates.size() == 1);
  esp::gfx::replay::RenderAssetInstanceKey instanceKey =
      keyframes[0].creations[0].first;
  ASSERT(keyframes[0].stateUpdates[0].first == instanceKey);

  // verify frame #1 has our translation and semantic Id
  ASSERT(keyframes[1].stateUpdates.size() == 1);
  ASSERT(keyframes[1].stateUpdates[0].second.absTransform.translation ==
         Mn::Vector3(1.f, 2.f, 3.f));
  ASSERT(keyframes[1].stateUpdates[0].second.semanticId == 7);

  // verify frame #2 has our deletion and our user transform
  ASSERT(keyframes[2].deletions.size() == 1);
  ASSERT(keyframes[2].deletions[0] == instanceKey);
  ASSERT(keyframes[2].userTransforms.size() == 1);
  ASSERT(keyframes[2].userTransforms.count("my_user_transform"));
  ASSERT(keyframes[2].userTransforms.at("my_user_transform").translation ==
         Mn::Vector3(4.f, 5.f, 6.f));
}
