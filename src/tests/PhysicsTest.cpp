// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Utility/Directory.h>
#include <gtest/gtest.h>
#include <string>

#include "esp/gfx/Simulator.h"

#include "esp/assets/ResourceManager.h"
#include "esp/gfx/Renderer.h"
#include "esp/physics/bullet/BulletPhysicsManager.h"
#include "esp/scene/SceneManager.h"

#include "configure.h"

namespace Cr = Corrade;

using esp::assets::ResourceManager;
using esp::physics::PhysicsManager;
using esp::scene::SceneManager;

const std::string dataDir = Cr::Utility::Directory::join(SCENE_DATASETS, "../");
const std::string physicsConfigFile =
    Cr::Utility::Directory::join(SCENE_DATASETS,
                                 "../default.phys_scene_config.json");

TEST(PhysicsTest, JoinCompound) {
  LOG(INFO) << "Starting physics test";
  // must create a context and renderer first
  esp::gfx::WindowlessContext::uptr context_ =
      esp::gfx::WindowlessContext::create_unique(0);
  std::shared_ptr<esp::gfx::Renderer> renderer_ = esp::gfx::Renderer::create();

  // must declare these in this order due to avoid deallocation errors
  ResourceManager resourceManager;
  SceneManager sceneManager_;
  std::shared_ptr<PhysicsManager> physicsManager_;

  std::string sceneFile =
      Cr::Utility::Directory::join(dataDir, "test_assets/scenes/plane.glb");
  std::string objectFile = Cr::Utility::Directory::join(
      dataDir, "test_assets/objects/nested_box.glb");

  int sceneID = sceneManager_.initSceneGraph();
  auto& sceneGraph = sceneManager_.getSceneGraph(sceneID);
  esp::scene::SceneNode* navSceneNode = &sceneGraph.getRootNode().createChild();
  auto& drawables = sceneManager_.getSceneGraph(sceneID).getDrawables();
  const esp::assets::AssetInfo info =
      esp::assets::AssetInfo::fromPath(sceneFile);

  resourceManager.loadScene(info, physicsManager_, navSceneNode, nullptr,
                            physicsConfigFile);

  if (physicsManager_->getPhysicsSimulationLibrary() !=
      PhysicsManager::PhysicsSimulationLibrary::NONE) {
    // if we have a simulation implementation then test a joined vs. unjoined
    // object
    esp::assets::PhysicsObjectAttributes physicsObjectAttributes;
    physicsObjectAttributes.setString("renderMeshHandle", objectFile);
    resourceManager.loadObject(physicsObjectAttributes, objectFile);

    // get a reference to the stored template to edit
    esp::assets::PhysicsObjectAttributes& objectTemplate =
        resourceManager.getPhysicsObjectAttributes(objectFile);

    for (int i = 0; i < 2; i++) {
      // mark the object not joined
      if (i == 0) {
        objectTemplate.setBool("joinCollisionMeshes", false);
      } else {
        objectTemplate.setBool("joinCollisionMeshes", true);
      }

      physicsManager_->reset();

      std::vector<int> objectIds;

      // add and simulate the object
      int num_objects = 7;
      for (int o = 0; o < num_objects; o++) {
        int objectId = physicsManager_->addObject(objectFile, nullptr);
        objectIds.push_back(o);
        Magnum::Matrix4 R{
            Magnum::Matrix4::rotationX(Magnum::Math::Rad<float>(-1.56)) *
            Magnum::Matrix4::rotationY(Magnum::Math::Rad<float>(-0.25))};
        float boxHeight = 2.0 + (o * 2);
        Magnum::Vector3 initialPosition{0.0, boxHeight, 0.0};
        physicsManager_->setRotation(
            objectId, Magnum::Quaternion::fromMatrix(R.rotationNormalized()));
        physicsManager_->setTranslation(objectId, initialPosition);
      }

      float timeToSim = 10.0;
      while (physicsManager_->getWorldTime() < timeToSim) {
        physicsManager_->stepPhysics(0.1);
      }
      int numActiveObjects = physicsManager_->checkActiveObjects();
      LOG(INFO) << " Number of active objects: " << numActiveObjects;

      if (i == 1) {
        // when collision meshes are joined, objects should be stable
        ASSERT_EQ(numActiveObjects, 0);
      }

      for (int o : objectIds) {
        physicsManager_->removeObject(o);
      }
    }
  }
}
