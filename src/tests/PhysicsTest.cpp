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

class PhysicsManagerTest : public testing::Test {
 protected:
  void SetUp() override {
    context_ = esp::gfx::WindowlessContext::create_unique(0);
    renderer_ = esp::gfx::Renderer::create();

    sceneID_ = sceneManager_.initSceneGraph();
  };

  void initScene(const std::string sceneFile) {
    sceneFile_ = sceneFile;

    const esp::assets::AssetInfo info =
        esp::assets::AssetInfo::fromPath(sceneFile);

    auto& sceneGraph = sceneManager_.getSceneGraph(sceneID_);
    esp::scene::SceneNode* navSceneNode =
        &sceneGraph.getRootNode().createChild();
    auto& drawables = sceneManager_.getSceneGraph(sceneID_).getDrawables();
    resourceManager_.loadScene(info, physicsManager_, navSceneNode, &drawables,
                               physicsConfigFile);
  }

  // must declare these in this order due to avoid deallocation errors
  esp::gfx::WindowlessContext::uptr context_;
  esp::gfx::Renderer::ptr renderer_;

  ResourceManager resourceManager_;
  SceneManager sceneManager_;
  PhysicsManager::ptr physicsManager_;

  std::string sceneFile_;

  int sceneID_;
};

TEST_F(PhysicsManagerTest, JoinCompound) {
  LOG(INFO) << "Starting physics test: JoinCompound";

  std::string sceneFile =
      Cr::Utility::Directory::join(dataDir, "test_assets/scenes/plane.glb");
  std::string objectFile = Cr::Utility::Directory::join(
      dataDir, "test_assets/objects/nested_box.glb");

  initScene(sceneFile);

  if (physicsManager_->getPhysicsSimulationLibrary() !=
      PhysicsManager::PhysicsSimulationLibrary::NONE) {
    // if we have a simulation implementation then test a joined vs. unjoined
    // object
    esp::assets::PhysicsObjectAttributes physicsObjectAttributes;
    physicsObjectAttributes.setString("renderMeshHandle", objectFile);
    resourceManager_.loadObject(physicsObjectAttributes, objectFile);

    // get a reference to the stored template to edit
    esp::assets::PhysicsObjectAttributes& objectTemplate =
        resourceManager_.getPhysicsObjectAttributes(objectFile);

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

TEST_F(PhysicsManagerTest, BulletCompoundShapeMargins) {
  // test that all different construction methods for a simple shape result in
  // the same Aabb for the given margin
  LOG(INFO) << "Starting physics test: BulletCompoundShapeMargins";

  std::string objectFile = Cr::Utility::Directory::join(
      dataDir, "test_assets/objects/transform_box.glb");

  initScene(objectFile);

  if (physicsManager_->getPhysicsSimulationLibrary() ==
      PhysicsManager::PhysicsSimulationLibrary::BULLET) {
    // test joined vs. unjoined
    esp::assets::PhysicsObjectAttributes physicsObjectAttributes;
    physicsObjectAttributes.setString("renderMeshHandle", objectFile);
    physicsObjectAttributes.setDouble("margin", 0.1);
    resourceManager_.loadObject(physicsObjectAttributes, objectFile);

    // get a reference to the stored template to edit
    esp::assets::PhysicsObjectAttributes& objectTemplate =
        resourceManager_.getPhysicsObjectAttributes(objectFile);

    // add the unjoined object
    objectTemplate.setBool("joinCollisionMeshes", false);
    int objectId0 = physicsManager_->addObject(objectFile, nullptr);

    // add the joined object
    objectTemplate.setBool("joinCollisionMeshes", true);
    int objectId1 = physicsManager_->addObject(objectFile, nullptr);

    esp::physics::BulletPhysicsManager* bPhysManager =
        static_cast<esp::physics::BulletPhysicsManager*>(physicsManager_.get());

    std::pair<Magnum::Vector3, Magnum::Vector3> AabbScene =
        bPhysManager->getSceneCollisionShapeAabb();

    std::pair<Magnum::Vector3, Magnum::Vector3> AabbOb0 =
        bPhysManager->getCollisionShapeAabb(objectId0);
    std::pair<Magnum::Vector3, Magnum::Vector3> AabbOb1 =
        bPhysManager->getCollisionShapeAabb(objectId1);

    std::pair<Magnum::Vector3, Magnum::Vector3> objectGroundTruth =
        std::pair<Magnum::Vector3, Magnum::Vector3>({-1.1, -1.1, -1.1},
                                                    {1.1, 1.1, 1.1});
    std::pair<Magnum::Vector3, Magnum::Vector3> sceneGroundTruth =
        std::pair<Magnum::Vector3, Magnum::Vector3>({-1.0, -1.0, -1.0},
                                                    {1.0, 1.0, 1.0});

    // Cr::Utility::Debug() << "aabb scene: " << AabbScene;
    // Cr::Utility::Debug() << "aabb ob0: " << AabbOb0;
    // Cr::Utility::Debug() << "aabb ob1: " << AabbOb1;
    ASSERT_EQ(AabbScene, sceneGroundTruth);
    ASSERT_EQ(AabbOb0, objectGroundTruth);
    ASSERT_EQ(AabbOb1, objectGroundTruth);
  }
}
