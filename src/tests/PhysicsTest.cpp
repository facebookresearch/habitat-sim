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

class PhysicsTestWorld {
 public:
  PhysicsTestWorld(const std::string sceneFile) {
    sceneFile_ = sceneFile;

    context_ = esp::gfx::WindowlessContext::create_unique(0);
    renderer_ = esp::gfx::Renderer::create();

    sceneID_ = sceneManager_.initSceneGraph();
    auto& sceneGraph = sceneManager_.getSceneGraph(sceneID_);
    esp::scene::SceneNode* navSceneNode =
        &sceneGraph.getRootNode().createChild();
    auto& drawables = sceneManager_.getSceneGraph(sceneID_).getDrawables();
    const esp::assets::AssetInfo info =
        esp::assets::AssetInfo::fromPath(sceneFile);

    resourceManager_.loadScene(info, physicsManager_, navSceneNode, &drawables,
                               physicsConfigFile);
  };

  // must declare these in this order due to avoid deallocation errors
  esp::gfx::WindowlessContext::uptr context_;
  esp::gfx::Renderer::ptr renderer_;

  ResourceManager resourceManager_;
  SceneManager sceneManager_;
  PhysicsManager::ptr physicsManager_;

  std::string sceneFile_;

  int sceneID_;
};

TEST(PhysicsTest, JoinCompound) {
  LOG(INFO) << "Starting physics test: JoinCompound";

  std::string sceneFile =
      Cr::Utility::Directory::join(dataDir, "test_assets/scenes/plane.glb");
  std::string objectFile = Cr::Utility::Directory::join(
      dataDir, "test_assets/objects/nested_box.glb");

  PhysicsTestWorld physicsTestWorld(sceneFile);

  if (physicsTestWorld.physicsManager_->getPhysicsSimulationLibrary() !=
      PhysicsManager::PhysicsSimulationLibrary::NONE) {
    // if we have a simulation implementation then test a joined vs. unjoined
    // object
    esp::assets::PhysicsObjectAttributes physicsObjectAttributes;
    physicsObjectAttributes.setString("renderMeshHandle", objectFile);
    physicsTestWorld.resourceManager_.loadObject(physicsObjectAttributes,
                                                 objectFile);

    // get a reference to the stored template to edit
    esp::assets::PhysicsObjectAttributes& objectTemplate =
        physicsTestWorld.resourceManager_.getPhysicsObjectAttributes(
            objectFile);

    for (int i = 0; i < 2; i++) {
      // mark the object not joined
      if (i == 0) {
        objectTemplate.setBool("joinCollisionMeshes", false);
      } else {
        objectTemplate.setBool("joinCollisionMeshes", true);
      }

      physicsTestWorld.physicsManager_->reset();

      std::vector<int> objectIds;

      // add and simulate the object
      int num_objects = 7;
      for (int o = 0; o < num_objects; o++) {
        int objectId =
            physicsTestWorld.physicsManager_->addObject(objectFile, nullptr);
        objectIds.push_back(o);
        Magnum::Matrix4 R{
            Magnum::Matrix4::rotationX(Magnum::Math::Rad<float>(-1.56)) *
            Magnum::Matrix4::rotationY(Magnum::Math::Rad<float>(-0.25))};
        float boxHeight = 2.0 + (o * 2);
        Magnum::Vector3 initialPosition{0.0, boxHeight, 0.0};
        physicsTestWorld.physicsManager_->setRotation(
            objectId, Magnum::Quaternion::fromMatrix(R.rotationNormalized()));
        physicsTestWorld.physicsManager_->setTranslation(objectId,
                                                         initialPosition);
      }

      float timeToSim = 10.0;
      while (physicsTestWorld.physicsManager_->getWorldTime() < timeToSim) {
        physicsTestWorld.physicsManager_->stepPhysics(0.1);
      }
      int numActiveObjects =
          physicsTestWorld.physicsManager_->checkActiveObjects();
      LOG(INFO) << " Number of active objects: " << numActiveObjects;

      if (i == 1) {
        // when collision meshes are joined, objects should be stable
        ASSERT_EQ(numActiveObjects, 0);
      }

      for (int o : objectIds) {
        physicsTestWorld.physicsManager_->removeObject(o);
      }
    }
  }
}

TEST(PhysicsTest, CollisionBoundingBox) {
  LOG(INFO) << "Starting physics test: CollisionBoundingBox";

  std::string sceneFile =
      Cr::Utility::Directory::join(dataDir, "test_assets/scenes/plane.glb");
  std::string objectFile =
      // Cr::Utility::Directory::join(dataDir,
      // "test_assets/objects/sphere.glb");
      Cr::Utility::Directory::join(dataDir,
                                   "test_assets/objects/transform_box.glb");

  PhysicsTestWorld physicsTestWorld(sceneFile);

  if (physicsTestWorld.physicsManager_->getPhysicsSimulationLibrary() !=
      PhysicsManager::PhysicsSimulationLibrary::NONE) {
    // if we have a simulation implementation then test bounding box vs mesh for
    // sphere object

    esp::assets::PhysicsObjectAttributes physicsObjectAttributes;
    physicsObjectAttributes.setString("renderMeshHandle", objectFile);
    physicsObjectAttributes.setDouble("margin", 0.1);
    physicsObjectAttributes.setBool("joinCollisionMeshes", false);
    physicsTestWorld.resourceManager_.loadObject(physicsObjectAttributes,
                                                 objectFile);

    // get a reference to the stored template to edit
    esp::assets::PhysicsObjectAttributes& objectTemplate =
        physicsTestWorld.resourceManager_.getPhysicsObjectAttributes(
            objectFile);

    for (int i = 0; i < 2; i++) {
      LOG(INFO) << "Pass " << i;
      if (i == 0) {
        objectTemplate.setBool("useBoundingBoxForCollision", false);
      } else {
        objectTemplate.setBool("useBoundingBoxForCollision", true);
      }

      physicsTestWorld.physicsManager_->reset();

      int objectId = physicsTestWorld.physicsManager_->addObject(
          objectFile, &physicsTestWorld.sceneManager_
                           .getSceneGraph(physicsTestWorld.sceneID_)
                           .getDrawables());

      Magnum::Vector3 initialPosition{0.0, 0.25, 0.0};
      physicsTestWorld.physicsManager_->setTranslation(objectId,
                                                       initialPosition);

      Magnum::Quaternion prevOrientation =
          physicsTestWorld.physicsManager_->getRotation(objectId);
      Magnum::Vector3 prevPosition =
          physicsTestWorld.physicsManager_->getTranslation(objectId);
      float timeToSim = 5.0;
      while (physicsTestWorld.physicsManager_->getWorldTime() < timeToSim) {
        Magnum::Vector3 force{1.5, 0.0, 0.0};
        physicsTestWorld.physicsManager_->applyForce(objectId, force,
                                                     Magnum::Vector3{});
        physicsTestWorld.physicsManager_->stepPhysics(0.1);

        Magnum::Quaternion orientation =
            physicsTestWorld.physicsManager_->getRotation(objectId);
        Magnum::Vector3 position =
            physicsTestWorld.physicsManager_->getTranslation(objectId);

        // Cr::Utility::Debug() << "prev vs current position: " << prevPosition
        // << " | " << position; Cr::Utility::Debug() << "prev vs current
        // orientation: " << prevOrientation << " | " << orientation;

        // object is being pushed, so should be moving
        // ASSERT_NE(position, prevPosition);
        if (i == 1) {
          // bounding box for collision, so the sphere should not be rolling
          // ASSERT_EQ(orientation, Magnum::Quaternion({0, 0, 0}, 1));
        } else {
          // no bounding box, so the sphere should be rolling
          // ASSERT_NE(orientation, prevOrientation);
        }

        prevOrientation = orientation;
        prevPosition = position;
      }

      physicsTestWorld.physicsManager_->removeObject(objectId);
    }
  }
}
