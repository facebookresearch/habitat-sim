// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Utility/Directory.h>
#include <gtest/gtest.h>
#include <string>

#include "esp/sim/Simulator.h"

#include "esp/assets/ResourceManager.h"
#include "esp/gfx/Renderer.h"
#include "esp/scene/SceneManager.h"

#ifdef ESP_BUILD_WITH_BULLET
#include "esp/physics/bullet/BulletPhysicsManager.h"
#endif

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

        const esp::scene::SceneNode& node =
            physicsManager_->getObjectSceneNode(objectId);

        Magnum::Matrix4 R{
            Magnum::Matrix4::rotationX(Magnum::Math::Rad<float>(-1.56)) *
            Magnum::Matrix4::rotationY(Magnum::Math::Rad<float>(-0.25))};
        float boxHeight = 2.0 + (o * 2);
        Magnum::Vector3 initialPosition{0.0, boxHeight, 0.0};
        physicsManager_->setRotation(
            objectId, Magnum::Quaternion::fromMatrix(R.rotationNormalized()));
        physicsManager_->setTranslation(objectId, initialPosition);

        ASSERT_EQ(node.absoluteTranslation(), initialPosition);
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

#ifdef ESP_BUILD_WITH_BULLET
TEST_F(PhysicsManagerTest, CollisionBoundingBox) {
  LOG(INFO) << "Starting physics test: CollisionBoundingBox";

  std::string sceneFile =
      Cr::Utility::Directory::join(dataDir, "test_assets/scenes/plane.glb");
  std::string objectFile =
      Cr::Utility::Directory::join(dataDir, "test_assets/objects/sphere.glb");

  initScene(sceneFile);

  if (physicsManager_->getPhysicsSimulationLibrary() !=
      PhysicsManager::PhysicsSimulationLibrary::NONE) {
    // if we have a simulation implementation then test bounding box vs mesh for
    // sphere object

    esp::assets::PhysicsObjectAttributes physicsObjectAttributes;
    physicsObjectAttributes.setString("renderMeshHandle", objectFile);
    physicsObjectAttributes.setDouble("margin", 0.0);
    physicsObjectAttributes.setBool("joinCollisionMeshes", false);
    resourceManager_.loadObject(physicsObjectAttributes, objectFile);

    // get a reference to the stored template to edit
    esp::assets::PhysicsObjectAttributes& objectTemplate =
        resourceManager_.getPhysicsObjectAttributes(objectFile);

    for (int i = 0; i < 2; i++) {
      if (i == 0) {
        objectTemplate.setBool("useBoundingBoxForCollision", false);
      } else {
        objectTemplate.setBool("useBoundingBoxForCollision", true);
      }

      physicsManager_->reset();

      int objectId = physicsManager_->addObject(
          objectFile, &sceneManager_.getSceneGraph(sceneID_).getDrawables());

      Magnum::Vector3 initialPosition{0.0, 0.25, 0.0};
      physicsManager_->setTranslation(objectId, initialPosition);

      Magnum::Quaternion prevOrientation =
          physicsManager_->getRotation(objectId);
      Magnum::Vector3 prevPosition = physicsManager_->getTranslation(objectId);
      float timeToSim = 3.0;
      while (physicsManager_->getWorldTime() < timeToSim) {
        Magnum::Vector3 force{3.0, 0.0, 0.0};
        physicsManager_->applyForce(objectId, force, Magnum::Vector3{});
        physicsManager_->stepPhysics(0.1);

        Magnum::Quaternion orientation = physicsManager_->getRotation(objectId);
        Magnum::Vector3 position = physicsManager_->getTranslation(objectId);

        // object is being pushed, so should be moving
        ASSERT_NE(position, prevPosition);
        Magnum::Rad q_angle =
            Magnum::Math::angle(orientation, Magnum::Quaternion({0, 0, 0}, 1));
        if (i == 1) {
          // bounding box for collision, so the sphere should not be rolling
          ASSERT_LE(q_angle, Magnum::Rad{0.1});
        } else {
          // no bounding box, so the sphere should be rolling
          ASSERT_NE(orientation, prevOrientation);
        }

        prevOrientation = orientation;
        prevPosition = position;
      }

      physicsManager_->removeObject(objectId);
    }
  }
}

TEST_F(PhysicsManagerTest, DiscreteContactTest) {
  LOG(INFO) << "Starting physics test: ContactTest";

  std::string sceneFile =
      Cr::Utility::Directory::join(dataDir, "test_assets/scenes/plane.glb");
  std::string objectFile = Cr::Utility::Directory::join(
      dataDir, "test_assets/objects/transform_box.glb");

  initScene(sceneFile);

  if (physicsManager_->getPhysicsSimulationLibrary() !=
      PhysicsManager::PhysicsSimulationLibrary::NONE) {
    esp::assets::PhysicsObjectAttributes physicsObjectAttributes;
    physicsObjectAttributes.setString("renderMeshHandle", objectFile);
    physicsObjectAttributes.setDouble("margin", 0.0);
    resourceManager_.loadObject(physicsObjectAttributes, objectFile);

    // generate two centered boxes with dimension 2x2x2
    int objectId0 = physicsManager_->addObject(objectFile, nullptr);
    int objectId1 = physicsManager_->addObject(objectFile, nullptr);

    // place them in collision free location (0.1 about ground plane and 0.2
    // apart)
    physicsManager_->setTranslation(objectId0, Magnum::Vector3{0, 1.1, 0});
    physicsManager_->setTranslation(objectId1, Magnum::Vector3{2.2, 1.1, 0});
    ASSERT_FALSE(physicsManager_->contactTest(objectId0));
    ASSERT_FALSE(physicsManager_->contactTest(objectId1));

    // move box 0 into floor
    physicsManager_->setTranslation(objectId0, Magnum::Vector3{0, 0.9, 0});
    ASSERT_TRUE(physicsManager_->contactTest(objectId0));
    ASSERT_FALSE(physicsManager_->contactTest(objectId1));

    // move box 0 into box 1
    physicsManager_->setTranslation(objectId0, Magnum::Vector3{1.1, 1.1, 0});
    ASSERT_TRUE(physicsManager_->contactTest(objectId0));
    ASSERT_TRUE(physicsManager_->contactTest(objectId1));
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

    auto* drawables = &sceneManager_.getSceneGraph(sceneID_).getDrawables();

    // add the unjoined object
    objectTemplate.setBool("joinCollisionMeshes", false);
    int objectId0 = physicsManager_->addObject(objectFile, drawables);

    // add the joined object
    objectTemplate.setBool("joinCollisionMeshes", true);
    int objectId1 = physicsManager_->addObject(objectFile, drawables);

    // add bounding box object
    objectTemplate.setBool("useBoundingBoxForCollision", true);
    int objectId2 = physicsManager_->addObject(objectFile, drawables);

    esp::physics::BulletPhysicsManager* bPhysManager =
        static_cast<esp::physics::BulletPhysicsManager*>(physicsManager_.get());

    const Magnum::Range3D AabbScene =
        bPhysManager->getSceneCollisionShapeAabb();

    const Magnum::Range3D AabbOb0 =
        bPhysManager->getCollisionShapeAabb(objectId0);
    const Magnum::Range3D AabbOb1 =
        bPhysManager->getCollisionShapeAabb(objectId1);
    const Magnum::Range3D AabbOb2 =
        bPhysManager->getCollisionShapeAabb(objectId1);

    Magnum::Range3D objectGroundTruth({-1.1, -1.1, -1.1}, {1.1, 1.1, 1.1});
    Magnum::Range3D sceneGroundTruth({-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0});

    ASSERT_EQ(AabbScene, sceneGroundTruth);
    ASSERT_EQ(AabbOb0, objectGroundTruth);
    ASSERT_EQ(AabbOb1, objectGroundTruth);
    ASSERT_EQ(AabbOb2, objectGroundTruth);
  }
}
#endif

TEST_F(PhysicsManagerTest, ConfigurableScaling) {
  // test scaling of objects via template configuration (visual and collision)
  LOG(INFO) << "Starting physics test: ConfigurableScaling";

  std::string objectFile = Cr::Utility::Directory::join(
      dataDir, "test_assets/objects/transform_box.glb");

  initScene("NONE");

  // test joined vs. unjoined
  esp::assets::PhysicsObjectAttributes physicsObjectAttributes;
  physicsObjectAttributes.setString("renderMeshHandle", objectFile);
  physicsObjectAttributes.setDouble("margin", 0.0);

  resourceManager_.loadObject(physicsObjectAttributes, objectFile);

  // get a reference to the stored template to edit
  esp::assets::PhysicsObjectAttributes& objectTemplate =
      resourceManager_.getPhysicsObjectAttributes(objectFile);

  std::vector<Magnum::Vector3> testScales{
      {1.0, 1.0, 1.0},  {4.0, 3.0, 2.0},    {0.1, 0.2, 0.3},
      {0.0, 0.0, 0.0},  {-1.0, -1.0, -1.0}, {-1.0, 1.0, 1.0},
      {4.0, -3.0, 2.0}, {0.1, -0.2, -0.3}};

  auto& drawables = sceneManager_.getSceneGraph(sceneID_).getDrawables();

  for (auto& testScale : testScales) {
    objectTemplate.setMagnumVec3("scale", testScale);

    Magnum::Range3D boundsGroundTruth(-abs(testScale), abs(testScale));

    int objectId = physicsManager_->addObject(objectFile, &drawables);

    const Magnum::Range3D& visualBounds =
        physicsManager_->getObjectSceneNode(objectId).getCumulativeBB();

    ASSERT_EQ(visualBounds, boundsGroundTruth);

// Test Bullet collision shape scaling
#ifdef ESP_BUILD_WITH_BULLET
    if (physicsManager_->getPhysicsSimulationLibrary() ==
        PhysicsManager::PhysicsSimulationLibrary::BULLET) {
      esp::physics::BulletPhysicsManager* bPhysManager =
          static_cast<esp::physics::BulletPhysicsManager*>(
              physicsManager_.get());

      Magnum::Range3D aabb = bPhysManager->getCollisionShapeAabb(objectId);

      ASSERT_EQ(aabb, boundsGroundTruth);
    }
#endif
  }
}

TEST_F(PhysicsManagerTest, TestVelocityControl) {
  // test scaling of objects via template configuration (visual and collision)
  LOG(INFO) << "Starting physics test: TestVelocityControl";

  std::string objectFile = Cr::Utility::Directory::join(
      dataDir, "test_assets/objects/transform_box.glb");

  std::string sceneFile =
      Cr::Utility::Directory::join(dataDir, "test_assets/scenes/plane.glb");

  initScene(sceneFile);

  esp::assets::PhysicsObjectAttributes physicsObjectAttributes;
  physicsObjectAttributes.setString("renderMeshHandle", objectFile);
  physicsObjectAttributes.setDouble("margin", 0.0);
  physicsObjectAttributes.setDouble("angDamping", 0.0);
  resourceManager_.loadObject(physicsObjectAttributes, objectFile);

  auto& drawables = sceneManager_.getSceneGraph(sceneID_).getDrawables();

  int objectId = physicsManager_->addObject(objectFile, &drawables);
  physicsManager_->setTranslation(objectId, Magnum::Vector3{0, 2.0, 0});

  Magnum::Vector3 commandLinVel(1.0, 1.0, 1.0);
  Magnum::Vector3 commandAngVel(1.0, 1.0, 1.0);

  if (physicsManager_->getPhysicsSimulationLibrary() ==
      PhysicsManager::PhysicsSimulationLibrary::BULLET) {
    physicsManager_->setLinearVelocity(objectId, commandLinVel);
    physicsManager_->setAngularVelocity(objectId, commandAngVel);

    ASSERT_EQ(physicsManager_->getLinearVelocity(objectId), commandLinVel);
    ASSERT_EQ(physicsManager_->getAngularVelocity(objectId), commandAngVel);

    // run some tests with dynamics
    float timeToSim = 5.0;
    float steptime = 0.05;
    commandAngVel = Magnum::Vector3{
        1.0, 1.0, -1.0};  // axis rotation chosen for easy checking
    esp::physics::VelocityControl& velControl =
        physicsManager_->getVelocityControl(objectId);
    velControl.controllingAngVel = true;
    velControl.controllingLinVel = true;
    velControl.linVel = commandLinVel;
    velControl.angVel = commandAngVel;

    Magnum::Quaternion kinematicOrientation =
        physicsManager_->getRotation(objectId);

    while (physicsManager_->getWorldTime() < timeToSim) {
      Magnum::Vector3 prevPos = physicsManager_->getTranslation(objectId);
      Magnum::Quaternion prevOrientation =
          physicsManager_->getRotation(objectId);

      physicsManager_->applyForce(
          objectId,
          Magnum::Vector3{0, float(9.8 * physicsManager_->getMass(objectId)),
                          0},
          Magnum::Vector3{});

      physicsManager_->stepPhysics(steptime);

      // LOG(INFO) << "time: " << physicsManager_->getWorldTime();

      // check that linear velocity has been approximately applied (could be
      // some error due to dynamics)
      for (int i = 0; i < 3; i++) {
        ASSERT_EQ((prevPos[i] + commandLinVel[i] > prevPos[i]),
                  (physicsManager_->getTranslation(objectId)[i] > prevPos[i]));
      }

      // check that angular velocity has been approximately applied via
      // exponential map
      Magnum::Quaternion q;
      Magnum::Vector3 ha =
          commandAngVel * steptime * 0.5;  // vector of half angle
      float l = ha.length();               // magnitude
      if (l > 0) {
        float ss = sin(l) / l;
        q = Magnum::Quaternion(
            Magnum::Vector3{ha.x() * ss, ha.y() * ss, ha.z() * ss}, cos(l));
      } else {
        q = Magnum::Quaternion(Magnum::Vector3{ha.x(), ha.y(), ha.z()}, 1.0);
      }
      Magnum::Quaternion finiteDifference =
          physicsManager_->getRotation(objectId) *
          prevOrientation
              .inverted();  // quat diff: diff*q1 = q2 -> diff = q2*inv(q1)
      Cr::Utility::Debug() << "kin delta: " << q
                           << ", dyn delta: " << finiteDifference;
      Magnum::Quaternion expectedOrientation = q * prevOrientation;
      Magnum::Rad q_angle = Magnum::Math::angle(
          physicsManager_->getRotation(objectId), expectedOrientation);
      Cr::Utility::Debug() << "orientation: "
                           << physicsManager_->getRotation(objectId)
                           << ", expected: " << expectedOrientation
                           << ", angle b/t: " << q_angle;

      for (int i = 0; i < 3; i++) {
        ASSERT_EQ(
            (expectedOrientation.vector()[i] > prevOrientation.vector()[i]),
            (physicsManager_->getRotation(objectId).vector()[i] >
             prevOrientation.vector()[i]));
      }
      ASSERT_EQ((expectedOrientation.scalar() > prevOrientation.scalar()),
                (physicsManager_->getRotation(objectId).scalar() >
                 prevOrientation.scalar()));

      // ASSERT_LE(q_angle, Magnum::Rad{0.1});
    }

  } else if (physicsManager_->getPhysicsSimulationLibrary() ==
             PhysicsManager::PhysicsSimulationLibrary::NONE) {
    physicsManager_->setLinearVelocity(objectId, commandLinVel);
    physicsManager_->setAngularVelocity(objectId, commandAngVel);

    // default kinematics always 0 velocity when queried
    ASSERT_EQ(physicsManager_->getLinearVelocity(objectId), Magnum::Vector3{});
    ASSERT_EQ(physicsManager_->getAngularVelocity(objectId), Magnum::Vector3{});
  }
}
