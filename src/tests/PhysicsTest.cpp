// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Utility/Directory.h>
#include <gtest/gtest.h>
#include <string>

#include "esp/sim/Simulator.h"

#include "esp/assets/ResourceManager.h"
#include "esp/scene/SceneManager.h"

#include "esp/physics/PhysicsManager.h"
#ifdef ESP_BUILD_WITH_BULLET
#include "esp/physics/bullet/BulletPhysicsManager.h"
#endif

#include "configure.h"

namespace Cr = Corrade;

namespace AttrMgrs = esp::metadata::managers;
using AttrMgrs::ObjectAttributesManager;
using AttrMgrs::PhysicsAttributesManager;
using esp::assets::ResourceManager;
using esp::metadata::MetadataMediator;
using esp::metadata::attributes::ObjectAttributes;
using esp::physics::PhysicsManager;
using esp::scene::SceneManager;

const std::string dataDir = Cr::Utility::Directory::join(SCENE_DATASETS, "../");
const std::string physicsConfigFile =
    Cr::Utility::Directory::join(SCENE_DATASETS,
                                 "../default.physics_config.json");

class PhysicsManagerTest : public testing::Test {
 protected:
  void SetUp() override {
    // set up a default simulation config to initialize MM
    auto cfg = esp::sim::SimulatorConfiguration{};
    metadataMediator_ = MetadataMediator::create(cfg);
    resourceManager_ = std::make_unique<ResourceManager>(metadataMediator_);
    context_ = esp::gfx::WindowlessContext::create_unique(0);

    sceneID_ = sceneManager_.initSceneGraph();
    // get attributes manager for physics world attributes
    physicsAttributesManager_ =
        metadataMediator_->getPhysicsAttributesManager();
  };

  void initStage(const std::string stageFile) {
    auto& sceneGraph = sceneManager_.getSceneGraph(sceneID_);
    auto& rootNode = sceneGraph.getRootNode();

    // construct appropriate physics attributes based on config file
    auto physicsManagerAttributes =
        physicsAttributesManager_->createObject(physicsConfigFile, true);
    auto stageAttributesMgr = metadataMediator_->getStageAttributesManager();
    if (physicsManagerAttributes != nullptr) {
      stageAttributesMgr->setCurrPhysicsManagerAttributesHandle(
          physicsManagerAttributes->getHandle());
    }
    auto stageAttributes = stageAttributesMgr->createObject(stageFile, true);

    // construct physics manager based on specifications in attributes
    resourceManager_->initPhysicsManager(physicsManager_, true, &rootNode,
                                         physicsManagerAttributes);

    // load scene
    std::vector<int> tempIDs{sceneID_, esp::ID_UNDEFINED};
    bool result = resourceManager_->loadStage(stageAttributes, physicsManager_,
                                              &sceneManager_, tempIDs, false);
  }

  // must declare these in this order due to avoid deallocation errors
  esp::gfx::WindowlessContext::uptr context_;

  std::shared_ptr<MetadataMediator> metadataMediator_ = nullptr;
  std::unique_ptr<ResourceManager> resourceManager_ = nullptr;

  AttrMgrs::PhysicsAttributesManager::ptr physicsAttributesManager_;
  SceneManager sceneManager_;
  PhysicsManager::ptr physicsManager_;

  int sceneID_;
};

TEST_F(PhysicsManagerTest, JoinCompound) {
  LOG(INFO) << "Starting physics test: JoinCompound";

  std::string stageFile = Cr::Utility::Directory::join(
      dataDir, "test_assets/scenes/simple_room.glb");
  std::string objectFile = Cr::Utility::Directory::join(
      dataDir, "test_assets/objects/nested_box.glb");

  initStage(stageFile);

  if (physicsManager_->getPhysicsSimulationLibrary() !=
      PhysicsManager::PhysicsSimulationLibrary::NONE) {
    // if we have a simulation implementation then test a joined vs. unjoined
    // object
    // ObjectAttributes ObjectAttributes;
    ObjectAttributes::ptr ObjectAttributes = ObjectAttributes::create();
    ObjectAttributes->setRenderAssetHandle(objectFile);
    auto objectAttributesManager =
        metadataMediator_->getObjectAttributesManager();
    objectAttributesManager->registerObject(ObjectAttributes, objectFile);

    // get a reference to the stored template to edit
    ObjectAttributes::ptr objectTemplate =
        objectAttributesManager->getObjectCopyByHandle(objectFile);

    for (int i = 0; i < 2; i++) {
      // mark the object not joined
      if (i == 0) {
        objectTemplate->setJoinCollisionMeshes(false);
      } else {
        objectTemplate->setJoinCollisionMeshes(true);
      }
      objectAttributesManager->registerObject(objectTemplate);
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
        Magnum::Vector3 initialPosition{0.0, boxHeight + 1.25f, 0.0};
        physicsManager_->setRotation(
            objectId, Magnum::Quaternion::fromMatrix(R.rotationNormalized()));
        physicsManager_->setTranslation(objectId, initialPosition);

        ASSERT_EQ(node.absoluteTranslation(), initialPosition);
      }

      float timeToSim = 20.0;
      while (physicsManager_->getWorldTime() < timeToSim) {
        physicsManager_->stepPhysics(0.1);
      }
      int numActiveObjects = physicsManager_->checkActiveObjects();
      LOG(INFO) << " Number of active objects: " << numActiveObjects;

      if (i == 1) {
        // when collision meshes are joined, objects should be stable
        ASSERT_EQ(numActiveObjects, 0);
        ASSERT_EQ(physicsManager_->getNumActiveContactPoints(), 0);
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

  std::string stageFile =
      Cr::Utility::Directory::join(dataDir, "test_assets/scenes/plane.glb");
  std::string objectFile =
      Cr::Utility::Directory::join(dataDir, "test_assets/objects/sphere.glb");

  initStage(stageFile);

  if (physicsManager_->getPhysicsSimulationLibrary() !=
      PhysicsManager::PhysicsSimulationLibrary::NONE) {
    // if we have a simulation implementation then test bounding box vs mesh for
    // sphere object

    ObjectAttributes::ptr ObjectAttributes = ObjectAttributes::create();
    ObjectAttributes->setRenderAssetHandle(objectFile);
    ObjectAttributes->setMargin(0.0);
    ObjectAttributes->setJoinCollisionMeshes(false);

    auto objectAttributesManager =
        metadataMediator_->getObjectAttributesManager();
    objectAttributesManager->registerObject(ObjectAttributes, objectFile);

    // get a reference to the stored template to edit
    ObjectAttributes::ptr objectTemplate =
        objectAttributesManager->getObjectCopyByHandle(objectFile);

    for (int i = 0; i < 2; i++) {
      if (i == 0) {
        objectTemplate->setBoundingBoxCollisions(false);
      } else {
        objectTemplate->setBoundingBoxCollisions(true);
      }
      objectAttributesManager->registerObject(objectTemplate);
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
        Magnum::Vector3 force{2.0, 0.0, 0.0};
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
  LOG(INFO) << "Starting physics test: DiscreteContactTest";

  std::string stageFile =
      Cr::Utility::Directory::join(dataDir, "test_assets/scenes/plane.glb");
  std::string objectFile = Cr::Utility::Directory::join(
      dataDir, "test_assets/objects/transform_box.glb");

  initStage(stageFile);

  if (physicsManager_->getPhysicsSimulationLibrary() !=
      PhysicsManager::PhysicsSimulationLibrary::NONE) {
    ObjectAttributes::ptr ObjectAttributes = ObjectAttributes::create();
    ObjectAttributes->setRenderAssetHandle(objectFile);
    ObjectAttributes->setMargin(0.0);
    auto objectAttributesManager =
        metadataMediator_->getObjectAttributesManager();
    objectAttributesManager->registerObject(ObjectAttributes, objectFile);

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

    // set stage to non-collidable
    ASSERT_TRUE(physicsManager_->getStageIsCollidable());
    physicsManager_->setStageIsCollidable(false);
    ASSERT_FALSE(physicsManager_->getStageIsCollidable());
    ASSERT_FALSE(physicsManager_->contactTest(objectId0));

    // move box 0 into box 1
    physicsManager_->setTranslation(objectId0, Magnum::Vector3{1.1, 1.1, 0});
    ASSERT_TRUE(physicsManager_->contactTest(objectId0));
    ASSERT_TRUE(physicsManager_->contactTest(objectId1));

    // set box 0 to non-collidable
    ASSERT_TRUE(physicsManager_->getObjectIsCollidable(objectId0));
    physicsManager_->setObjectIsCollidable(objectId0, false);
    ASSERT_FALSE(physicsManager_->getObjectIsCollidable(objectId0));
    ASSERT_FALSE(physicsManager_->contactTest(objectId0));
    ASSERT_FALSE(physicsManager_->contactTest(objectId1));
  }
}

TEST_F(PhysicsManagerTest, BulletCompoundShapeMargins) {
  // test that all different construction methods for a simple shape result in
  // the same Aabb for the given margin
  LOG(INFO) << "Starting physics test: BulletCompoundShapeMargins";

  std::string objectFile = Cr::Utility::Directory::join(
      dataDir, "test_assets/objects/transform_box.glb");

  initStage(objectFile);

  if (physicsManager_->getPhysicsSimulationLibrary() ==
      PhysicsManager::PhysicsSimulationLibrary::BULLET) {
    // test joined vs. unjoined
    ObjectAttributes::ptr ObjectAttributes = ObjectAttributes::create();
    ObjectAttributes->setRenderAssetHandle(objectFile);
    ObjectAttributes->setMargin(0.1);

    auto objectAttributesManager =
        metadataMediator_->getObjectAttributesManager();
    objectAttributesManager->registerObject(ObjectAttributes, objectFile);

    // get a reference to the stored template to edit
    ObjectAttributes::ptr objectTemplate =
        objectAttributesManager->getObjectCopyByHandle(objectFile);

    auto* drawables = &sceneManager_.getSceneGraph(sceneID_).getDrawables();

    // add the unjoined object
    objectTemplate->setJoinCollisionMeshes(false);
    objectAttributesManager->registerObject(objectTemplate);
    int objectId0 = physicsManager_->addObject(objectFile, drawables);

    // add the joined object
    objectTemplate->setJoinCollisionMeshes(true);
    objectAttributesManager->registerObject(objectTemplate);
    int objectId1 = physicsManager_->addObject(objectFile, drawables);

    // add bounding box object
    objectTemplate->setBoundingBoxCollisions(true);
    objectAttributesManager->registerObject(objectTemplate);
    int objectId2 = physicsManager_->addObject(objectFile, drawables);

    esp::physics::BulletPhysicsManager* bPhysManager =
        static_cast<esp::physics::BulletPhysicsManager*>(physicsManager_.get());

    const Magnum::Range3D AabbStage =
        bPhysManager->getStageCollisionShapeAabb();

    const Magnum::Range3D AabbOb0 =
        bPhysManager->getCollisionShapeAabb(objectId0);
    const Magnum::Range3D AabbOb1 =
        bPhysManager->getCollisionShapeAabb(objectId1);
    const Magnum::Range3D AabbOb2 =
        bPhysManager->getCollisionShapeAabb(objectId2);

    Magnum::Range3D objectGroundTruth({-1.1, -1.1, -1.1}, {1.1, 1.1, 1.1});
    Magnum::Range3D stageGroundTruth({-1.04, -1.04, -1.04}, {1.04, 1.04, 1.04});

    ASSERT_EQ(AabbStage, stageGroundTruth);
    ASSERT_EQ(AabbOb0, objectGroundTruth);
    ASSERT_EQ(AabbOb1, objectGroundTruth);
    ASSERT_EQ(AabbOb2, objectGroundTruth);
  }
}
#endif

TEST_F(PhysicsManagerTest, ConfigurableScaling) {
  // test scaling of objects via template configuration (visual and collision)
  LOG(INFO) << "Starting physics test: ConfigurableScaling";

  std::string stageFile =
      Cr::Utility::Directory::join(dataDir, "test_assets/scenes/plane.glb");

  std::string objectFile = Cr::Utility::Directory::join(
      dataDir, "test_assets/objects/transform_box.glb");

  initStage(stageFile);

  // test joined vs. unjoined
  ObjectAttributes::ptr ObjectAttributes = ObjectAttributes::create();
  ObjectAttributes->setRenderAssetHandle(objectFile);
  ObjectAttributes->setMargin(0.0);

  auto objectAttributesManager =
      metadataMediator_->getObjectAttributesManager();
  objectAttributesManager->registerObject(ObjectAttributes, objectFile);

  // get a reference to the stored template to edit
  ObjectAttributes::ptr objectTemplate =
      objectAttributesManager->getObjectCopyByHandle(objectFile);

  std::vector<Magnum::Vector3> testScales{
      {1.0, 1.0, 1.0},  {4.0, 3.0, 2.0},    {0.1, 0.2, 0.3},
      {0.0, 0.0, 0.0},  {-1.0, -1.0, -1.0}, {-1.0, 1.0, 1.0},
      {4.0, -3.0, 2.0}, {0.1, -0.2, -0.3}};

  auto& drawables = sceneManager_.getSceneGraph(sceneID_).getDrawables();

  std::vector<int> objectIDs;
  for (auto& testScale : testScales) {
    objectTemplate->setScale(testScale);
    objectAttributesManager->registerObject(objectTemplate);

    Magnum::Range3D boundsGroundTruth(-abs(testScale), abs(testScale));

    int objectId = physicsManager_->addObject(objectFile, &drawables);
    objectIDs.push_back(objectId);

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

  // check that scales are stored and queried correctly
  for (size_t ix = 0; ix < objectIDs.size(); ix++) {
    ASSERT_EQ(physicsManager_->getScale(objectIDs[ix]), testScales[ix]);
  }
}

TEST_F(PhysicsManagerTest, TestVelocityControl) {
  // test scaling of objects via template configuration (visual and collision)
  LOG(INFO) << "Starting physics test: TestVelocityControl";

  std::string objectFile = Cr::Utility::Directory::join(
      dataDir, "test_assets/objects/transform_box.glb");

  std::string stageFile =
      Cr::Utility::Directory::join(dataDir, "test_assets/scenes/plane.glb");

  initStage(stageFile);

  ObjectAttributes::ptr ObjectAttributes = ObjectAttributes::create();
  ObjectAttributes->setRenderAssetHandle(objectFile);
  ObjectAttributes->setMargin(0.0);
  auto objectAttributesManager =
      metadataMediator_->getObjectAttributesManager();
  objectAttributesManager->registerObject(ObjectAttributes, objectFile);

  auto& drawables = sceneManager_.getSceneGraph(sceneID_).getDrawables();

  int objectId = physicsManager_->addObject(objectFile, &drawables);
  physicsManager_->setTranslation(objectId, Magnum::Vector3{0, 1.0, 0});

  Magnum::Vector3 commandLinVel(1.0, 1.0, 1.0);
  Magnum::Vector3 commandAngVel(1.0, 1.0, 1.0);

  // test results of getting/setting
  if (physicsManager_->getPhysicsSimulationLibrary() ==
      PhysicsManager::PhysicsSimulationLibrary::BULLET) {
    physicsManager_->setLinearVelocity(objectId, commandLinVel);
    physicsManager_->setAngularVelocity(objectId, commandAngVel);

    ASSERT_EQ(physicsManager_->getLinearVelocity(objectId), commandLinVel);
    ASSERT_EQ(physicsManager_->getAngularVelocity(objectId), commandAngVel);

  } else if (physicsManager_->getPhysicsSimulationLibrary() ==
             PhysicsManager::PhysicsSimulationLibrary::NONE) {
    physicsManager_->setLinearVelocity(objectId, commandLinVel);
    physicsManager_->setAngularVelocity(objectId, commandAngVel);

    // default kinematics always 0 velocity when queried
    ASSERT_EQ(physicsManager_->getLinearVelocity(objectId), Magnum::Vector3{});
    ASSERT_EQ(physicsManager_->getAngularVelocity(objectId), Magnum::Vector3{});
  }

  // test constant velocity control mechanism
  esp::physics::VelocityControl::ptr velControl =
      physicsManager_->getVelocityControl(objectId);
  velControl->controllingAngVel = true;
  velControl->controllingLinVel = true;
  velControl->linVel = Magnum::Vector3{1.0, -1.0, 1.0};
  velControl->angVel = Magnum::Vector3{1.0, 0, 0};

  // first kinematic
  physicsManager_->setObjectMotionType(objectId,
                                       esp::physics::MotionType::KINEMATIC);
  physicsManager_->setTranslation(objectId, Magnum::Vector3{0, 2.0, 0});

  float targetTime = 2.0;
  while (physicsManager_->getWorldTime() < targetTime) {
    physicsManager_->stepPhysics(targetTime - physicsManager_->getWorldTime());
  }
  Magnum::Vector3 posGroundTruth{2.0, 0.0, 2.0};
  Magnum::Quaternion qGroundTruth{{0.842602, 0, 0}, 0.538537};

  float errorEps = 0.015;  // fairly loose due to discrete timestep
  ASSERT_LE(
      (physicsManager_->getTranslation(objectId) - posGroundTruth).length(),
      errorEps);
  Magnum::Rad angleError =
      Magnum::Math::angle(physicsManager_->getRotation(objectId), qGroundTruth);

  ASSERT_LE(float(angleError), errorEps);

  if (physicsManager_->getPhysicsSimulationLibrary() ==
      PhysicsManager::PhysicsSimulationLibrary::BULLET) {
    physicsManager_->setObjectMotionType(objectId,
                                         esp::physics::MotionType::DYNAMIC);
    physicsManager_->resetTransformation(objectId);
    physicsManager_->setTranslation(objectId, Magnum::Vector3{0, 2.0, 0});
    physicsManager_->setGravity({});  // 0 gravity interference
    physicsManager_->reset();         // reset time to 0

    // should closely follow kinematic result while uninhibited in 0 gravity
    float targetTime = 0.5;
    esp::core::RigidState initialObjectState(
        physicsManager_->getRotation(objectId),
        physicsManager_->getTranslation(objectId));
    esp::core::RigidState kinematicResult =
        velControl->integrateTransform(targetTime, initialObjectState);
    while (physicsManager_->getWorldTime() < targetTime) {
      physicsManager_->stepPhysics(physicsManager_->getTimestep());
    }
    ASSERT_LE((physicsManager_->getTranslation(objectId) -
               kinematicResult.translation)
                  .length(),
              errorEps);
    angleError = Magnum::Math::angle(physicsManager_->getRotation(objectId),
                                     kinematicResult.rotation);
    ASSERT_LE(float(angleError), errorEps);

    // should then get blocked by ground plane collision
    targetTime = 2.0;
    while (physicsManager_->getWorldTime() < targetTime) {
      physicsManager_->stepPhysics(physicsManager_->getTimestep());
    }
    ASSERT_GE(physicsManager_->getTranslation(objectId)[1], 1.0 - errorEps);
  }

  // test local velocity
  physicsManager_->setObjectMotionType(objectId,
                                       esp::physics::MotionType::KINEMATIC);
  physicsManager_->resetTransformation(objectId);
  physicsManager_->setTranslation(objectId, Magnum::Vector3{0, 2.0, 0});

  velControl->linVel = Magnum::Vector3{0.0, 0.0, -1.0};
  velControl->angVel = Magnum::Vector3{1.0, 0, 0};
  velControl->angVelIsLocal = true;
  velControl->linVelIsLocal = true;

  targetTime = 10.0;
  physicsManager_->reset();  // reset time to 0
  while (physicsManager_->getWorldTime() < targetTime) {
    physicsManager_->stepPhysics(physicsManager_->getTimestep());
  }

  Magnum::Vector3 posLocalGroundTruth{0, 3.83589, 0.543553};
  Magnum::Quaternion qLocalGroundTruth{{-0.95782, 0, 0}, 0.287495};
  qLocalGroundTruth = qLocalGroundTruth.normalized();

  // test zero velocity kinematic integration (should not change state)
  velControl->linVel = Magnum::Vector3{0.0, 0.0, 0.0};
  velControl->angVel = Magnum::Vector3{0.0, 0.0, 0.0};
  physicsManager_->stepPhysics(physicsManager_->getTimestep());

  ASSERT_LE((physicsManager_->getTranslation(objectId) - posLocalGroundTruth)
                .length(),
            errorEps);
  Magnum::Rad angleErrorLocal = Magnum::Math::angle(
      physicsManager_->getRotation(objectId), qLocalGroundTruth);

  ASSERT_LE(float(angleErrorLocal), errorEps);
}

TEST_F(PhysicsManagerTest, TestSceneNodeAttachment) {
  // test attaching/detaching existing SceneNode to/from physical simulation
  LOG(INFO) << "Starting physics test: TestSceneNodeAttachment";

  std::string objectFile = Cr::Utility::Directory::join(
      dataDir, "test_assets/objects/transform_box.glb");

  std::string stageFile =
      Cr::Utility::Directory::join(dataDir, "test_assets/scenes/plane.glb");

  initStage(stageFile);

  ObjectAttributes::ptr ObjectAttributes = ObjectAttributes::create();
  ObjectAttributes->setRenderAssetHandle(objectFile);
  auto objectAttributesManager =
      metadataMediator_->getObjectAttributesManager();
  objectAttributesManager->registerObject(ObjectAttributes, objectFile);

  esp::scene::SceneNode& root =
      sceneManager_.getSceneGraph(sceneID_).getRootNode();
  esp::scene::SceneNode* newNode = &root.createChild();
  ASSERT_EQ(root.children().last(), newNode);

  auto& drawables = sceneManager_.getSceneGraph(sceneID_).getDrawables();

  // Test attaching newNode to a RigidBody
  int objectId = physicsManager_->addObject(objectFile, &drawables, newNode);
  ASSERT_EQ(&physicsManager_->getObjectSceneNode(objectId), newNode);

  // Test updating newNode position with PhysicsManager
  Magnum::Vector3 newPos{1.0, 3.0, 0.0};
  physicsManager_->setTranslation(objectId, newPos);
  ASSERT_EQ(physicsManager_->getTranslation(objectId), newPos);
  ASSERT_EQ(newNode->translation(), newPos);

  // Test leaving newNode without visualNode_ after destroying the RigidBody
  physicsManager_->removeObject(objectId, false, true);
  ASSERT(newNode->children().isEmpty());

  // Test leaving the visualNode attached to newNode after destroying the
  // RigidBody
  objectId = physicsManager_->addObject(objectFile, &drawables, newNode);
  physicsManager_->removeObject(objectId, false, false);
  ASSERT(!newNode->children().isEmpty());

  // Test destroying newNode with the RigidBody
  objectId = physicsManager_->addObject(objectFile, &drawables, newNode);
  physicsManager_->removeObject(objectId, true, true);
  ASSERT_NE(root.children().last(), newNode);
}

TEST_F(PhysicsManagerTest, TestMotionTypes) {
  // test setting motion types and expected simulation behaviors
  LOG(INFO) << "Starting physics test: TestMotionTypes";

  std::string objectFile = Cr::Utility::Directory::join(
      dataDir, "test_assets/objects/transform_box.glb");

  std::string stageFile =
      Cr::Utility::Directory::join(dataDir, "test_assets/scenes/plane.glb");

  initStage(stageFile);

  // ensure that changing default timestep does not affect results
  physicsManager_->setTimestep(0.0041666666);

  // We need dynamics to test this.
  if (physicsManager_->getPhysicsSimulationLibrary() !=
      PhysicsManager::PhysicsSimulationLibrary::NONE) {
    float boxHalfExtent = 0.2;

    ObjectAttributes::ptr ObjectAttributes = ObjectAttributes::create();
    ObjectAttributes->setRenderAssetHandle(objectFile);
    ObjectAttributes->setBoundingBoxCollisions(true);
    ObjectAttributes->setScale({boxHalfExtent, boxHalfExtent, boxHalfExtent});
    auto objectAttributesManager =
        metadataMediator_->getObjectAttributesManager();

    int boxId =
        objectAttributesManager->registerObject(ObjectAttributes, objectFile);

    auto& drawables = sceneManager_.getSceneGraph(sceneID_).getDrawables();

    std::vector<int> instancedObjects;
    float stageCollisionMargin = 0.04;

    for (int testId = 0; testId < 3; testId++) {
      instancedObjects.push_back(physicsManager_->addObject(boxId, &drawables));
      instancedObjects.push_back(physicsManager_->addObject(boxId, &drawables));

      switch (testId) {
        case 0: {
          // test 0: stacking two DYNAMIC objects
          physicsManager_->setTranslation(
              instancedObjects[0],
              {0, stageCollisionMargin + boxHalfExtent, 0});
          physicsManager_->setTranslation(
              instancedObjects[1],
              {0, stageCollisionMargin + boxHalfExtent * 3, 0});

          while (physicsManager_->getWorldTime() < 6.0) {
            physicsManager_->stepPhysics(0.1);
          }
          ASSERT_FALSE(physicsManager_->isObjectAwake(instancedObjects[0]));
          ASSERT_FALSE(physicsManager_->isObjectAwake(instancedObjects[1]));
          ASSERT_LE(
              (physicsManager_->getTranslation(instancedObjects[0]) -
               Magnum::Vector3{0.0, stageCollisionMargin + boxHalfExtent, 0.0})
                  .length(),
              1.0e-3);
          ASSERT_LE((physicsManager_->getTranslation(instancedObjects[1]) -
                     Magnum::Vector3{
                         0.0, stageCollisionMargin + boxHalfExtent * 3, 0.0})
                        .length(),
                    1.0e-3);
        } break;
        case 1: {
          // test 1: stacking a DYNAMIC object on a STATIC object
          physicsManager_->setTranslation(instancedObjects[0],
                                          {0, boxHalfExtent * 2, 0});
          physicsManager_->setObjectMotionType(
              instancedObjects[0], esp::physics::MotionType::STATIC);
          physicsManager_->setTranslation(instancedObjects[1],
                                          {0, boxHalfExtent * 5, 0});

          while (physicsManager_->getWorldTime() < 6.0) {
            physicsManager_->stepPhysics(0.1);
          }
          ASSERT_FALSE(physicsManager_->isObjectAwake(instancedObjects[1]));
          ASSERT_LE((physicsManager_->getTranslation(instancedObjects[0]) -
                     Magnum::Vector3{0.0, boxHalfExtent * 2, 0.0})
                        .length(),
                    1.0e-4);
          ASSERT_LE((physicsManager_->getTranslation(instancedObjects[1]) -
                     Magnum::Vector3{0.0, boxHalfExtent * 4, 0.0})
                        .length(),
                    2.0e-4);
        } break;
        case 2: {
          // test 2: stacking a DYNAMIC object on a moving KINEMATIC object
          physicsManager_->setTranslation(instancedObjects[0],
                                          {0, boxHalfExtent * 2, 0});
          physicsManager_->setObjectMotionType(
              instancedObjects[0], esp::physics::MotionType::KINEMATIC);

          esp::physics::VelocityControl::ptr velCon =
              physicsManager_->getVelocityControl(instancedObjects[0]);
          velCon->controllingLinVel = true;
          velCon->linVel = {0.2, 0, 0};

          physicsManager_->setTranslation(instancedObjects[1],
                                          {0, boxHalfExtent * 5, 0});

          while (physicsManager_->getWorldTime() < 3.0) {
            physicsManager_->stepPhysics(0.1);
          }
          ASSERT_LE((physicsManager_->getTranslation(instancedObjects[0]) -
                     Magnum::Vector3{0.62, boxHalfExtent * 2, 0.0})
                        .length(),
                    1.0e-4);
          ASSERT_LE((physicsManager_->getTranslation(instancedObjects[1]) -
                     Magnum::Vector3{0.578, boxHalfExtent * 4, 0.0})
                        .length(),
                    2.0e-2);
        } break;
      }

      // reset the scene
      for (auto id : instancedObjects) {
        physicsManager_->removeObject(id);
      }
      instancedObjects.clear();
      physicsManager_->reset();  // time=0
    }
  }
}

TEST_F(PhysicsManagerTest, TestURDF) {
  // test loading URDF and simulating an ArticulatedObject
  LOG(INFO) << "Starting physics test: TestURDF";

  std::string robotFile = Cr::Utility::Directory::join(
      TEST_ASSETS, "URDF/kuka_iiwa/model_free_base.urdf");

  std::string stageFile = Cr::Utility::Directory::join(
      dataDir, "test_assets/scenes/simple_room.glb");

  initStage(stageFile);

  // need a library to try loading a URDF
  if (physicsManager_->getPhysicsSimulationLibrary() !=
      PhysicsManager::PhysicsSimulationLibrary::NONE) {
    int robotId = physicsManager_->addArticulatedObjectFromURDF(
        robotFile, &sceneManager_.getSceneGraph(sceneID_).getDrawables());

    ASSERT_NE(robotId, esp::ID_UNDEFINED);

    physicsManager_->stepPhysics(1.0 / 60.0);
  }
}

TEST_F(PhysicsManagerTest, TestNumActiveContactPoints) {
  std::string stageFile = Cr::Utility::Directory::join(
      dataDir, "test_assets/scenes/simple_room.glb");

  initStage(stageFile);
  auto& drawables = sceneManager_.getSceneGraph(sceneID_).getDrawables();

  // We need dynamics to test this.
  if (physicsManager_->getPhysicsSimulationLibrary() !=
      PhysicsManager::PhysicsSimulationLibrary::NONE) {
    auto objectAttributesManager =
        metadataMediator_->getObjectAttributesManager();

    std::string cubeHandle =
        objectAttributesManager->getObjectHandlesBySubstring("cubeSolid")[0];

    // add a single cube
    Mn::Vector3 stackBase(0.21964, 1.29183, -0.0897472);
    std::vector<int> cubeIds;
    cubeIds.push_back(physicsManager_->addObject(cubeHandle, &drawables));
    physicsManager_->setTranslation(cubeIds.back(), stackBase);

    // no active contact points at start
    ASSERT_EQ(physicsManager_->getNumActiveContactPoints(), 0);

    // simulate to let cube fall, stabilize and go to sleep
    bool didHaveActiveContacts = false;
    while (physicsManager_->getWorldTime() < 4.0) {
      physicsManager_->stepPhysics(0.1);
      if (physicsManager_->getNumActiveContactPoints() > 0) {
        didHaveActiveContacts = true;
      }
    }
    ASSERT(didHaveActiveContacts);

    // no active contact points at end
    ASSERT_EQ(physicsManager_->getNumActiveContactPoints(), 0);
  }
}

TEST_F(PhysicsManagerTest, TestRemoveSleepingSupport) {
  // test that removing a sleeping support object wakes its collision island
  LOG(INFO) << "Starting physics test: TestRemoveSleepingSupport";

  std::string stageFile = "NONE";

  initStage(stageFile);
  auto& drawables = sceneManager_.getSceneGraph(sceneID_).getDrawables();

  // We need dynamics to test this.
  if (physicsManager_->getPhysicsSimulationLibrary() !=
      PhysicsManager::PhysicsSimulationLibrary::NONE) {
    auto objectAttributesManager =
        metadataMediator_->getObjectAttributesManager();

    std::string cubeHandle =
        objectAttributesManager->getObjectHandlesBySubstring("cubeSolid")[0];

    // create a stack of cubes in free space
    Mn::Vector3 stackBase(0.21964, 1.29183, -0.0897472);
    std::vector<int> cubeIds;
    int stackSize = 4;
    for (int i = 0; i < stackSize; ++i) {
      cubeIds.push_back(physicsManager_->addObject(cubeHandle, &drawables));
      physicsManager_->setTranslation(cubeIds.back(),
                                      (Mn::Vector3(0, 0.2, 0) * i) + stackBase);
    }

    physicsManager_->setObjectMotionType(cubeIds.front(),
                                         esp::physics::MotionType::STATIC);

    for (int testCase = 0; testCase < 2; ++testCase) {
      // reset time to 0, should not otherwise modify state
      physicsManager_->reset();
      ASSERT(physicsManager_->getNumRigidObjects() > 0);

      // simulate to stabilize the stack and populate collision islands
      while (physicsManager_->getWorldTime() < 4.0) {
        physicsManager_->stepPhysics(0.1);
      }

      // cubes should be sleeping
      for (auto id : cubeIds) {
        ASSERT(!physicsManager_->isObjectAwake(id));
      }

      // no active contact points
      ASSERT_EQ(physicsManager_->getNumActiveContactPoints(), 0);

      if (testCase == 0) {
        // first remove the bottom-most DYNAMIC object, expecting those above to
        // fall
        physicsManager_->removeObject(cubeIds[1]);
        cubeIds.erase(cubeIds.begin() + 1);
      } else if (testCase == 1) {
        // second remove the STATIC bottom cube
        physicsManager_->removeObject(cubeIds.front());
        cubeIds.erase(cubeIds.begin());
      }

      // remaining cubes should now be awake
      for (auto id : cubeIds) {
        if (physicsManager_->getObjectMotionType(id) !=
            esp::physics::MotionType::STATIC) {
          ASSERT(physicsManager_->isObjectAwake(id));
        }
      }

      ASSERT_GT(physicsManager_->getNumActiveContactPoints(), 0);
    }
  }
}
