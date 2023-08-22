// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/TestSuite/Compare/Numeric.h>
#include <Corrade/TestSuite/Tester.h>
#include <Corrade/Utility/Path.h>
#include <string>

#include "esp/sim/Simulator.h"

#include "esp/assets/ResourceManager.h"
#include "esp/metadata/MetadataMediator.h"
#include "esp/scene/SceneManager.h"

#include "esp/physics/PhysicsManager.h"
#include "esp/physics/objectManagers/RigidObjectManager.h"
#ifdef ESP_BUILD_WITH_BULLET
#include "esp/physics/bullet/BulletPhysicsManager.h"
#include "esp/physics/bullet/objectWrappers/ManagedBulletRigidObject.h"
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

const std::string dataDir = Cr::Utility::Path::join(SCENE_DATASETS, "../");
const std::string physicsConfigFile =
    Cr::Utility::Path::join(SCENE_DATASETS, "../default.physics_config.json");

namespace {
struct PhysicsTest : Cr::TestSuite::Tester {
  explicit PhysicsTest();

  void close() {
    physicsManager_ = nullptr;

    sceneManager_ = nullptr;

    resourceManager_ = nullptr;

    context_ = nullptr;
  }

  void resetCreateRendererFlag(bool createRenderer) {
    close();
    auto cfg = esp::sim::SimulatorConfiguration{};
    cfg.createRenderer = createRenderer;
    // setting values for stage load
    cfg.loadSemanticMesh = false;
    cfg.forceSeparateSemanticSceneGraph = false;
    cfg.enablePhysics = true;
    metadataMediator_ = MetadataMediator::create(cfg);
    // get attributes manager for physics world attributes
    physicsAttributesManager_ =
        metadataMediator_->getPhysicsAttributesManager();

    sceneManager_ = SceneManager::create_unique();
    resourceManager_ = std::make_unique<ResourceManager>(metadataMediator_);
    if (createRenderer) {
      context_ = esp::gfx::WindowlessContext::create_unique(0);
    }
    resourceManager_->setRequiresTextures(createRenderer);

    sceneID_ = sceneManager_->initSceneGraph();
  }

  void initStage(const std::string& stageFile) {
    auto& sceneGraph = sceneManager_->getSceneGraph(sceneID_);
    auto& rootNode = sceneGraph.getRootNode();

    // construct appropriate physics attributes based on config file
    auto physicsManagerAttributes =
        physicsAttributesManager_->createObject(physicsConfigFile, true);
    auto stageAttributesMgr = metadataMediator_->getStageAttributesManager();
    if (physicsManagerAttributes != nullptr) {
      stageAttributesMgr->setCurrPhysicsManagerAttributesHandle(
          physicsManagerAttributes->getHandle());
    }
    // create scene instance attributes
    esp::metadata::attributes::SceneInstanceAttributes::cptr
        curSceneInstanceAttributes =
            metadataMediator_->getSceneInstanceAttributesByName(stageFile);

    const esp::metadata::attributes::SceneObjectInstanceAttributes::cptr
        stageInstanceAttributes =
            curSceneInstanceAttributes->getStageInstance();

    // Get full library name of StageAttributes
    const std::string stageAttributesHandle =
        metadataMediator_->getStageAttrFullHandle(
            stageInstanceAttributes->getHandle());
    // Get StageAttributes copy
    auto stageAttributes =
        metadataMediator_->getStageAttributesManager()->getObjectCopyByHandle(
            stageAttributesHandle);

    // construct physics manager based on specifications in attributes
    resourceManager_->initPhysicsManager(physicsManager_, &rootNode,
                                         physicsManagerAttributes);

    // load scene
    std::vector<int> tempIDs{sceneID_, esp::ID_UNDEFINED};
    auto stageInstanceAttrs = resourceManager_->loadStage(
        stageAttributes, stageInstanceAttributes, physicsManager_,
        sceneManager_.get(), tempIDs);

    rigidObjectManager_ = physicsManager_->getRigidObjectManager();
  }

  auto makeObjectGetWrapper(const std::string& objectFile,
                            esp::gfx::DrawableGroup* drawables = nullptr,
                            esp::scene::SceneNode* attachmentNode = nullptr) {
    if (drawables == nullptr) {
      drawables = &sceneManager_->getSceneGraph(sceneID_).getDrawables();
    }
    auto objAttr =
        metadataMediator_->getObjectAttributesManager()->getObjectCopyByHandle(
            objectFile);

    int objectId =
        physicsManager_->addObject(objAttr, drawables, attachmentNode);
#ifdef ESP_BUILD_WITH_BULLET
    esp::physics::ManagedBulletRigidObject::ptr objectWrapper =
        rigidObjectManager_
            ->getObjectCopyByID<esp::physics::ManagedBulletRigidObject>(
                objectId);
#else
    esp::physics::ManagedRigidObject::ptr objectWrapper =
        rigidObjectManager_->getObjectCopyByID(objectId);
#endif
    return objectWrapper;
  }

  // tests
  void testJoinCompound();
  void testCollisionBoundingBox();
  void testDiscreteContactTest();
  void testBulletCompoundShapeMargins();
  void testConfigurableScaling();
  void testVelocityControl();
  void testSceneNodeAttachment();
  void testMotionTypes();
  void testNumActiveContactPoints();
  void testRemoveSleepingSupport();
  /////

  esp::logging::LoggingContext loggingContext_;
  // must declare these in this order due to avoid deallocation errors
  esp::gfx::WindowlessContext::uptr context_;

  std::shared_ptr<MetadataMediator> metadataMediator_ = nullptr;
  std::unique_ptr<ResourceManager> resourceManager_ = nullptr;

  AttrMgrs::PhysicsAttributesManager::ptr physicsAttributesManager_;
  SceneManager::uptr sceneManager_ = nullptr;
  PhysicsManager::ptr physicsManager_;

  std::shared_ptr<esp::physics::RigidObjectManager> rigidObjectManager_;

  int sceneID_;
};  // struct PhysicsTest
const struct {
  const char* name;
  bool enabled;
} RendererEnabledData[]{{"", true}, {"renderer disabled", false}};

PhysicsTest::PhysicsTest() {
  addInstancedTests(
      {
#ifdef ESP_BUILD_WITH_BULLET
          &PhysicsTest::testJoinCompound,
          &PhysicsTest::testCollisionBoundingBox,
          &PhysicsTest::testDiscreteContactTest,
          &PhysicsTest::testBulletCompoundShapeMargins,
          &PhysicsTest::testMotionTypes,
          &PhysicsTest::testRemoveSleepingSupport,
          &PhysicsTest::testNumActiveContactPoints,
#endif
          &PhysicsTest::testConfigurableScaling,
          &PhysicsTest::testVelocityControl,
          &PhysicsTest::testSceneNodeAttachment},
      Cr::Containers::arraySize(RendererEnabledData));
}

#ifdef ESP_BUILD_WITH_BULLET
void PhysicsTest::testJoinCompound() {
  std::string stageFile =
      Cr::Utility::Path::join(dataDir, "test_assets/scenes/simple_room.glb");
  std::string objectFile =
      Cr::Utility::Path::join(dataDir, "test_assets/objects/nested_box.glb");

  resetCreateRendererFlag(RendererEnabledData[testCaseInstanceId()].enabled);

  initStage(stageFile);

  if (physicsManager_->getPhysicsSimulationLibrary() !=
      PhysicsManager::PhysicsSimulationLibrary::NoPhysics) {
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

    for (int i = 0; i < 2; ++i) {
      // mark the object not joined
      if (i == 0) {
        objectTemplate->setJoinCollisionMeshes(false);
      } else {
        objectTemplate->setJoinCollisionMeshes(true);
      }
      objectAttributesManager->registerObject(objectTemplate);
      physicsManager_->reset();

      // add and simulate objects
      int num_objects = 7;
      for (int o = 0; o < num_objects; ++o) {
        auto objWrapper = rigidObjectManager_->addObjectByHandle(objectFile);

        esp::scene::SceneNode* node = objWrapper->getSceneNode();

        Magnum::Matrix4 R{
            Magnum::Matrix4::rotationX(Magnum::Math::Rad<float>(-1.56)) *
            Magnum::Matrix4::rotationY(Magnum::Math::Rad<float>(-0.25))};
        float boxHeight = 2.0 + (o * 2);
        Magnum::Vector3 initialPosition{0.0, boxHeight + 1.5f, 0.0};
        objWrapper->setRotation(
            Magnum::Quaternion::fromMatrix(R.rotationNormalized()));
        objWrapper->setTranslation(initialPosition);

        CORRADE_COMPARE(node->absoluteTranslation(), initialPosition);
      }

      float timeToSim = 20.0;
      while (physicsManager_->getWorldTime() < timeToSim) {
        physicsManager_->stepPhysics(0.1);
      }
      int numActiveObjects = physicsManager_->checkActiveObjects();
      ESP_DEBUG() << "Number of active objects:" << numActiveObjects
                  << "| Num Total Objects :"
                  << physicsManager_->getNumRigidObjects();

      if (i == 1) {
        // when collision meshes are joined, objects should be stable
        CORRADE_COMPARE(numActiveObjects, 0);
        CORRADE_COMPARE(physicsManager_->getNumActiveContactPoints(), 0);
      }

      rigidObjectManager_->removeAllObjects();
    }
  }
}  // PhysicsTest::testJoinCompound

void PhysicsTest::testCollisionBoundingBox() {
  std::string stageFile =
      Cr::Utility::Path::join(dataDir, "test_assets/scenes/plane.glb");
  std::string objectFile =
      Cr::Utility::Path::join(dataDir, "test_assets/objects/sphere.glb");

  resetCreateRendererFlag(RendererEnabledData[testCaseInstanceId()].enabled);

  initStage(stageFile);

  if (physicsManager_->getPhysicsSimulationLibrary() !=
      PhysicsManager::PhysicsSimulationLibrary::NoPhysics) {
    // if we have a simulation implementation then test bounding box vs mesh
    // for sphere object

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

    for (int i = 0; i < 2; ++i) {
      if (i == 0) {
        objectTemplate->setBoundingBoxCollisions(false);
      } else {
        objectTemplate->setBoundingBoxCollisions(true);
      }
      objectAttributesManager->registerObject(objectTemplate);
      physicsManager_->reset();

      auto objectWrapper = makeObjectGetWrapper(
          objectFile, &sceneManager_->getSceneGraph(sceneID_).getDrawables());
      CORRADE_VERIFY(objectWrapper);

      Magnum::Vector3 initialPosition{0.0, 0.25, 0.0};
      objectWrapper->setTranslation(initialPosition);

      Magnum::Quaternion prevOrientation = objectWrapper->getRotation();
      Magnum::Vector3 prevPosition = objectWrapper->getTranslation();
      float timeToSim = 3.0;
      while (physicsManager_->getWorldTime() < timeToSim) {
        Magnum::Vector3 force{2.0, 0.0, 0.0};
        objectWrapper->applyForce(force, Magnum::Vector3{});
        physicsManager_->stepPhysics(0.1);

        Magnum::Quaternion orientation = objectWrapper->getRotation();
        Magnum::Vector3 position = objectWrapper->getTranslation();

        // object is being pushed, so should be moving
        CORRADE_COMPARE_AS(position, prevPosition,
                           Cr::TestSuite::Compare::NotEqual);
        Magnum::Rad q_angle =
            Magnum::Math::angle(orientation, Magnum::Quaternion({0, 0, 0}, 1));
        if (i == 1) {
          // bounding box for collision, so the sphere should not be rolling
          CORRADE_COMPARE_AS(q_angle, Magnum::Rad{0.1},
                             Cr::TestSuite::Compare::LessOrEqual);
        } else {
          // no bounding box, so the sphere should be rolling
          CORRADE_COMPARE_AS(orientation, prevOrientation,
                             Cr::TestSuite::Compare::NotEqual);
        }

        prevOrientation = orientation;
        prevPosition = position;
      }

      rigidObjectManager_->removePhysObjectByID(objectWrapper->getID());
    }
  }
}  // PhysicsTest::testCollisionBoundingBox

void PhysicsTest::testDiscreteContactTest() {
  std::string stageFile =
      Cr::Utility::Path::join(dataDir, "test_assets/scenes/plane.glb");
  std::string objectFile =
      Cr::Utility::Path::join(dataDir, "test_assets/objects/transform_box.glb");

  resetCreateRendererFlag(RendererEnabledData[testCaseInstanceId()].enabled);
  initStage(stageFile);

  if (physicsManager_->getPhysicsSimulationLibrary() !=
      PhysicsManager::PhysicsSimulationLibrary::NoPhysics) {
    ObjectAttributes::ptr ObjectAttributes = ObjectAttributes::create();
    ObjectAttributes->setRenderAssetHandle(objectFile);
    ObjectAttributes->setMargin(0.0);
    auto objectAttributesManager =
        metadataMediator_->getObjectAttributesManager();
    objectAttributesManager->registerObject(ObjectAttributes, objectFile);

    // generate two centered boxes with dimension 2x2x2
    auto objWrapper0 = rigidObjectManager_->addObjectByHandle(objectFile);
    auto objWrapper1 = rigidObjectManager_->addObjectByHandle(objectFile);

    // place them in collision free location (0.1 about ground plane and 0.2
    // apart)
    objWrapper0->setTranslation(Magnum::Vector3{0, 1.1, 0});
    objWrapper1->setTranslation(Magnum::Vector3{2.2, 1.1, 0});
    CORRADE_VERIFY(!objWrapper0->contactTest());
    CORRADE_VERIFY(!objWrapper1->contactTest());

    // move box 0 into floor
    objWrapper0->setTranslation(Magnum::Vector3{0, 0.9, 0});
    CORRADE_VERIFY(objWrapper0->contactTest());
    CORRADE_VERIFY(!objWrapper1->contactTest());
    // set box 0 STATIC (STATIC vs STATIC stage)
    objWrapper0->setMotionType(esp::physics::MotionType::STATIC);
    CORRADE_VERIFY(!objWrapper0->contactTest());
    // set box 0 KINEMATIC (KINEMATIC vs STATIC stage)
    objWrapper0->setMotionType(esp::physics::MotionType::KINEMATIC);
    CORRADE_VERIFY(!objWrapper0->contactTest());
    // reset to DYNAMIC
    objWrapper0->setMotionType(esp::physics::MotionType::DYNAMIC);

    // set stage to non-collidable
    CORRADE_VERIFY(physicsManager_->getStageIsCollidable());
    physicsManager_->setStageIsCollidable(false);
    CORRADE_VERIFY(!physicsManager_->getStageIsCollidable());
    CORRADE_VERIFY(!objWrapper0->contactTest());

    // move box 0 into box 1
    objWrapper0->setTranslation(Magnum::Vector3{1.1, 1.1, 0});
    CORRADE_VERIFY(objWrapper0->contactTest());
    CORRADE_VERIFY(objWrapper1->contactTest());
    // set box 0 STATIC (STATIC vs DYNAMIC)
    objWrapper0->setMotionType(esp::physics::MotionType::STATIC);
    CORRADE_VERIFY(objWrapper0->contactTest());
    CORRADE_VERIFY(objWrapper1->contactTest());
    // set box 1 STATIC (STATIC vs STATIC)
    objWrapper1->setMotionType(esp::physics::MotionType::STATIC);
    CORRADE_VERIFY(!objWrapper0->contactTest());
    CORRADE_VERIFY(!objWrapper1->contactTest());
    // set box 0 KINEMATIC (KINEMATIC vs STATIC)
    objWrapper0->setMotionType(esp::physics::MotionType::KINEMATIC);
    CORRADE_VERIFY(!objWrapper0->contactTest());
    CORRADE_VERIFY(!objWrapper1->contactTest());
    // set box 1 KINEMATIC (KINEMATIC vs KINEMATIC)
    objWrapper1->setMotionType(esp::physics::MotionType::KINEMATIC);
    CORRADE_VERIFY(!objWrapper0->contactTest());
    CORRADE_VERIFY(!objWrapper1->contactTest());
    // reset box 0 DYNAMIC (DYNAMIC vs KINEMATIC)
    objWrapper0->setMotionType(esp::physics::MotionType::DYNAMIC);
    CORRADE_VERIFY(objWrapper0->contactTest());
    CORRADE_VERIFY(objWrapper1->contactTest());

    // set box 0 to non-collidable
    CORRADE_VERIFY(objWrapper0->getCollidable());
    objWrapper0->setCollidable(false);
    CORRADE_VERIFY(!objWrapper0->getCollidable());
    CORRADE_VERIFY(!objWrapper0->contactTest());
    CORRADE_VERIFY(!objWrapper1->contactTest());
  }
}  // PhysicsTest::testDiscreteContactTest

void PhysicsTest::testBulletCompoundShapeMargins() {
  // test that all different construction methods for a simple shape result in
  // the same Aabb for the given margin

  std::string objectFile =
      Cr::Utility::Path::join(dataDir, "test_assets/objects/transform_box.glb");

  resetCreateRendererFlag(RendererEnabledData[testCaseInstanceId()].enabled);
  initStage(objectFile);

  if (physicsManager_->getPhysicsSimulationLibrary() ==
      PhysicsManager::PhysicsSimulationLibrary::Bullet) {
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

    auto* drawables = &sceneManager_->getSceneGraph(sceneID_).getDrawables();

    // add the unjoined object
    objectTemplate->setJoinCollisionMeshes(false);
    objectAttributesManager->registerObject(objectTemplate);
    auto objectWrapper0 = makeObjectGetWrapper(objectFile, drawables);
    CORRADE_VERIFY(objectWrapper0);

    // add the joined object
    objectTemplate->setJoinCollisionMeshes(true);
    objectAttributesManager->registerObject(objectTemplate);

    auto objectWrapper1 = makeObjectGetWrapper(objectFile, drawables);
    CORRADE_VERIFY(objectWrapper1);

    // add bounding box object
    objectTemplate->setBoundingBoxCollisions(true);
    objectAttributesManager->registerObject(objectTemplate);
    auto objectWrapper2 = makeObjectGetWrapper(objectFile, drawables);
    CORRADE_VERIFY(objectWrapper2);

    esp::physics::BulletPhysicsManager* bPhysManager =
        static_cast<esp::physics::BulletPhysicsManager*>(physicsManager_.get());

    const Magnum::Range3D AabbStage =
        bPhysManager->getStageCollisionShapeAabb();

    const Magnum::Range3D AabbOb0 = objectWrapper0->getCollisionShapeAabb();
    const Magnum::Range3D AabbOb1 = objectWrapper1->getCollisionShapeAabb();
    const Magnum::Range3D AabbOb2 = objectWrapper2->getCollisionShapeAabb();

    Magnum::Range3D objectGroundTruth({-1.1, -1.1, -1.1}, {1.1, 1.1, 1.1});
    Magnum::Range3D stageGroundTruth({-1.04, -1.04, -1.04}, {1.04, 1.04, 1.04});

    CORRADE_COMPARE(AabbStage, stageGroundTruth);
    CORRADE_COMPARE(AabbOb0, objectGroundTruth);
    CORRADE_COMPARE(AabbOb1, objectGroundTruth);
    CORRADE_COMPARE(AabbOb2, objectGroundTruth);
  }
}  // PhysicsTest::testBulletCompoundShapeMargins

void PhysicsTest::testMotionTypes() {
  // test setting motion types and expected simulation behaviors

  resetCreateRendererFlag(RendererEnabledData[testCaseInstanceId()].enabled);

  std::string objectFile =
      Cr::Utility::Path::join(dataDir, "test_assets/objects/transform_box.glb");

  std::string stageFile =
      Cr::Utility::Path::join(dataDir, "test_assets/scenes/plane.glb");

  initStage(stageFile);

  // ensure that changing default timestep does not affect results
  physicsManager_->setTimestep(0.0041666666);

  // We need dynamics to test this.
  if (physicsManager_->getPhysicsSimulationLibrary() !=
      PhysicsManager::PhysicsSimulationLibrary::NoPhysics) {
    float boxHalfExtent = 0.2;

    ObjectAttributes::ptr ObjectAttributes = ObjectAttributes::create();
    ObjectAttributes->setRenderAssetHandle(objectFile);
    ObjectAttributes->setBoundingBoxCollisions(true);
    ObjectAttributes->setScale({boxHalfExtent, boxHalfExtent, boxHalfExtent});
    auto objectAttributesManager =
        metadataMediator_->getObjectAttributesManager();

    int boxId =
        objectAttributesManager->registerObject(ObjectAttributes, objectFile);
    auto objTemplate = objectAttributesManager->getObjectByID(boxId);

    auto& drawables = sceneManager_->getSceneGraph(sceneID_).getDrawables();

    float stageCollisionMargin = 0.04;

    for (int testId = 0; testId < 3; ++testId) {
      auto objWrapper0 =
          makeObjectGetWrapper(objTemplate->getHandle(), &drawables);
      auto objWrapper1 =
          makeObjectGetWrapper(objTemplate->getHandle(), &drawables);

      switch (testId) {
        case 0: {
          // test 0: stacking two DYNAMIC objects
          objWrapper0->setTranslation(
              {0, stageCollisionMargin + boxHalfExtent, 0});
          objWrapper1->setTranslation(
              {0, stageCollisionMargin + boxHalfExtent * 3, 0});

          while (physicsManager_->getWorldTime() < 6.0) {
            physicsManager_->stepPhysics(0.1);
          }
          CORRADE_VERIFY(!objWrapper0->isActive());
          CORRADE_VERIFY(!objWrapper1->isActive());
          CORRADE_COMPARE_AS(
              (objWrapper0->getTranslation() -
               Magnum::Vector3{0.0, stageCollisionMargin + boxHalfExtent, 0.0})
                  .length(),
              1.0e-3, Cr::TestSuite::Compare::LessOrEqual);
          CORRADE_COMPARE_AS(
              (objWrapper1->getTranslation() -
               Magnum::Vector3{0.0, stageCollisionMargin + boxHalfExtent * 3,
                               0.0})
                  .length(),
              1.0e-3, Cr::TestSuite::Compare::LessOrEqual);
        } break;
        case 1: {
          // test 1: stacking a DYNAMIC object on a STATIC object
          objWrapper0->setTranslation({0, boxHalfExtent * 2, 0});
          objWrapper0->setMotionType(esp::physics::MotionType::STATIC);
          objWrapper1->setTranslation({0, boxHalfExtent * 5, 0});

          while (physicsManager_->getWorldTime() < 6.0) {
            physicsManager_->stepPhysics(0.1);
          }
          CORRADE_VERIFY(!objWrapper1->isActive());
          CORRADE_COMPARE_AS((objWrapper0->getTranslation() -
                              Magnum::Vector3{0.0, boxHalfExtent * 2, 0.0})
                                 .length(),
                             1.0e-4, Cr::TestSuite::Compare::LessOrEqual);
          CORRADE_COMPARE_AS((objWrapper1->getTranslation() -
                              Magnum::Vector3{0.0, boxHalfExtent * 4, 0.0})
                                 .length(),
                             2.0e-4, Cr::TestSuite::Compare::LessOrEqual);
        } break;
        case 2: {
          // test 2: stacking a DYNAMIC object on a moving KINEMATIC object
          objWrapper0->setTranslation({0, boxHalfExtent * 2, 0});
          objWrapper0->setMotionType(esp::physics::MotionType::KINEMATIC);

          esp::physics::VelocityControl::ptr velCon =
              objWrapper0->getVelocityControl();
          velCon->controllingLinVel = true;
          velCon->linVel = {0.2, 0, 0};

          objWrapper1->setTranslation({0, boxHalfExtent * 5, 0});

          while (physicsManager_->getWorldTime() < 3.0) {
            // take single sub-steps for velocity control precision
            physicsManager_->stepPhysics(-1);
          }
          CORRADE_COMPARE_AS((objWrapper0->getTranslation() -
                              Magnum::Vector3{0.6, boxHalfExtent * 2, 0.0})
                                 .length(),
                             1.0e-3, Cr::TestSuite::Compare::LessOrEqual);
          CORRADE_COMPARE_AS((objWrapper1->getTranslation() -
                              Magnum::Vector3{0.559155, boxHalfExtent * 4, 0.0})
                                 .length(),
                             2.0e-3, Cr::TestSuite::Compare::LessOrEqual);
        } break;
      }

      // reset the scene
      rigidObjectManager_->removeAllObjects();
      physicsManager_->reset();  // time=0
    }
  }
}  // PhysicsTest::testMotionTypes

void PhysicsTest::testRemoveSleepingSupport() {
  // test that removing a sleeping support object wakes its collision island
  resetCreateRendererFlag(RendererEnabledData[testCaseInstanceId()].enabled);

  std::string stageFile = "NONE";

  initStage(stageFile);
  auto& drawables = sceneManager_->getSceneGraph(sceneID_).getDrawables();

  // We need dynamics to test this.
  if (physicsManager_->getPhysicsSimulationLibrary() !=
      PhysicsManager::PhysicsSimulationLibrary::NoPhysics) {
    auto objectAttributesManager =
        metadataMediator_->getObjectAttributesManager();

    std::string cubeHandle =
        objectAttributesManager->getObjectHandlesBySubstring("cubeSolid")[0];

    // create a stack of cubes in free space
    Mn::Vector3 stackBase(0.21964, 1.29183, -0.0897472);
    std::vector<esp::physics::ManagedRigidObject::ptr> cubes;
    int stackSize = 4;
    for (int i = 0; i < stackSize; ++i) {
      auto cubeWrapper = makeObjectGetWrapper(cubeHandle, &drawables);
      cubeWrapper->setTranslation((Mn::Vector3(0, 0.2, 0) * i) + stackBase);
      cubes.push_back(cubeWrapper);
    }

    cubes[0]->setMotionType(esp::physics::MotionType::STATIC);

    for (int testCase = 0; testCase < 2; ++testCase) {
      // reset time to 0, should not otherwise modify state
      physicsManager_->reset();
      CORRADE_COMPARE_AS(physicsManager_->getNumRigidObjects(), 0,
                         Cr::TestSuite::Compare::Greater);

      // simulate to stabilize the stack and populate collision islands
      while (physicsManager_->getWorldTime() < 4.0) {
        physicsManager_->stepPhysics(0.1);
      }

      // cubes should be sleeping
      for (auto cube : cubes) {
        CORRADE_VERIFY(!cube->isActive());
      }

      // no active contact points
      CORRADE_COMPARE(physicsManager_->getNumActiveContactPoints(), 0);

      if (testCase == 0) {
        // first remove the bottom-most DYNAMIC object, expecting those above
        // to fall
        rigidObjectManager_->removePhysObjectByID(cubes[1]->getID());
        cubes.erase(cubes.begin() + 1);
      } else if (testCase == 1) {
        // second remove the STATIC bottom cube
        rigidObjectManager_->removePhysObjectByID(cubes[0]->getID());
        cubes.erase(cubes.begin());
      }

      // remaining cubes should now be awake
      for (auto cube : cubes) {
        if (cube->getMotionType() != esp::physics::MotionType::STATIC) {
          CORRADE_VERIFY(cube->isActive());
        }
      }
      CORRADE_COMPARE_AS(physicsManager_->getNumActiveContactPoints(), 0,
                         Cr::TestSuite::Compare::Greater);
    }
  }
}  // PhysicsTest::testRemoveSleepingSupport

void PhysicsTest::testNumActiveContactPoints() {
  resetCreateRendererFlag(RendererEnabledData[testCaseInstanceId()].enabled);

  std::string stageFile =
      Cr::Utility::Path::join(dataDir, "test_assets/scenes/simple_room.glb");

  initStage(stageFile);
  auto& drawables = sceneManager_->getSceneGraph(sceneID_).getDrawables();

  // We need dynamics to test this.
  if (physicsManager_->getPhysicsSimulationLibrary() !=
      PhysicsManager::PhysicsSimulationLibrary::NoPhysics) {
    auto objectAttributesManager =
        metadataMediator_->getObjectAttributesManager();

    std::string cubeHandle =
        objectAttributesManager->getObjectHandlesBySubstring("cubeSolid")[0];

    // add a single cube
    Mn::Vector3 stackBase(0.21964, 1.29183, -0.0897472);
    auto objWrapper0 = makeObjectGetWrapper(cubeHandle, &drawables);
    objWrapper0->setTranslation(stackBase);

    // no active contact points at start
    CORRADE_COMPARE(physicsManager_->getNumActiveContactPoints(), 0);
    CORRADE_COMPARE(physicsManager_->getNumActiveOverlappingPairs(), 0);
    CORRADE_COMPARE(physicsManager_->getStepCollisionSummary(),
                    "(no active collision manifolds)\n");

    // simulate to let cube fall and hit the ground
    while (physicsManager_->getWorldTime() < 2.0) {
      physicsManager_->stepPhysics(0.1);
    }

    auto allContactPoints = physicsManager_->getContactPoints();
    // expect 4 active contact points for cube
    CORRADE_COMPARE(allContactPoints.size(), 4);
    CORRADE_COMPARE(physicsManager_->getNumActiveContactPoints(), 4);
    // simple_room.glb has multiple subparts and our box is near two of them
    CORRADE_COMPARE(physicsManager_->getNumActiveOverlappingPairs(), 2);
    CORRADE_COMPARE(
        physicsManager_->getStepCollisionSummary(),
        "[RigidObject, cubeSolid, id 0] vs [Stage, subpart 6], 4 points\n");

    float totalNormalForce = 0;
    for (auto& cp : allContactPoints) {
      // contacts are still active
      CORRADE_VERIFY(cp.isActive);
      // normal direction is unit Y (world up)
      CORRADE_COMPARE_AS(
          (cp.contactNormalOnBInWS - Magnum::Vector3{0.0, 1.0, 0.0}).length(),
          1.0e-4, Cr::TestSuite::Compare::LessOrEqual);
      // one object is the cube (0), other is the stage (-1)
      CORRADE_COMPARE(cp.objectIdA, 0);
      CORRADE_COMPARE(cp.objectIdB, -1);
      // accumulate the normal force
      totalNormalForce += cp.normalForce;
      // solver should keep the cube at the contact boundary (~0 penetration)
      CORRADE_COMPARE_AS(cp.contactDistance, 1.0e-4,
                         Cr::TestSuite::Compare::LessOrEqual);
    }
    // mass 1 cube under gravity should require normal contact force of ~9.8
    CORRADE_COMPARE_AS(totalNormalForce - 9.8, 3.0e-4,
                       Cr::TestSuite::Compare::LessOrEqual);

    // continue simulation until the cube is stable and sleeping
    while (physicsManager_->getWorldTime() < 4.0) {
      physicsManager_->stepPhysics(0.1);
    }
    // 4 inactive contact points at end
    CORRADE_COMPARE(physicsManager_->getContactPoints().size(), 4);
    // no active contact points at end
    CORRADE_COMPARE(physicsManager_->getNumActiveContactPoints(), 0);
    CORRADE_COMPARE(physicsManager_->getNumActiveOverlappingPairs(), 0);
  }
}  // PhysicsTest::testNumActiveContactPoints

#endif

void PhysicsTest::testConfigurableScaling() {
  // test scaling of objects via template configuration (visual and collision)

  resetCreateRendererFlag(RendererEnabledData[testCaseInstanceId()].enabled);

  std::string stageFile =
      Cr::Utility::Path::join(dataDir, "test_assets/scenes/plane.glb");

  std::string objectFile =
      Cr::Utility::Path::join(dataDir, "test_assets/objects/transform_box.glb");

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

  auto& drawables = sceneManager_->getSceneGraph(sceneID_).getDrawables();

  std::vector<int> objectIDs;
  for (auto& testScale : testScales) {
    objectTemplate->setScale(testScale);
    objectAttributesManager->registerObject(objectTemplate);

    Magnum::Range3D boundsGroundTruth(-abs(testScale), abs(testScale));

    auto objectWrapper = makeObjectGetWrapper(objectFile, &drawables);
    CORRADE_VERIFY(objectWrapper);

    objectIDs.push_back(objectWrapper->getID());

    const Magnum::Range3D& visualBounds =
        objectWrapper->getSceneNode()->getCumulativeBB();

    CORRADE_COMPARE(visualBounds, boundsGroundTruth);

// Test Bullet collision shape scaling
#ifdef ESP_BUILD_WITH_BULLET
    if (physicsManager_->getPhysicsSimulationLibrary() ==
        PhysicsManager::PhysicsSimulationLibrary::Bullet) {
      Magnum::Range3D aabb = objectWrapper->getCollisionShapeAabb();

      CORRADE_COMPARE(aabb, boundsGroundTruth);
    }
#endif
  }

  // check that scales are stored and queried correctly
  for (size_t ix = 0; ix < objectIDs.size(); ++ix) {
    CORRADE_COMPARE(
        rigidObjectManager_->getObjectCopyByID(objectIDs[ix])->getScale(),
        testScales[ix]);
  }
}  // PhysicsTest::testConfigurableScaling

void PhysicsTest::testVelocityControl() {
  // test scaling of objects via template configuration (visual and collision)

  resetCreateRendererFlag(RendererEnabledData[testCaseInstanceId()].enabled);

  std::string objectFile =
      Cr::Utility::Path::join(dataDir, "test_assets/objects/transform_box.glb");

  std::string stageFile =
      Cr::Utility::Path::join(dataDir, "test_assets/scenes/plane.glb");

  initStage(stageFile);

  ObjectAttributes::ptr ObjectAttributes = ObjectAttributes::create();
  ObjectAttributes->setRenderAssetHandle(objectFile);
  ObjectAttributes->setMargin(0.0);
  auto objectAttributesManager =
      metadataMediator_->getObjectAttributesManager();
  objectAttributesManager->registerObject(ObjectAttributes, objectFile);

  auto& drawables = sceneManager_->getSceneGraph(sceneID_).getDrawables();

  auto objectWrapper = makeObjectGetWrapper(objectFile, &drawables);
  CORRADE_VERIFY(objectWrapper);

  objectWrapper->setTranslation(Magnum::Vector3{0, 1.0, 0});

  Magnum::Vector3 commandLinVel(1.0, 1.0, 1.0);
  Magnum::Vector3 commandAngVel(1.0, 1.0, 1.0);

#ifdef ESP_BUILD_WITH_BULLET
  // test results of getting/setting
  if (physicsManager_->getPhysicsSimulationLibrary() ==
      PhysicsManager::PhysicsSimulationLibrary::Bullet) {
    objectWrapper->setLinearVelocity(commandLinVel);
    objectWrapper->setAngularVelocity(commandAngVel);

    CORRADE_COMPARE(objectWrapper->getLinearVelocity(), commandLinVel);
    CORRADE_COMPARE(objectWrapper->getAngularVelocity(), commandAngVel);

  } else
#endif
      if (physicsManager_->getPhysicsSimulationLibrary() ==
          PhysicsManager::PhysicsSimulationLibrary::NoPhysics) {
    objectWrapper->setLinearVelocity(commandLinVel);
    objectWrapper->setAngularVelocity(commandAngVel);

    // default kinematics always 0 velocity when queried
    CORRADE_COMPARE(objectWrapper->getLinearVelocity(), Magnum::Vector3{});
    CORRADE_COMPARE(objectWrapper->getAngularVelocity(), Magnum::Vector3{});
  }

  // test constant velocity control mechanism
  esp::physics::VelocityControl::ptr velControl =
      objectWrapper->getVelocityControl();
  velControl->controllingAngVel = true;
  velControl->controllingLinVel = true;
  velControl->linVel = Magnum::Vector3{1.0, -1.0, 1.0};
  velControl->angVel = Magnum::Vector3{1.0, 0, 0};

  // first kinematic
  objectWrapper->setMotionType(esp::physics::MotionType::KINEMATIC);
  objectWrapper->setTranslation(Magnum::Vector3{0, 2.0, 0});

  float targetTime = 2.0;
  while (physicsManager_->getWorldTime() < targetTime) {
    physicsManager_->stepPhysics(targetTime - physicsManager_->getWorldTime());
  }
  Magnum::Vector3 posGroundTruth{2.0, 0.0, 2.0};
  Magnum::Quaternion qGroundTruth{{0.842602, 0, 0}, 0.538537};

  float errorEps = 0.015;  // fairly loose due to discrete timestep
  CORRADE_COMPARE_AS(
      (objectWrapper->getTranslation() - posGroundTruth).length(), errorEps,
      Cr::TestSuite::Compare::LessOrEqual);
  Magnum::Rad angleError =
      Magnum::Math::angle(objectWrapper->getRotation(), qGroundTruth);

  CORRADE_COMPARE_AS(float(angleError), errorEps,
                     Cr::TestSuite::Compare::LessOrEqual);

  if (physicsManager_->getPhysicsSimulationLibrary() ==
      PhysicsManager::PhysicsSimulationLibrary::Bullet) {
    objectWrapper->setMotionType(esp::physics::MotionType::DYNAMIC);
    objectWrapper->resetTransformation();
    objectWrapper->setTranslation(Magnum::Vector3{0, 2.0, 0});
    physicsManager_->setGravity({});  // 0 gravity interference
    physicsManager_->reset();         // reset time to 0

    // should closely follow kinematic result while uninhibited in 0 gravity
    float targetTime = 0.5;
    esp::core::RigidState initialObjectState(objectWrapper->getRotation(),
                                             objectWrapper->getTranslation());
    esp::core::RigidState kinematicResult =
        velControl->integrateTransform(targetTime, initialObjectState);
    while (physicsManager_->getWorldTime() < targetTime) {
      physicsManager_->stepPhysics(physicsManager_->getTimestep());
    }
    CORRADE_COMPARE_AS(
        (objectWrapper->getTranslation() - kinematicResult.translation)
            .length(),
        errorEps, Cr::TestSuite::Compare::LessOrEqual);
    angleError = Magnum::Math::angle(objectWrapper->getRotation(),
                                     kinematicResult.rotation);
    CORRADE_COMPARE_AS(float(angleError), errorEps,
                       Cr::TestSuite::Compare::LessOrEqual);

    // should then get blocked by ground plane collision
    targetTime = 2.0;
    while (physicsManager_->getWorldTime() < targetTime) {
      physicsManager_->stepPhysics(physicsManager_->getTimestep());
    }
    CORRADE_COMPARE_AS(objectWrapper->getTranslation()[1], 1.0 - errorEps,
                       Cr::TestSuite::Compare::GreaterOrEqual);
  }

  // test local velocity
  objectWrapper->setMotionType(esp::physics::MotionType::KINEMATIC);
  objectWrapper->resetTransformation();
  objectWrapper->setTranslation(Magnum::Vector3{0, 2.0, 0});

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

  CORRADE_COMPARE_AS(
      (objectWrapper->getTranslation() - posLocalGroundTruth).length(),
      errorEps, Cr::TestSuite::Compare::LessOrEqual);
  Magnum::Rad angleErrorLocal =
      Magnum::Math::angle(objectWrapper->getRotation(), qLocalGroundTruth);

  CORRADE_COMPARE_AS(float(angleErrorLocal), errorEps,
                     Cr::TestSuite::Compare::LessOrEqual);
}  // PhysicsTest::testVelocityControl

void PhysicsTest::testSceneNodeAttachment() {
  // test attaching/detaching existing SceneNode to/from physical simulation

  resetCreateRendererFlag(RendererEnabledData[testCaseInstanceId()].enabled);

  std::string objectFile =
      Cr::Utility::Path::join(dataDir, "test_assets/objects/transform_box.glb");

  std::string stageFile =
      Cr::Utility::Path::join(dataDir, "test_assets/scenes/plane.glb");

  initStage(stageFile);

  ObjectAttributes::ptr ObjectAttributes = ObjectAttributes::create();
  ObjectAttributes->setRenderAssetHandle(objectFile);
  auto objectAttributesManager =
      metadataMediator_->getObjectAttributesManager();
  objectAttributesManager->registerObject(ObjectAttributes, objectFile);

  esp::scene::SceneNode& root =
      sceneManager_->getSceneGraph(sceneID_).getRootNode();
  esp::scene::SceneNode* newNode = &root.createChild();
  CORRADE_COMPARE(root.children().last(), newNode);

  auto& drawables = sceneManager_->getSceneGraph(sceneID_).getDrawables();

  // Test attaching newNode to a RigidBody

  auto objectWrapper = makeObjectGetWrapper(objectFile, &drawables, newNode);
  CORRADE_VERIFY(objectWrapper);

  CORRADE_COMPARE(objectWrapper->getSceneNode(), newNode);

  // Test updating newNode position with PhysicsManager
  Magnum::Vector3 newPos{1.0, 3.0, 0.0};
  objectWrapper->setTranslation(newPos);
  CORRADE_COMPARE(objectWrapper->getTranslation(), newPos);
  CORRADE_COMPARE(newNode->translation(), newPos);

  // Test leaving newNode without visualNode_ after destroying the RigidBody
  physicsManager_->removeObject(objectWrapper->getID(), false, true);
  CORRADE_VERIFY(newNode->children().isEmpty());

  // Test leaving the visualNode attached to newNode after destroying the
  // RigidBody
  objectWrapper = makeObjectGetWrapper(objectFile, &drawables, newNode);
  CORRADE_VERIFY(objectWrapper);
  physicsManager_->removeObject(objectWrapper->getID(), false, false);
  CORRADE_VERIFY(!newNode->children().isEmpty());

  // Test destroying newNode with the RigidBody
  objectWrapper = makeObjectGetWrapper(objectFile, &drawables, newNode);
  CORRADE_VERIFY(objectWrapper);
  physicsManager_->removeObject(objectWrapper->getID(), true, true);
  CORRADE_COMPARE_AS(root.children().last(), newNode,
                     Cr::TestSuite::Compare::NotEqual);
}  // PhysicsTest::testSceneNodeAttachment

}  // namespace

CORRADE_TEST_MAIN(PhysicsTest)
