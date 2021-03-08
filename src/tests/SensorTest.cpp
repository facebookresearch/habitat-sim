// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#include <Corrade/TestSuite/Tester.h>
#include <Magnum/Magnum.h>

#include "esp/scene/SceneManager.h"
#include "esp/scene/SceneNode.h"
#include "esp/sensor/CameraSensor.h"
#include "esp/sensor/Sensor.h"
#include "esp/sensor/SensorFactory.h"

namespace Cr = Corrade;
using namespace esp::sensor;
using namespace esp::scene;

// TODO: Add tests for different Sensors
struct SensorTest : Cr::TestSuite::Tester {
  explicit SensorTest();

  void testSensorFactory();
  void testSensorDestructors();
  void testSetParent();
};

SensorTest::SensorTest() {
  // clang-format off
  addTests({&SensorTest::testSensorFactory});
  addTests({&SensorTest::testSensorDestructors});
  addTests({&SensorTest::testSetParent});
  // clang-format on
}

void SensorTest::testSensorFactory() {
  SceneManager sceneManager_;

  int sceneID = sceneManager_.initSceneGraph();
  auto& sceneGraph = sceneManager_.getSceneGraph(sceneID);

  // retrieve root node
  auto& rootNode = sceneGraph.getRootNode();
  SceneNode& parentNode = rootNode.createChild();
  parentNode.setId(1);
  CORRADE_VERIFY(parentNode.getId() == 1);
  SceneNode& childNode = parentNode.createChild();
  childNode.setId(2);
  CORRADE_VERIFY(childNode.getId() == 2);

  // Add different uuid sensors to same node and assert increase
  auto sensorSpecA = CameraSensorSpec::create();
  sensorSpecA->uuid = "A";
  auto sensorSpecB = CameraSensorSpec::create();
  sensorSpecB->uuid = "B";
  SensorFactory::createSensors(parentNode, {sensorSpecA, sensorSpecB});
  CORRADE_VERIFY(parentNode.getNodeSensorSuite().getSensors().size() == 2);
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 2);

  // Add different uuid sensors to different nodes and assert increase
  auto sensorSpecC = CameraSensorSpec::create();
  sensorSpecC->uuid = "C";
  SensorFactory::createSensors(parentNode, {sensorSpecC});
  CORRADE_VERIFY(parentNode.getNodeSensorSuite().getSensors().size() == 3);
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 3);

  auto sensorSpecD = CameraSensorSpec::create();
  sensorSpecD->uuid = "D";
  auto sensorSpecE = CameraSensorSpec::create();
  sensorSpecE->uuid = "E";
  SensorFactory::createSensors(childNode, {sensorSpecD, sensorSpecE});
  CORRADE_VERIFY(parentNode.getNodeSensorSuite().getSensors().size() == 3);
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 5);
  CORRADE_VERIFY(childNode.getNodeSensorSuite().getSensors().size() == 2);
  CORRADE_VERIFY(childNode.getSubtreeSensorSuite().getSensors().size() == 2);

  // Add same uuid sensor to same node and assert that only one sensor was added
  auto sensorSpecF = CameraSensorSpec::create();
  sensorSpecF->uuid = "F";
  SensorFactory::createSensors(parentNode, {sensorSpecF, sensorSpecF});
  CORRADE_VERIFY(parentNode.getNodeSensorSuite().getSensors().size() == 4);
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 6);
}

void SensorTest::testSensorDestructors() {
  SceneManager sceneManager_;

  int sceneID = sceneManager_.initSceneGraph();
  auto& sceneGraph = sceneManager_.getSceneGraph(sceneID);

  // retrieve root node
  auto& rootNode = sceneGraph.getRootNode();
  SceneNode& parentNode = rootNode.createChild();
  parentNode.setId(1);
  CORRADE_VERIFY(parentNode.getId() == 1);
  SceneNode& childNode = parentNode.createChild();
  childNode.setId(2);
  CORRADE_VERIFY(childNode.getId() == 2);
  SceneNode& grandchildNode = childNode.createChild();
  grandchildNode.setId(3);
  CORRADE_VERIFY(grandchildNode.getId() == 3);
  SceneNode& grandchild2Node = childNode.createChild();
  grandchild2Node.setId(4);
  CORRADE_VERIFY(grandchild2Node.getId() == 4);
  SceneNode& greatgrandchildNode = grandchildNode.createChild();
  greatgrandchildNode.setId(5);
  CORRADE_VERIFY(greatgrandchildNode.getId() == 5);

  // Add sensors to parent node
  auto sensorSpec1A = CameraSensorSpec::create();
  sensorSpec1A->uuid = "1A";
  auto sensorSpec1B = CameraSensorSpec::create();
  sensorSpec1B->uuid = "1B";
  SensorFactory::createSensors(parentNode, {sensorSpec1A, sensorSpec1B});
  CORRADE_VERIFY(parentNode.getNodeSensorSuite().getSensors().size() == 2);
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 2);

  // Add sensors to child node
  auto sensorSpec2A = CameraSensorSpec::create();
  sensorSpec2A->uuid = "2A";
  auto sensorSpec2B = CameraSensorSpec::create();
  sensorSpec2B->uuid = "2B";
  auto sensorSpec2C = CameraSensorSpec::create();
  sensorSpec2C->uuid = "2C";
  SensorFactory::createSensors(childNode,
                               {sensorSpec2A, sensorSpec2B, sensorSpec2C});
  CORRADE_VERIFY(parentNode.getNodeSensorSuite().getSensors().size() == 2);
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 5);
  CORRADE_VERIFY(childNode.getNodeSensorSuite().getSensors().size() == 3);
  CORRADE_VERIFY(childNode.getSubtreeSensorSuite().getSensors().size() == 3);

  // Add sensors to grandchild node
  auto sensorSpec3A = CameraSensorSpec::create();
  sensorSpec3A->uuid = "3A";
  auto sensorSpec3B = CameraSensorSpec::create();
  sensorSpec3B->uuid = "3B";
  auto sensorSpec3C = CameraSensorSpec::create();
  sensorSpec3C->uuid = "3C";
  auto sensorSpec3D = CameraSensorSpec::create();
  sensorSpec3D->uuid = "3D";
  SensorFactory::createSensors(
      grandchildNode, {sensorSpec3A, sensorSpec3B, sensorSpec3C, sensorSpec3D});
  CORRADE_VERIFY(parentNode.getNodeSensorSuite().getSensors().size() == 2);
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 9);
  CORRADE_VERIFY(childNode.getNodeSensorSuite().getSensors().size() == 3);
  CORRADE_VERIFY(childNode.getSubtreeSensorSuite().getSensors().size() == 7);
  CORRADE_VERIFY(grandchildNode.getNodeSensorSuite().getSensors().size() == 4);
  CORRADE_VERIFY(grandchildNode.getSubtreeSensorSuite().getSensors().size() ==
                 4);

  // Add sensors to grandchild2 node
  auto sensorSpec4A = CameraSensorSpec::create();
  sensorSpec4A->uuid = "4A";
  auto sensorSpec4B = CameraSensorSpec::create();
  sensorSpec4B->uuid = "4B";
  SensorFactory::createSensors(grandchildNode, {sensorSpec4A, sensorSpec4B});
  CORRADE_VERIFY(parentNode.getNodeSensorSuite().getSensors().size() == 2);
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 11);
  CORRADE_VERIFY(childNode.getNodeSensorSuite().getSensors().size() == 3);
  CORRADE_VERIFY(childNode.getSubtreeSensorSuite().getSensors().size() == 9);
  CORRADE_VERIFY(grandchildNode.getNodeSensorSuite().getSensors().size() == 4);
  CORRADE_VERIFY(grandchildNode.getSubtreeSensorSuite().getSensors().size() ==
                 4);
  CORRADE_VERIFY(grandchild2Node.getNodeSensorSuite().getSensors().size() == 2);
  CORRADE_VERIFY(grandchild2Node.getSubtreeSensorSuite().getSensors().size() ==
                 2);

  // Add sensors to greatgrandchild node
  auto sensorSpec5A = CameraSensorSpec::create();
  sensorSpec5A->uuid = "5A";
  auto sensorSpec5B = CameraSensorSpec::create();
  sensorSpec5B->uuid = "5B";
  SensorFactory::createSensors(greatgrandchildNode,
                               {sensorSpec5A, sensorSpec5B});
  CORRADE_VERIFY(parentNode.getNodeSensorSuite().getSensors().size() == 2);
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 13);
  CORRADE_VERIFY(childNode.getNodeSensorSuite().getSensors().size() == 3);
  CORRADE_VERIFY(childNode.getSubtreeSensorSuite().getSensors().size() == 11);
  CORRADE_VERIFY(grandchildNode.getNodeSensorSuite().getSensors().size() == 4);
  CORRADE_VERIFY(grandchildNode.getSubtreeSensorSuite().getSensors().size() ==
                 6);
  CORRADE_VERIFY(grandchild2Node.getNodeSensorSuite().getSensors().size() == 2);
  CORRADE_VERIFY(grandchild2Node.getSubtreeSensorSuite().getSensors().size() ==
                 2);
  CORRADE_VERIFY(greatgrandchildNode.getNodeSensorSuite().getSensors().size() ==
                 2);
  CORRADE_VERIFY(
      greatgrandchildNode.getSubtreeSensorSuite().getSensors().size() == 2);

  // Remove sensor from parentNode
  SensorFactory::deleteSensor(parentNode, "1A");
  CORRADE_VERIFY(parentNode.getNodeSensorSuite().getSensors().size() == 1);
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 12);
  CORRADE_VERIFY(childNode.getNodeSensorSuite().getSensors().size() == 3);
  CORRADE_VERIFY(childNode.getSubtreeSensorSuite().getSensors().size() == 11);
  CORRADE_VERIFY(grandchildNode.getNodeSensorSuite().getSensors().size() == 4);
  CORRADE_VERIFY(grandchildNode.getSubtreeSensorSuite().getSensors().size() ==
                 6);
  CORRADE_VERIFY(grandchild2Node.getNodeSensorSuite().getSensors().size() == 2);
  CORRADE_VERIFY(grandchild2Node.getSubtreeSensorSuite().getSensors().size() ==
                 2);
  CORRADE_VERIFY(greatgrandchildNode.getNodeSensorSuite().getSensors().size() ==
                 2);
  CORRADE_VERIFY(
      greatgrandchildNode.getSubtreeSensorSuite().getSensors().size() == 2);

  // Remove sensor from child node
  SensorFactory::deleteSensor(parentNode, "2A");
  CORRADE_VERIFY(parentNode.getNodeSensorSuite().getSensors().size() == 1);
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 11);
  CORRADE_VERIFY(childNode.getNodeSensorSuite().getSensors().size() == 2);
  CORRADE_VERIFY(childNode.getSubtreeSensorSuite().getSensors().size() == 10);
  CORRADE_VERIFY(grandchildNode.getNodeSensorSuite().getSensors().size() == 4);
  CORRADE_VERIFY(grandchildNode.getSubtreeSensorSuite().getSensors().size() ==
                 6);
  CORRADE_VERIFY(grandchild2Node.getNodeSensorSuite().getSensors().size() == 2);
  CORRADE_VERIFY(grandchild2Node.getSubtreeSensorSuite().getSensors().size() ==
                 2);
  CORRADE_VERIFY(greatgrandchildNode.getNodeSensorSuite().getSensors().size() ==
                 2);
  CORRADE_VERIFY(
      greatgrandchildNode.getSubtreeSensorSuite().getSensors().size() == 2);

  // Remove sensor from grandchild node
  SensorFactory::deleteSensor(parentNode, "3A");
  CORRADE_VERIFY(parentNode.getNodeSensorSuite().getSensors().size() == 1);
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 10);
  CORRADE_VERIFY(childNode.getNodeSensorSuite().getSensors().size() == 2);
  CORRADE_VERIFY(childNode.getSubtreeSensorSuite().getSensors().size() == 9);
  CORRADE_VERIFY(grandchildNode.getNodeSensorSuite().getSensors().size() == 3);
  CORRADE_VERIFY(grandchildNode.getSubtreeSensorSuite().getSensors().size() ==
                 5);
  CORRADE_VERIFY(grandchild2Node.getNodeSensorSuite().getSensors().size() == 2);
  CORRADE_VERIFY(grandchild2Node.getSubtreeSensorSuite().getSensors().size() ==
                 2);
  CORRADE_VERIFY(greatgrandchildNode.getNodeSensorSuite().getSensors().size() ==
                 2);
  CORRADE_VERIFY(
      greatgrandchildNode.getSubtreeSensorSuite().getSensors().size() == 2);

  // Remove sensor from greatgrandchild node
  SensorFactory::deleteSensor(parentNode, "5A");
  CORRADE_VERIFY(parentNode.getNodeSensorSuite().getSensors().size() == 1);
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 9);
  CORRADE_VERIFY(childNode.getNodeSensorSuite().getSensors().size() == 2);
  CORRADE_VERIFY(childNode.getSubtreeSensorSuite().getSensors().size() == 8);
  CORRADE_VERIFY(grandchildNode.getNodeSensorSuite().getSensors().size() == 3);
  CORRADE_VERIFY(grandchildNode.getSubtreeSensorSuite().getSensors().size() ==
                 4);
  CORRADE_VERIFY(grandchild2Node.getNodeSensorSuite().getSensors().size() == 2);
  CORRADE_VERIFY(grandchild2Node.getSubtreeSensorSuite().getSensors().size() ==
                 2);
  CORRADE_VERIFY(greatgrandchildNode.getNodeSensorSuite().getSensors().size() ==
                 1);
  CORRADE_VERIFY(
      greatgrandchildNode.getSubtreeSensorSuite().getSensors().size() == 1);

  // Remove grandchild node and assert greatgrandchild node is destructed as
  // well, grandchild2 still exists
  delete (&grandchildNode);
  CORRADE_VERIFY(parentNode.getNodeSensorSuite().getSensors().size() == 1);
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 5);
  CORRADE_VERIFY(childNode.getNodeSensorSuite().getSensors().size() == 2);
  CORRADE_VERIFY(childNode.getSubtreeSensorSuite().getSensors().size() == 4);
  CORRADE_VERIFY(grandchild2Node.getNodeSensorSuite().getSensors().size() == 2);
  CORRADE_VERIFY(grandchild2Node.getSubtreeSensorSuite().getSensors().size() ==
                 2);
  auto* grandchild = childNode.children().first();
  CORRADE_VERIFY(grandchild != nullptr);
  CORRADE_VERIFY(grandchild->nextSibling() == nullptr);

  // Todo: delete a sensor that doesn't exist
}

void SensorTest::testSetParent() {}

CORRADE_TEST_MAIN(SensorTest)
