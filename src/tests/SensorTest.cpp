// Copyright (c) Meta Platforms, Inc. and its affiliates.
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

namespace {
// TODO: Add tests for different Sensors
struct SensorTest : Cr::TestSuite::Tester {
  explicit SensorTest();

  void testSensorFactory();
  void testSensorDestructors();
  void testSetParent();

 private:
  esp::logging::LoggingContext loggingContext_;
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
  CORRADE_COMPARE(parentNode.getId(), 1);
  SceneNode& childNode = parentNode.createChild();
  childNode.setId(2);
  CORRADE_COMPARE(childNode.getId(), 2);

  // Add different uuid sensors to same node and assert increase
  auto sensorSpecA = CameraSensorSpec::create();
  sensorSpecA->uuid = "A";
  auto sensorSpecB = CameraSensorSpec::create();
  sensorSpecB->uuid = "B";
  SensorFactory::createSensors(parentNode, {sensorSpecA, sensorSpecB});
  CORRADE_COMPARE(rootNode.getNodeSensors().size(), 0);
  CORRADE_COMPARE(rootNode.getSubtreeSensors().size(), 2);
  CORRADE_COMPARE(parentNode.getNodeSensors().size(), 2);
  CORRADE_COMPARE(parentNode.getSubtreeSensors().size(), 2);

  // Add different uuid sensors to different nodes and assert increase
  auto sensorSpecC = CameraSensorSpec::create();
  sensorSpecC->uuid = "C";
  SensorFactory::createSensors(parentNode, {sensorSpecC});
  CORRADE_COMPARE(rootNode.getNodeSensors().size(), 0);
  CORRADE_COMPARE(rootNode.getSubtreeSensors().size(), 3);
  CORRADE_COMPARE(parentNode.getNodeSensors().size(), 3);
  CORRADE_COMPARE(parentNode.getSubtreeSensors().size(), 3);

  auto sensorSpecD = CameraSensorSpec::create();
  sensorSpecD->uuid = "D";
  auto sensorSpecE = CameraSensorSpec::create();
  sensorSpecE->uuid = "E";
  SensorFactory::createSensors(childNode, {sensorSpecD, sensorSpecE});
  CORRADE_COMPARE(rootNode.getNodeSensors().size(), 0);
  CORRADE_COMPARE(rootNode.getSubtreeSensors().size(), 5);
  CORRADE_COMPARE(parentNode.getNodeSensors().size(), 3);
  CORRADE_COMPARE(parentNode.getSubtreeSensors().size(), 5);
  CORRADE_COMPARE(childNode.getNodeSensors().size(), 2);
  CORRADE_COMPARE(childNode.getSubtreeSensors().size(), 2);

  // Add same uuid sensor to same node and assert that only one sensor was added
  auto sensorSpecF = CameraSensorSpec::create();
  sensorSpecF->uuid = "F";
  SensorFactory::createSensors(parentNode, {sensorSpecF, sensorSpecF});
  CORRADE_COMPARE(rootNode.getNodeSensors().size(), 0);
  CORRADE_COMPARE(rootNode.getSubtreeSensors().size(), 6);
  CORRADE_COMPARE(parentNode.getNodeSensors().size(), 4);
  CORRADE_COMPARE(parentNode.getSubtreeSensors().size(), 6);
}

void SensorTest::testSensorDestructors() {
  SceneManager sceneManager_;

  int sceneID = sceneManager_.initSceneGraph();
  auto& sceneGraph = sceneManager_.getSceneGraph(sceneID);

  // retrieve root node
  auto& rootNode = sceneGraph.getRootNode();
  SceneNode& parentNode = rootNode.createChild();
  parentNode.setId(1);
  CORRADE_COMPARE(parentNode.getId(), 1);
  SceneNode& childNode = parentNode.createChild();
  childNode.setId(2);
  CORRADE_COMPARE(childNode.getId(), 2);
  SceneNode& grandchildNode = childNode.createChild();
  grandchildNode.setId(3);
  CORRADE_COMPARE(grandchildNode.getId(), 3);
  SceneNode& grandchild2Node = childNode.createChild();
  grandchild2Node.setId(4);
  CORRADE_COMPARE(grandchild2Node.getId(), 4);
  SceneNode& greatgrandchildNode = grandchildNode.createChild();
  greatgrandchildNode.setId(5);
  CORRADE_COMPARE(greatgrandchildNode.getId(), 5);

  // Add sensors to parent node
  auto sensorSpec1A = CameraSensorSpec::create();
  sensorSpec1A->uuid = "1A";
  auto sensorSpec1B = CameraSensorSpec::create();
  sensorSpec1B->uuid = "1B";
  SensorFactory::createSensors(parentNode, {sensorSpec1A, sensorSpec1B});
  CORRADE_COMPARE(rootNode.getNodeSensors().size(), 0);
  CORRADE_COMPARE(rootNode.getSubtreeSensors().size(), 2);
  CORRADE_COMPARE(parentNode.getNodeSensors().size(), 2);
  CORRADE_COMPARE(parentNode.getSubtreeSensors().size(), 2);

  // Add sensors to child node
  auto sensorSpec2A = CameraSensorSpec::create();
  sensorSpec2A->uuid = "2A";
  auto sensorSpec2B = CameraSensorSpec::create();
  sensorSpec2B->uuid = "2B";
  auto sensorSpec2C = CameraSensorSpec::create();
  sensorSpec2C->uuid = "2C";
  SensorFactory::createSensors(childNode,
                               {sensorSpec2A, sensorSpec2B, sensorSpec2C});
  CORRADE_COMPARE(rootNode.getNodeSensors().size(), 0);
  CORRADE_COMPARE(rootNode.getSubtreeSensors().size(), 5);
  CORRADE_COMPARE(parentNode.getNodeSensors().size(), 2);
  CORRADE_COMPARE(parentNode.getSubtreeSensors().size(), 5);
  CORRADE_COMPARE(childNode.getNodeSensors().size(), 3);
  CORRADE_COMPARE(childNode.getSubtreeSensors().size(), 3);

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
  CORRADE_COMPARE(rootNode.getNodeSensors().size(), 0);
  CORRADE_COMPARE(rootNode.getSubtreeSensors().size(), 9);
  CORRADE_COMPARE(parentNode.getNodeSensors().size(), 2);
  CORRADE_COMPARE(parentNode.getSubtreeSensors().size(), 9);
  CORRADE_COMPARE(childNode.getNodeSensors().size(), 3);
  CORRADE_COMPARE(childNode.getSubtreeSensors().size(), 7);
  CORRADE_COMPARE(grandchildNode.getNodeSensors().size(), 4);
  CORRADE_COMPARE(grandchildNode.getSubtreeSensors().size(), 4);

  // Add sensors to grandchild2 node
  auto sensorSpec4A = CameraSensorSpec::create();
  sensorSpec4A->uuid = "4A";
  auto sensorSpec4B = CameraSensorSpec::create();
  sensorSpec4B->uuid = "4B";
  SensorFactory::createSensors(grandchild2Node, {sensorSpec4A, sensorSpec4B});
  CORRADE_COMPARE(rootNode.getNodeSensors().size(), 0);
  CORRADE_COMPARE(rootNode.getSubtreeSensors().size(), 11);
  CORRADE_COMPARE(parentNode.getNodeSensors().size(), 2);
  CORRADE_COMPARE(parentNode.getSubtreeSensors().size(), 11);
  CORRADE_COMPARE(childNode.getNodeSensors().size(), 3);
  CORRADE_COMPARE(childNode.getSubtreeSensors().size(), 9);
  CORRADE_COMPARE(grandchildNode.getNodeSensors().size(), 4);
  CORRADE_COMPARE(grandchildNode.getSubtreeSensors().size(), 4);
  CORRADE_COMPARE(grandchild2Node.getNodeSensors().size(), 2);
  CORRADE_COMPARE(grandchild2Node.getSubtreeSensors().size(), 2);

  // Add sensors to greatgrandchild node
  auto sensorSpec5A = CameraSensorSpec::create();
  sensorSpec5A->uuid = "5A";
  auto sensorSpec5B = CameraSensorSpec::create();
  sensorSpec5B->uuid = "5B";
  SensorFactory::createSensors(greatgrandchildNode,
                               {sensorSpec5A, sensorSpec5B});
  CORRADE_COMPARE(rootNode.getNodeSensors().size(), 0);
  CORRADE_COMPARE(rootNode.getSubtreeSensors().size(), 13);
  CORRADE_COMPARE(parentNode.getNodeSensors().size(), 2);
  CORRADE_COMPARE(parentNode.getSubtreeSensors().size(), 13);
  CORRADE_COMPARE(childNode.getNodeSensors().size(), 3);
  CORRADE_COMPARE(childNode.getSubtreeSensors().size(), 11);
  CORRADE_COMPARE(grandchildNode.getNodeSensors().size(), 4);
  CORRADE_COMPARE(grandchildNode.getSubtreeSensors().size(), 6);
  CORRADE_COMPARE(grandchild2Node.getNodeSensors().size(), 2);
  CORRADE_COMPARE(grandchild2Node.getSubtreeSensors().size(), 2);
  CORRADE_COMPARE(greatgrandchildNode.getNodeSensors().size(), 2);
  CORRADE_COMPARE(greatgrandchildNode.getSubtreeSensors().size(), 2);

  // Remove sensor from parentNode
  SensorFactory::deleteSubtreeSensor(parentNode, "1A");
  CORRADE_COMPARE(rootNode.getNodeSensors().size(), 0);
  CORRADE_COMPARE(rootNode.getSubtreeSensors().size(), 12);
  CORRADE_COMPARE(parentNode.getNodeSensors().size(), 1);
  CORRADE_COMPARE(parentNode.getSubtreeSensors().size(), 12);
  CORRADE_COMPARE(childNode.getNodeSensors().size(), 3);
  CORRADE_COMPARE(childNode.getSubtreeSensors().size(), 11);
  CORRADE_COMPARE(grandchildNode.getNodeSensors().size(), 4);
  CORRADE_COMPARE(grandchildNode.getSubtreeSensors().size(), 6);
  CORRADE_COMPARE(grandchild2Node.getNodeSensors().size(), 2);
  CORRADE_COMPARE(grandchild2Node.getSubtreeSensors().size(), 2);
  CORRADE_COMPARE(greatgrandchildNode.getNodeSensors().size(), 2);
  CORRADE_COMPARE(greatgrandchildNode.getSubtreeSensors().size(), 2);

  // Remove sensor from child node
  SensorFactory::deleteSubtreeSensor(parentNode, "2A");
  CORRADE_COMPARE(rootNode.getNodeSensors().size(), 0);
  CORRADE_COMPARE(rootNode.getSubtreeSensors().size(), 11);
  CORRADE_COMPARE(parentNode.getNodeSensors().size(), 1);
  CORRADE_COMPARE(parentNode.getSubtreeSensors().size(), 11);
  CORRADE_COMPARE(childNode.getNodeSensors().size(), 2);
  CORRADE_COMPARE(childNode.getSubtreeSensors().size(), 10);
  CORRADE_COMPARE(grandchildNode.getNodeSensors().size(), 4);
  CORRADE_COMPARE(grandchildNode.getSubtreeSensors().size(), 6);
  CORRADE_COMPARE(grandchild2Node.getNodeSensors().size(), 2);
  CORRADE_COMPARE(grandchild2Node.getSubtreeSensors().size(), 2);
  CORRADE_COMPARE(greatgrandchildNode.getNodeSensors().size(), 2);
  CORRADE_COMPARE(greatgrandchildNode.getSubtreeSensors().size(), 2);

  // Remove sensor from grandchild node
  SensorFactory::deleteSubtreeSensor(parentNode, "3A");
  CORRADE_COMPARE(rootNode.getNodeSensors().size(), 0);
  CORRADE_COMPARE(rootNode.getSubtreeSensors().size(), 10);
  CORRADE_COMPARE(parentNode.getNodeSensors().size(), 1);
  CORRADE_COMPARE(parentNode.getSubtreeSensors().size(), 10);
  CORRADE_COMPARE(childNode.getNodeSensors().size(), 2);
  CORRADE_COMPARE(childNode.getSubtreeSensors().size(), 9);
  CORRADE_COMPARE(grandchildNode.getNodeSensors().size(), 3);
  CORRADE_COMPARE(grandchildNode.getSubtreeSensors().size(), 5);
  CORRADE_COMPARE(grandchild2Node.getNodeSensors().size(), 2);
  CORRADE_COMPARE(grandchild2Node.getSubtreeSensors().size(), 2);
  CORRADE_COMPARE(greatgrandchildNode.getNodeSensors().size(), 2);
  CORRADE_COMPARE(greatgrandchildNode.getSubtreeSensors().size(), 2);

  // Remove sensor from greatgrandchild node
  SensorFactory::deleteSubtreeSensor(parentNode, "5A");
  CORRADE_COMPARE(rootNode.getNodeSensors().size(), 0);
  CORRADE_COMPARE(rootNode.getSubtreeSensors().size(), 9);
  CORRADE_COMPARE(parentNode.getNodeSensors().size(), 1);
  CORRADE_COMPARE(parentNode.getSubtreeSensors().size(), 9);
  CORRADE_COMPARE(childNode.getNodeSensors().size(), 2);
  CORRADE_COMPARE(childNode.getSubtreeSensors().size(), 8);
  CORRADE_COMPARE(grandchildNode.getNodeSensors().size(), 3);
  CORRADE_COMPARE(grandchildNode.getSubtreeSensors().size(), 4);
  CORRADE_COMPARE(grandchild2Node.getNodeSensors().size(), 2);
  CORRADE_COMPARE(grandchild2Node.getSubtreeSensors().size(), 2);
  CORRADE_COMPARE(greatgrandchildNode.getNodeSensors().size(), 1);
  CORRADE_COMPARE(greatgrandchildNode.getSubtreeSensors().size(), 1);

  // Remove grandchild node and assert grandchild2Node still exists
  // Before, verify that childNode has 2 children nodes
  auto* grandchild = childNode.children().first();
  CORRADE_VERIFY(grandchild);
  // remaining grandchild is grandchild2 with id 4
  CORRADE_COMPARE(dynamic_cast<SceneNode*>(grandchild)->getId(), 3);
  CORRADE_COMPARE(dynamic_cast<SceneNode*>(grandchild->nextSibling())->getId(),
                  4);

  delete (&grandchildNode);
  CORRADE_COMPARE(rootNode.getNodeSensors().size(), 0);
  CORRADE_COMPARE(rootNode.getSubtreeSensors().size(), 5);
  CORRADE_COMPARE(parentNode.getNodeSensors().size(), 1);
  CORRADE_COMPARE(parentNode.getSubtreeSensors().size(), 5);
  CORRADE_COMPARE(childNode.getNodeSensors().size(), 2);
  CORRADE_COMPARE(childNode.getSubtreeSensors().size(), 4);
  CORRADE_COMPARE(grandchild2Node.getNodeSensors().size(), 2);
  CORRADE_COMPARE(grandchild2Node.getSubtreeSensors().size(), 2);

  // After, verify that childNode has 1 child node with id 4 (grandchild2Node)
  // and 2 children nodes with id 2, and no other children
  grandchild = childNode.children().first();
  CORRADE_VERIFY(grandchild);
  // remaining grandchild is grandchild2 with id 4
  CORRADE_COMPARE(dynamic_cast<SceneNode*>(grandchild)->getId(), 4);
  auto* nextGrandchild = grandchild->nextSibling();
  CORRADE_COMPARE(dynamic_cast<SceneNode*>(nextGrandchild)->getId(), 2);
  nextGrandchild = nextGrandchild->nextSibling();
  CORRADE_COMPARE(dynamic_cast<SceneNode*>(nextGrandchild)->getId(), 2);
  nextGrandchild = nextGrandchild->nextSibling();
  CORRADE_VERIFY(!nextGrandchild);

  // Delete sensor that doesn't exist
  // Message that Sensor does not exist will be logged, but no errors will be
  // thrown
  SensorFactory::deleteSubtreeSensor(parentNode, "1C");
}

void SensorTest::testSetParent() {
  SceneManager sceneManager_;

  int sceneID = sceneManager_.initSceneGraph();
  auto& sceneGraph = sceneManager_.getSceneGraph(sceneID);

  // retrieve root node
  auto& rootNode = sceneGraph.getRootNode();
  SceneNode& parentNode = rootNode.createChild();
  parentNode.setId(1);
  CORRADE_COMPARE(parentNode.getId(), 1);
  SceneNode& childNode = parentNode.createChild();
  childNode.setId(2);
  CORRADE_COMPARE(childNode.getId(), 2);
  SceneNode& grandchildNode = childNode.createChild();
  grandchildNode.setId(3);
  CORRADE_COMPARE(grandchildNode.getId(), 3);
  SceneNode& child2Node = childNode.createChild();
  child2Node.setId(4);
  CORRADE_COMPARE(child2Node.getId(), 4);

  // Add sensors to child2 node and assert correct number of sensors in child
  // and parent subtreeSensorSuites
  auto sensorSpec4A = CameraSensorSpec::create();
  sensorSpec4A->uuid = "4A";
  auto sensorSpec4B = CameraSensorSpec::create();
  sensorSpec4B->uuid = "4B";
  SensorFactory::createSensors(child2Node, {sensorSpec4A, sensorSpec4B});
  CORRADE_COMPARE(rootNode.getNodeSensors().size(), 0);
  CORRADE_COMPARE(rootNode.getSubtreeSensors().size(), 2);
  CORRADE_COMPARE(parentNode.getNodeSensors().size(), 0);
  CORRADE_COMPARE(parentNode.getSubtreeSensors().size(), 2);
  CORRADE_COMPARE(childNode.getNodeSensors().size(), 0);
  CORRADE_COMPARE(childNode.getSubtreeSensors().size(), 2);
  CORRADE_COMPARE(grandchildNode.getNodeSensors().size(), 0);
  CORRADE_COMPARE(grandchildNode.getSubtreeSensors().size(), 0);
  CORRADE_COMPARE(child2Node.getNodeSensors().size(), 2);
  CORRADE_COMPARE(child2Node.getSubtreeSensors().size(), 2);

  // Set parent of child2 node to parentNode and assert correct number of
  // sensors in child and parent subtreeSensorSuites
  child2Node.setParent(&parentNode);
  // Assert rootNode SensorSuites unchanged
  CORRADE_COMPARE(rootNode.getNodeSensors().size(), 0);
  CORRADE_COMPARE(rootNode.getSubtreeSensors().size(), 2);
  // Assert parentNode subtreeSensorSuite unchanged
  CORRADE_COMPARE(parentNode.getNodeSensors().size(), 0);
  CORRADE_COMPARE(parentNode.getSubtreeSensors().size(), 2);
  // Assert childNode subtreeSensorSuite no longer holds sensors
  CORRADE_COMPARE(childNode.getNodeSensors().size(), 0);
  CORRADE_COMPARE(childNode.getSubtreeSensors().size(), 0);
  // Assert grandchildNode subtreeSensorSuite unchanged
  CORRADE_COMPARE(grandchildNode.getNodeSensors().size(), 0);
  CORRADE_COMPARE(grandchildNode.getSubtreeSensors().size(), 0);
  // Assert child2Node subtreeSensorSuite unchanged
  CORRADE_COMPARE(child2Node.getNodeSensors().size(), 2);
  CORRADE_COMPARE(child2Node.getSubtreeSensors().size(), 2);

  // Set parent of sensor to grandchildNode and assert correct number of sensors
  // in sensorSuites
  parentNode.getSubtreeSensorSuite().get("4A").node().setParent(
      &grandchildNode);
  // Assert rootNode SensorSuites unchanged
  CORRADE_COMPARE(rootNode.getNodeSensors().size(), 0);
  CORRADE_COMPARE(rootNode.getSubtreeSensors().size(), 2);
  // Assert parentNode subtreeSensorSuite unchanged
  CORRADE_COMPARE(parentNode.getNodeSensors().size(), 0);
  CORRADE_COMPARE(parentNode.getSubtreeSensors().size(), 2);
  // Assert childNode subtreeSensorSuite no longer holds sensors
  CORRADE_COMPARE(childNode.getNodeSensors().size(), 0);
  CORRADE_COMPARE(childNode.getSubtreeSensors().size(), 1);
  // Assert grandchildNode subtreeSensorSuite unchanged
  CORRADE_COMPARE(grandchildNode.getNodeSensors().size(), 1);
  CORRADE_COMPARE(grandchildNode.getSubtreeSensors().size(), 1);
  // Assert child2Node subtreeSensorSuite decreases
  CORRADE_COMPARE(child2Node.getNodeSensors().size(), 1);
  CORRADE_COMPARE(child2Node.getSubtreeSensors().size(), 1);
}
}  // namespace

CORRADE_TEST_MAIN(SensorTest)
