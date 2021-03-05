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
};

SensorTest::SensorTest() {
  // clang-format off
  addTests({&SensorTest::testSensorFactory});
  addTests({&SensorTest::testSensorDestructors});
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

  // Remove sensor from parentNode
  SensorFactory::deleteSensor(parentNode, "1A");
  CORRADE_VERIFY(parentNode.getNodeSensorSuite().getSensors().size() == 1);
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 8);
  CORRADE_VERIFY(childNode.getNodeSensorSuite().getSensors().size() == 3);
  CORRADE_VERIFY(childNode.getSubtreeSensorSuite().getSensors().size() == 7);
  CORRADE_VERIFY(grandchildNode.getNodeSensorSuite().getSensors().size() == 4);
  CORRADE_VERIFY(grandchildNode.getSubtreeSensorSuite().getSensors().size() ==
                 4);

  // Remove sensor from child node
  SensorFactory::deleteSensor(parentNode, "2A");
  CORRADE_VERIFY(parentNode.getNodeSensorSuite().getSensors().size() == 1);
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 7);
  CORRADE_VERIFY(childNode.getNodeSensorSuite().getSensors().size() == 2);
  CORRADE_VERIFY(childNode.getSubtreeSensorSuite().getSensors().size() == 6);
  CORRADE_VERIFY(grandchildNode.getNodeSensorSuite().getSensors().size() == 4);
  CORRADE_VERIFY(grandchildNode.getSubtreeSensorSuite().getSensors().size() ==
                 4);

  // Remove sensor from grandchild node
  SensorFactory::deleteSensor(parentNode, "3A");
  CORRADE_VERIFY(parentNode.getNodeSensorSuite().getSensors().size() == 1);
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 6);
  CORRADE_VERIFY(childNode.getNodeSensorSuite().getSensors().size() == 2);
  CORRADE_VERIFY(childNode.getSubtreeSensorSuite().getSensors().size() == 5);
  CORRADE_VERIFY(grandchildNode.getNodeSensorSuite().getSensors().size() == 3);
  CORRADE_VERIFY(grandchildNode.getSubtreeSensorSuite().getSensors().size() ==
                 3);

  // Remove child node and assert grandchild node is destructed as well
  delete (&childNode);
  CORRADE_VERIFY(parentNode.getNodeSensorSuite().getSensors().size() == 1);
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 1);
}

CORRADE_TEST_MAIN(SensorTest)
