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
};

SensorTest::SensorTest() {
  // clang-format off
  addTests({&SensorTest::testSensorFactory});
  // clang-format on
}

void SensorTest::testSensorFactory() {
  SceneManager sceneManager_;

  int sceneID = sceneManager_.initSceneGraph();
  auto& sceneGraph = sceneManager_.getSceneGraph(sceneID);

  // retrieve root node
  auto& rootNode = sceneGraph.getRootNode();
  SceneNode parentNode = rootNode.createChild();
  parentNode.setId(1);
  CORRADE_VERIFY(parentNode.getId() == 1);
  SceneNode childNode = parentNode.createChild();
  childNode.setId(2);
  CORRADE_VERIFY(childNode.getId() == 2);

  // Add different uuid sensors to same node and assert increase
  auto sensorSpecA = CameraSensorSpec::create();
  sensorSpecA->uuid = "A";
  auto sensorSpecB = CameraSensorSpec::create();
  sensorSpecB->uuid = "B";
  SensorFactory::createSensors(parentNode, {sensorSpecA, sensorSpecB});
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 2);

  // Add different uuid sensors to different nodes and assert increase
  auto sensorSpecC = CameraSensorSpec::create();
  sensorSpecC->uuid = "C";
  SensorFactory::createSensors(parentNode, {sensorSpecC});
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 3);

  auto sensorSpecD = CameraSensorSpec::create();
  sensorSpecD->uuid = "D";
  SensorFactory::createSensors(childNode, {sensorSpecD});
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 4);
  CORRADE_VERIFY(childNode.getSubtreeSensorSuite().getSensors().size() == 1);

  // Add same uuid sensor to same node and assert that only one sensor was added
  auto sensorSpecE = CameraSensorSpec::create();
  sensorSpecE->uuid = "E";
  SensorFactory::createSensors(parentNode, {sensorSpecE, sensorSpecE});
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 5);

  // Add same uuid sensors to parent, then child nodes and assert that parentNode subtreeSensorSuite does not increase, but child subtreeSensorSuite does
  auto sensorSpecF = CameraSensorSpec::create();
  sensorSpecF->uuid = "F";
  SensorFactory::createSensors(parentNode, {sensorSpecF});
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 6);
  SensorFactory::createSensors(childNode, {sensorSpecF});
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 6);
  CORRADE_VERIFY(childNode.getSubtreeSensorSuite().getSensors().size() == 2);

  //Add same uuid sensors to child, then parent nodes and assert that parentNode subtreeSensorSuite only increases once
  auto sensorSpecG = CameraSensorSpec::create();
  sensorSpecG->uuid = "G";
  SensorFactory::createSensors(childNode, {sensorSpecG});
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 7);
  CORRADE_VERIFY(childNode.getSubtreeSensorSuite().getSensors().size() == 3);
  SensorFactory::createSensors(parentNode, {sensorSpecG});
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 7);
  CORRADE_VERIFY(childNode.getSubtreeSensorSuite().getSensors().size() == 3);

  // Remove sensor from parent node and assert that it is no longer in parent subtreeSensorSuite
  parentNode.getSubtreeSensorSuite().get("A").~Sensor();
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 6);

  //Remove sensor from child node and assert that it is no longer in parent and child subtreeSensorSuite
  childNode.getSubtreeSensorSuite().get("D").~Sensor();
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 5);
  CORRADE_VERIFY(childNode.getSubtreeSensorSuite().getSensors().size() == 2);

  //Delete child node and assert that sensors are no longer in parentNode
  childNode.~SceneNode();
  CORRADE_VERIFY(parentNode.getSubtreeSensorSuite().getSensors().size() == 3);
}

CORRADE_TEST_MAIN(SensorTest)
