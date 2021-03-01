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
  SensorSuite sensorSuite_;

  int sceneID = sceneManager_.initSceneGraph();
  auto& sceneGraph = sceneManager_.getSceneGraph(sceneID);

  // retrieve root node
  auto& rootNode = sceneGraph.getRootNode();
  SceneNode sceneNode1 = rootNode.createChild();
  sceneNode1.setId(1);
  CORRADE_VERIFY(sceneNode1.getId() == 1);
  SceneNode sceneNode2 = rootNode.createChild();
  sceneNode2.setId(2);
  CORRADE_VERIFY(sceneNode2.getId() == 2);

  // Add different uuid sensors to same node and assert increase
  auto sensorSpecA = CameraSensorSpec::create();
  sensorSpecA->uuid = "A";
  auto sensorSpecB = CameraSensorSpec::create();
  sensorSpecB->uuid = "B";
  SensorSetup sensorSpecificationsAB = {sensorSpecA, sensorSpecB};
  SensorSuite sensorSuiteAB =
      SensorFactory::createSensors(sceneNode1, sensorSpecificationsAB);
  sensorSuite_.merge(sensorSuiteAB);
  CORRADE_VERIFY(sensorSuite_.getSensors().size() == 2);

  // Add different uuid sensors to different nodes and assert increase
  auto sensorSpecC = CameraSensorSpec::create();
  sensorSpecC->uuid = "C";
  SensorSetup sensorSpecificationsC = {sensorSpecC};
  SensorSuite sensorSuiteC =
      SensorFactory::createSensors(sceneNode1, sensorSpecificationsC);
  sensorSuite_.merge(sensorSuiteC);
  CORRADE_VERIFY(sensorSuite_.getSensors().size() == 3);

  auto sensorSpecD = CameraSensorSpec::create();
  sensorSpecD->uuid = "D";
  SensorSetup sensorSpecificationsD = {sensorSpecD};
  SensorSuite sensorSuiteD =
      SensorFactory::createSensors(sceneNode2, sensorSpecificationsD);
  sensorSuite_.merge(sensorSuiteD);
  CORRADE_VERIFY(sensorSuite_.getSensors().size() == 4);
  // Add same uuid sensor to same node and assert that only one sensor was added
  auto sensorSpecE = CameraSensorSpec::create();
  sensorSpecE->uuid = "E";
  SensorSetup sensorSpecificationsEE = {sensorSpecE, sensorSpecE};
  SensorSuite sensorSuiteEE =
      SensorFactory::createSensors(sceneNode1, sensorSpecificationsEE);
  sensorSuite_.merge(sensorSuiteEE);
  CORRADE_VERIFY(sensorSuite_.getSensors().size() == 5);

  // Add same uuid sensors to different nodes and assert only one sensor was
  // added
  auto sensorSpecF = CameraSensorSpec::create();
  sensorSpecF->uuid = "F";
  SensorSetup sensorSpecificationsF = {sensorSpecF};
  SensorSuite sensorSuiteF1 =
      SensorFactory::createSensors(sceneNode1, sensorSpecificationsF);
  sensorSuite_.merge(sensorSuiteF1);
  CORRADE_VERIFY(sensorSuite_.getSensors().size() == 6);

  SensorSuite sensorSuiteF2 =
      SensorFactory::createSensors(sceneNode2, sensorSpecificationsF);
  sensorSuite_.merge(sensorSuiteF2);
  CORRADE_VERIFY(sensorSuite_.getSensors().size() == 6);

  // Remove sensors and assert that sensorSuite_ is empty
  sensorSuite_.clear();
  CORRADE_VERIFY(sensorSuite_.getSensors().size() == 0);
}

CORRADE_TEST_MAIN(SensorTest)
