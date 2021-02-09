// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#include <Corrade/Utility/Directory.h>
#include <gtest/gtest.h>

#include "esp/scene/SceneManager.h"
#include "esp/scene/SceneNode.h"
#include "esp/sensor/Sensor.h"
#include "esp/sensor/SensorFactory.h"

#include "configure.h"

using namespace esp::sensor;
using namespace esp::scene;

// TODO: Add tests for different Sensors

TEST(SensorTest, SensorFactory) {
  SceneManager sceneManager_;
  SensorSuite sensorSuite_;

  int sceneID = sceneManager_.initSceneGraph();
  auto& sceneGraph = sceneManager_.getSceneGraph(sceneID);

  // retrieve root node
  auto& rootNode = sceneGraph.getRootNode();
  SceneNode sceneNode1 = rootNode.createChild();
  sceneNode1.setId(1);
  ASSERT(sceneNode1.getId() == 1);
  SceneNode sceneNode2 = rootNode.createChild();
  sceneNode2.setId(2);
  ASSERT(sceneNode2.getId() == 2);

  // Add different uuid sensors to same node and assert increase
  SensorSpec::ptr sensorSpecA = SensorSpec::create();
  sensorSpecA->uuid = "A";
  SensorSpec::ptr sensorSpecB = SensorSpec::create();
  sensorSpecB->uuid = "B";
  SensorSetup sensorSpecificationsAB = {sensorSpecA, sensorSpecB};
  SensorSuite sensorSuiteAB =
      SensorFactory::createSensors(sceneNode1, sensorSpecificationsAB);
  sensorSuite_.merge(sensorSuiteAB);
  ASSERT(sensorSuite_.getSensors().size() == 2);

  // Add different uuid sensors to different nodes and assert increase
  SensorSpec::ptr sensorSpecC = SensorSpec::create();
  sensorSpecC->uuid = "C";
  SensorSetup sensorSpecificationsC = {sensorSpecC};
  SensorSuite sensorSuiteC =
      SensorFactory::createSensors(sceneNode1, sensorSpecificationsC);
  sensorSuite_.merge(sensorSuiteC);
  ASSERT(sensorSuite_.getSensors().size() == 3);

  SensorSpec::ptr sensorSpecD = SensorSpec::create();
  sensorSpecD->uuid = "D";
  SensorSetup sensorSpecificationsD = {sensorSpecD};
  SensorSuite sensorSuiteD =
      SensorFactory::createSensors(sceneNode2, sensorSpecificationsD);
  sensorSuite_.merge(sensorSuiteD);
  ASSERT(sensorSuite_.getSensors().size() == 4);
  // Add same uuid sensor to same node and assert that only one sensor was added
  SensorSpec::ptr sensorSpecE = SensorSpec::create();
  sensorSpecE->uuid = "E";
  SensorSetup sensorSpecificationsEE = {sensorSpecE, sensorSpecE};
  SensorSuite sensorSuiteEE =
      SensorFactory::createSensors(sceneNode1, sensorSpecificationsEE);
  sensorSuite_.merge(sensorSuiteEE);
  ASSERT(sensorSuite_.getSensors().size() == 5);

  // Add same uuid sensors to different nodes and assert only one sensor was
  // added
  SensorSpec::ptr sensorSpecF = SensorSpec::create();
  sensorSpecF->uuid = "F";
  SensorSetup sensorSpecificationsF = {sensorSpecF};
  SensorSuite sensorSuiteF1 =
      SensorFactory::createSensors(sceneNode1, sensorSpecificationsF);
  sensorSuite_.merge(sensorSuiteF1);
  ASSERT(sensorSuite_.getSensors().size() == 6);

  SensorSuite sensorSuiteF2 =
      SensorFactory::createSensors(sceneNode2, sensorSpecificationsF);
  sensorSuite_.merge(sensorSuiteF2);
  ASSERT(sensorSuite_.getSensors().size() == 6);

  // Remove sensors and assert that sensorSuite_ is empty
  sensorSuite_.clear();
  ASSERT(sensorSuite_.getSensors().size() == 0);
}
