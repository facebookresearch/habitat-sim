// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/TestSuite/Tester.h>
#include <Corrade/Utility/FormatStl.h>
#include "esp/core/Configuration.h"
#include "esp/core/Esp.h"

using namespace esp::core::config;
namespace Cr = Corrade;

namespace {

struct CoreTest : Cr::TestSuite::Tester {
  explicit CoreTest();

  void TestConfiguration();

  // void TestConfigurationSubConfigs();
  void TestConfigurationFind();

  esp::logging::LoggingContext loggingContext_;
};  // struct CoreTest

CoreTest::CoreTest() {
  addTests({
      &CoreTest::TestConfiguration,
      //&CoreTest::TestConfigurationSubConfigs,
      &CoreTest::TestConfigurationFind,
  });
}

void CoreTest::TestConfiguration() {
  Configuration cfg;
  cfg.set("myBool", true);
  cfg.set("myInt", 10);
  cfg.set("myFloatToDouble", 1.2f);
  cfg.set("myVec2", Mn::Vector2{1.0, 2.0});
  cfg.set("myVec3", Mn::Vector3{1.0, 2.0, 3.0});
  cfg.set("myVec4", Mn::Vector4{1.0, 2.0, 3.0, 4.0});
  cfg.set("myQuat", Mn::Quaternion{{1.0, 2.0, 3.0}, 0.1});
  cfg.set("myMat3", Mn::Matrix3(Mn::Math::IdentityInit));
  cfg.set("myRad", Mn::Rad{1.23});
  cfg.set("myString", "test");

  CORRADE_VERIFY(cfg.hasValue("myBool"));
  CORRADE_VERIFY(cfg.hasValue("myInt"));
  CORRADE_VERIFY(cfg.hasValue("myFloatToDouble"));
  CORRADE_VERIFY(cfg.hasValue("myVec2"));
  CORRADE_VERIFY(cfg.hasValue("myVec3"));
  CORRADE_VERIFY(cfg.hasValue("myMat3"));
  CORRADE_VERIFY(cfg.hasValue("myVec4"));
  CORRADE_VERIFY(cfg.hasValue("myQuat"));
  CORRADE_VERIFY(cfg.hasValue("myMat3"));
  CORRADE_VERIFY(cfg.hasValue("myRad"));
  CORRADE_VERIFY(cfg.hasValue("myString"));

  CORRADE_COMPARE(cfg.get<bool>("myBool"), true);
  CORRADE_COMPARE(cfg.get<int>("myInt"), 10);
  CORRADE_COMPARE(cfg.get<double>("myFloatToDouble"), 1.2f);
  CORRADE_COMPARE(cfg.get<Mn::Vector2>("myVec2"), Mn::Vector2(1.0, 2.0));
  CORRADE_COMPARE(cfg.get<Mn::Vector3>("myVec3"), Mn::Vector3(1.0, 2.0, 3.0));
  CORRADE_COMPARE(cfg.get<Mn::Vector4>("myVec4"),
                  Mn::Vector4(1.0, 2.0, 3.0, 4.0));
  CORRADE_COMPARE(cfg.get<Mn::Quaternion>("myQuat"),
                  Mn::Quaternion({1.0, 2.0, 3.0}, 0.1));
  CORRADE_COMPARE(cfg.get<Mn::Rad>("myRad"), Mn::Rad(1.23));

  for (int i = 0; i < 3; ++i) {
    CORRADE_COMPARE(cfg.get<Mn::Matrix3>("myMat3").row(i)[i], 1);
  }
  CORRADE_COMPARE(cfg.get<std::string>("myString"), "test");
}  // CoreTest::TestConfiguration test

// Test subconfig editing and merging
// void CoreTest::TestConfigurationSubConfigs() {

// }  // CoreTest::TestConfigurationSubConfigs test

// Test find functionality
void CoreTest::TestConfigurationFind() {
  Configuration::ptr cfg = Configuration::create();
  Configuration::ptr baseCfg = cfg;
  // Build layers of subconfigs
  std::string titleBase("subConfig_");
  for (int i = 0; i < 10; ++i) {
    Configuration::ptr newCfg = cfg->editSubconfig<Configuration>(
        Cr::Utility::formatString("subConfig_{}", i));
    newCfg->set("depth", i + 1);
    cfg = newCfg;
  }
  // Set deepest layer config to have treasure
  cfg->set("treasure", "this is the treasure!");
  // Find the treasure!
  std::vector<std::string> keyList = baseCfg->findValue("treasure");

  // Display breadcrumbs
  std::string resStr = Cr::Utility::formatString(
      "Vector of results is size : {}", keyList.size());
  for (const auto& key : keyList) {
    Cr::Utility::formatInto(resStr, resStr.size(), "\n\t{}", key);
  }
  ESP_DEBUG() << resStr;
  Configuration::cptr view_cfg = baseCfg;
  // Verify the found treasure
  CORRADE_COMPARE(keyList.size(), 11);
  for (int i = 0; i < 10; ++i) {
    const std::string subConfigKey =
        Cr::Utility::formatString("subConfig_{}", i);
    CORRADE_COMPARE(keyList[i], subConfigKey);
    // Verify that subconfig with given name exists
    CORRADE_VERIFY(view_cfg->hasSubconfig(subConfigKey));
    // retrieve actual subconfig
    Configuration::cptr newCfg = view_cfg->getSubconfigView(subConfigKey);
    // verity subconfig has correct depth value
    CORRADE_COMPARE(newCfg->get<int>("depth"), i + 1);
    view_cfg = newCfg;
  }
  CORRADE_VERIFY(view_cfg->hasValue("treasure"));
  CORRADE_COMPARE(view_cfg->get<std::string>("treasure"),
                  "this is the treasure!");

}  // CoreTest::TestConfigurationFind test

}  // namespace

CORRADE_TEST_MAIN(CoreTest)
