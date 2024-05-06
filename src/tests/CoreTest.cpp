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

  void buildSubconfigsInConfig(int countPerDepth,
                               int curDepth,
                               int totalDepth,
                               Configuration::ptr& config);

  void verifySubconfigTree(int countPerDepth,
                           int curDepth,
                           int totalDepth,
                           Configuration::cptr& config);

  void compareSubconfigs(Configuration::cptr& src, Configuration::cptr& target);

  void TestConfiguration();

  /**
   * @brief Test Configuration find and subconfig edit capability. Find returns
   * a list of subconfiguration keys required to find a particular key
   */
  void TestConfigurationSubconfigFind();

  esp::logging::LoggingContext loggingContext_;
};  // struct CoreTest

CoreTest::CoreTest() {
  addTests({
      &CoreTest::TestConfiguration,
      &CoreTest::TestConfigurationSubconfigFind,
  });
}

void CoreTest::buildSubconfigsInConfig(int countPerDepth,
                                       int curDepth,
                                       int totalDepth,
                                       Configuration::ptr& config) {
  if (curDepth == totalDepth) {
    return;
  }
  std::string breadcrumb = config->get<std::string>("breakcrumb");
  for (int i = 0; i < countPerDepth; ++i) {
    const std::string subconfigKey = Cr::Utility::formatString(
        "breadcrumb_{}_depth_{}_subconfig_{}", breadcrumb, curDepth, i);
    Configuration::ptr newCfg =
        config->editSubconfig<Configuration>(subconfigKey);
    newCfg->set("depth", curDepth);
    newCfg->set("iter", i);
    newCfg->set("breakcrumb", Cr::Utility::formatString("{}{}", breadcrumb, i));
    newCfg->set("key", subconfigKey);

    buildSubconfigsInConfig(countPerDepth, curDepth + 1, totalDepth, newCfg);
  }
}  // CoreTest::buildSubconfigsInConfig

void CoreTest::verifySubconfigTree(int countPerDepth,
                                   int curDepth,
                                   int totalDepth,
                                   Configuration::cptr& config) {
  if (curDepth == totalDepth) {
    return;
  }
  std::string breadcrumb = config->get<std::string>("breakcrumb");
  for (int i = 0; i < countPerDepth; ++i) {
    const std::string subconfigKey = Cr::Utility::formatString(
        "breadcrumb_{}_depth_{}_subconfig_{}", breadcrumb, curDepth, i);
    // Verify subconfig with key exists
    CORRADE_VERIFY(config->hasSubconfig(subconfigKey));
    Configuration::cptr newCfg = config->getSubconfigView(subconfigKey);
    CORRADE_VERIFY(newCfg);
    // Verify subconfig depth value is as expected
    CORRADE_COMPARE(newCfg->get<int>("depth"), curDepth);
    // Verify iteration and parent iteration
    CORRADE_COMPARE(newCfg->get<int>("iter"), i);
    CORRADE_COMPARE(newCfg->get<std::string>("breakcrumb"),
                    Cr::Utility::formatString("{}{}", breadcrumb, i));
    CORRADE_COMPARE(newCfg->get<std::string>("key"), subconfigKey);
    // Check into tree
    verifySubconfigTree(countPerDepth, curDepth + 1, totalDepth, newCfg);
  }
  CORRADE_COMPARE(config->getNumSubconfigs(), countPerDepth);

}  // CoreTest::verifySubconfigTree

void CoreTest::compareSubconfigs(Configuration::cptr& src,
                                 Configuration::cptr& target) {
  // verify target has at least all the number of subconfigs that src has
  CORRADE_VERIFY(src->getNumSubconfigs() <= target->getNumSubconfigs());
  // verify that target has all the values that src has, and they are equal.
  auto srcIterValPair = src->getValuesIterator();
  for (auto& cfgIter = srcIterValPair.first; cfgIter != srcIterValPair.second;
       ++cfgIter) {
    CORRADE_VERIFY(cfgIter->second == target->get(cfgIter->first));
  }

  // Verify all subconfigs have the same keys - get begin/end iterators for src
  // subconfigs
  auto srcIterConfigPair = src->getSubconfigIterator();
  for (auto& cfgIter = srcIterConfigPair.first;
       cfgIter != srcIterConfigPair.second; ++cfgIter) {
    CORRADE_VERIFY(target->hasSubconfig(cfgIter->first));
    Configuration::cptr srcSubConfig = cfgIter->second;
    Configuration::cptr tarSubConfig = target->getSubconfigView(cfgIter->first);
    compareSubconfigs(srcSubConfig, tarSubConfig);
  }

}  // CoreTest::compareSubconfigs

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
  cfg.set("myMat4", Mn::Matrix4(Mn::Math::IdentityInit));
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
  for (int i = 0; i < 4; ++i) {
    CORRADE_COMPARE(cfg.get<Mn::Matrix4>("myMat4").row(i)[i], 1);
  }

  CORRADE_COMPARE(cfg.get<std::string>("myString"), "test");
}  // CoreTest::TestConfiguration test

// Test configuration find functionality
void CoreTest::TestConfigurationSubconfigFind() {
  {
    Configuration::ptr cfg = Configuration::create();
    Configuration::ptr baseCfg = cfg;
    // Build layers of subconfigs
    for (int i = 0; i < 10; ++i) {
      Configuration::ptr newCfg = cfg->editSubconfig<Configuration>(
          Cr::Utility::formatString("depth_{}_subconfig_{}", i + 1, i));
      newCfg->set("depth", i + 1);
      cfg = newCfg;
    }
    // Set deepest layer config to have treasure
    cfg->set("treasure", "this is the treasure!");
    // Find the treasure!
    std::vector<std::string> keyList = baseCfg->findValue("treasure");

    // Display breadcrumbs
    std::string resStr = Cr::Utility::formatString(
        "Vector of 'treasure' keys is size : {}", keyList.size());
    for (const auto& key : keyList) {
      Cr::Utility::formatInto(resStr, resStr.size(), "\n\t{}", key);
    }
    ESP_DEBUG() << resStr;
    // Verify the found treasure
    CORRADE_COMPARE(keyList.size(), 11);

    Configuration::cptr viewConfig = baseCfg;
    for (int i = 0; i < 10; ++i) {
      const std::string subconfigKey =
          Cr::Utility::formatString("depth_{}_subconfig_{}", i + 1, i);
      CORRADE_COMPARE(keyList[i], subconfigKey);
      // Verify that subconfig with given name exists
      CORRADE_VERIFY(viewConfig->hasSubconfig(subconfigKey));
      // retrieve actual subconfig
      Configuration::cptr newCfg = viewConfig->getSubconfigView(subconfigKey);
      // verity subconfig has correct depth value
      CORRADE_COMPARE(newCfg->get<int>("depth"), i + 1);
      viewConfig = newCfg;
    }
    CORRADE_VERIFY(viewConfig->hasValue("treasure"));
    CORRADE_COMPARE(viewConfig->get<std::string>("treasure"),
                    "this is the treasure!");

    // Verify pirate isn't found
    std::vector<std::string> notFoundKeyList = baseCfg->findValue("pirate");
    CORRADE_COMPARE(notFoundKeyList.size(), 0);
  }
  // Test subconfig edit/merge and save
  {
    Configuration::ptr cfg = Configuration::create();
    CORRADE_VERIFY(cfg);
    // Set base value for parent iteration
    cfg->set("breakcrumb", "");
    int depth = 5;
    int count = 3;
    // Build subconfig tree 4 levels deep, 3 subconfigs per config
    buildSubconfigsInConfig(count, 0, depth, cfg);
    Configuration::cptr const_cfg =
        std::const_pointer_cast<const Configuration>(cfg);
    // Verify subconfig tree structure using const views
    verifySubconfigTree(count, 0, depth, const_cfg);

    // grab a subconfig, edit it and save it with a different key
    // use find to get key path
    const std::string keyToFind = "breadcrumb_212_depth_3_subconfig_2";
    std::vector<std::string> subconfigPath = cfg->findValue(keyToFind);
    // Display breadcrumbs
    std::string resStr = Cr::Utility::formatString(
        "Vector of desired subconfig keys to get to '{}' "
        "subconfig is size : {}",
        keyToFind, subconfigPath.size());
    for (const auto& key : subconfigPath) {
      Cr::Utility::formatInto(resStr, resStr.size(), "\n\t{}", key);
    }
    ESP_DEBUG() << resStr;
    CORRADE_COMPARE(subconfigPath.size(), 4);
    CORRADE_COMPARE(subconfigPath[3], keyToFind);
    Configuration::cptr viewConfig = cfg;
    Configuration::cptr lastConfig = cfg;
    for (const std::string& key : subconfigPath) {
      Configuration::cptr tempConfig = viewConfig->getSubconfigView(key);
      CORRADE_COMPARE(tempConfig->get<std::string>("key"), key);
      lastConfig = viewConfig;
      viewConfig = tempConfig;
    }
    // By here lastConfig is the parent of the config we want, and viewConfig is
    // the config we are getting a copy of.
    Configuration::ptr cfgToEdit =
        lastConfig->getSubconfigCopy<Configuration>(keyToFind);
    const std::string newKey = "edited_subconfig";
    cfgToEdit->set("key", newKey);
    cfgToEdit->set("depth", 0);
    cfgToEdit->set("breakcrumb", "");
    cfgToEdit->set("test_string", "this is an added test string");
    // Added edited subconfig to base config
    cfg->setSubconfigPtr(newKey, cfgToEdit);
    CORRADE_VERIFY(cfg->hasSubconfig(newKey));
    Configuration::cptr cfgToVerify = cfg->getSubconfigView(newKey);
    CORRADE_COMPARE(cfgToVerify->get<std::string>("breakcrumb"), "");
    CORRADE_COMPARE(cfgToVerify->get<std::string>("test_string"),
                    "this is an added test string");
    CORRADE_COMPARE(cfgToVerify->get<std::string>("key"), newKey);
    CORRADE_COMPARE(cfgToVerify->get<int>("depth"), 0);
    // Make sure original was not modified
    CORRADE_VERIFY(viewConfig->get<std::string>("breakcrumb") != "");
    CORRADE_VERIFY(viewConfig->get<std::string>("key") != newKey);
    CORRADE_VERIFY(viewConfig->get<int>("depth") != 0);
    CORRADE_VERIFY(!viewConfig->hasValue("test_string"));

    // Now Test merge/Overwriting
    const std::string mergedKey = "merged_subconfig";
    Configuration::ptr cfgToOverwrite =
        cfg->editSubconfig<Configuration>(mergedKey);

    // first set some test values that will be clobbered
    cfgToOverwrite->set("key", mergedKey);
    cfgToOverwrite->set("depth", 11);
    cfgToOverwrite->set("breakcrumb", "1123");
    cfgToOverwrite->set("test_string", "this string will be clobbered");
    // Now add some values that won't be clobbered
    cfgToOverwrite->set("myBool", true);
    cfgToOverwrite->set("myInt", 10);
    cfgToOverwrite->set("myFloatToDouble", 1.2f);
    cfgToOverwrite->set("myVec2", Mn::Vector2{1.0, 2.0});
    cfgToOverwrite->set("myVec3", Mn::Vector3{1.0, 2.0, 3.0});
    cfgToOverwrite->set("myVec4", Mn::Vector4{1.0, 2.0, 3.0, 4.0});
    cfgToOverwrite->set("myQuat", Mn::Quaternion{{1.0, 2.0, 3.0}, 0.1});
    cfgToOverwrite->set("myRad", Mn::Rad{1.23});
    cfgToOverwrite->set("myString", "test");

    // Now overwrite the values from cfgToVerify
    cfgToOverwrite->overwriteWithConfig(cfgToVerify);

    CORRADE_VERIFY(cfg->hasSubconfig(mergedKey));
    // Verify all the overwritten values are correct
    Configuration::cptr cfgToVerifyOverwrite = cfg->getSubconfigView(mergedKey);
    CORRADE_COMPARE(cfgToVerifyOverwrite->get<std::string>("breakcrumb"), "");
    CORRADE_COMPARE(cfgToVerifyOverwrite->get<std::string>("test_string"),
                    "this is an added test string");
    CORRADE_COMPARE(cfgToVerifyOverwrite->get<std::string>("key"), newKey);
    CORRADE_COMPARE(cfgToVerifyOverwrite->get<int>("depth"), 0);
    // Verify original non-overwritten values are still present
    CORRADE_COMPARE(cfgToVerifyOverwrite->get<bool>("myBool"), true);
    CORRADE_COMPARE(cfgToVerifyOverwrite->get<int>("myInt"), 10);
    CORRADE_COMPARE(cfgToVerifyOverwrite->get<double>("myFloatToDouble"), 1.2f);
    CORRADE_COMPARE(cfgToVerifyOverwrite->get<Mn::Vector2>("myVec2"),
                    Mn::Vector2(1.0, 2.0));
    CORRADE_COMPARE(cfgToVerifyOverwrite->get<Mn::Vector3>("myVec3"),
                    Mn::Vector3(1.0, 2.0, 3.0));
    CORRADE_COMPARE(cfgToVerifyOverwrite->get<Mn::Vector4>("myVec4"),
                    Mn::Vector4(1.0, 2.0, 3.0, 4.0));
    CORRADE_COMPARE(cfgToVerifyOverwrite->get<Mn::Quaternion>("myQuat"),
                    Mn::Quaternion({1.0, 2.0, 3.0}, 0.1));
    CORRADE_COMPARE(cfgToVerifyOverwrite->get<Mn::Rad>("myRad"), Mn::Rad(1.23));

    // Verify overwrite performed properly
    compareSubconfigs(cfgToVerify, cfgToVerifyOverwrite);
  }

}  // CoreTest::TestConfigurationSubconfigFind test

}  // namespace

CORRADE_TEST_MAIN(CoreTest)
