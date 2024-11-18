// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/TestSuite/Compare/Numeric.h>
#include <Corrade/TestSuite/Tester.h>
#include <Corrade/Utility/FormatStl.h>
#include "esp/core/Configuration.h"
#include "esp/core/Esp.h"

using namespace esp::core::config;
namespace Cr = Corrade;

namespace {

struct ConfigurationTest : Cr::TestSuite::Tester {
  explicit ConfigurationTest();

  /**
   * @brief A Configuration is a hierarchical storage structure. This
   * recursively builds a subconfiguration hierarchy within the passed @p config
   * using the given values.
   * @param countPerDepth Number of subconfigs to build for each depth layer.
   * @param curDepth Current recursion depth.
   * @param totalDepth Total recursion depth +1 == total subconfig hierarchy
   * depth of given @p config counting base layer (add 1 because head
   * recursion).
   * @param config The Configuration in which to build the subconfig hierarchy.
   */
  void buildSubconfigsInConfig(int countPerDepth,
                               int curDepth,
                               int totalDepth,
                               Configuration::ptr& config);

  /**
   * @brief A Configuration is a hierarchical storage structure. This
   * recursively verifies the passed @p config has the subconfig hierarchy
   * structure as described by the given values.
   * @param countPerDepth Number of subconfigs that should be preseent in each
   * depth layer.
   * @param curDepth Current recursion depth.
   * @param totalDepth Total recursion depth +1 == total subconfig hierarchy
   * depth of given @p config counting base layer (add 1 because head
   * recursion).
   * @param config The Configuration holding the subconfig hierarchy we are
   * trying to verify.
   */
  void verifySubconfigTree(int countPerDepth,
                           int curDepth,
                           int totalDepth,
                           Configuration::cptr& config) const;

  /**
   * @brief Recursively add or modify the string value of the given key in every
   * subconfig of @p config with the given value
   */
  void addOrModSubconfigTreeVals(const std::string& key,
                                 const std::string& val,
                                 Configuration::ptr& config);

  /**
   * @brief Recursively verify that the given key exists in every subconfig of
   * @p config with the appropriately modified version of the given value.
   */
  void verifySubconfigTreeVals(const std::string& key,
                               const std::string& val,
                               Configuration::cptr& config) const;

  /**
   * @brief Verifies that the two passed Configurations hold identical
   * subconfig hierarchies for both values and structure.
   */
  void compareSubconfigs(Configuration::cptr& src, Configuration::cptr& target);

  /**
   * @brief Basic test of Configuration functionality
   */
  void TestConfiguration();

  /**
   * @brief Test Configuration comparisons between value types requiring fuzzy
   * logic (i.e. doubles). All additional numeric types added in the future that
   * require fuzzy comparison should have their fuzzy equality bounds tested
   * here.
   */
  void TestConfigurationFuzzyVals();

  /**
   * @brief Test Configuration find capability. Find returns a list of
   * subconfiguration keys required to find a particular key
   */
  void TestSubconfigFind();

  /**
   * @brief Test Configuration find, merge and edit capability. Builds
   * subconfig hierarchy, finds a desired subconfig, duplicates and merges it
   * with another and verifies results.
   */
  void TestSubconfigFindAndMerge();

  /**
   * @brief Test Configuration subconfig filtering. Builds two identical
   * subconfigs, modifies one such that it has changed values and also added
   * values, then filters it using original copy and verifies that only
   * new/different values remains.
   */
  void TestSubconfigFilter();

  esp::logging::LoggingContext loggingContext_;
};  // struct ConfigurationTest

ConfigurationTest::ConfigurationTest() {
  addTests({
      &ConfigurationTest::TestConfiguration,
      &ConfigurationTest::TestConfigurationFuzzyVals,
      &ConfigurationTest::TestSubconfigFind,
      &ConfigurationTest::TestSubconfigFindAndMerge,
      &ConfigurationTest::TestSubconfigFilter,
  });
}

void ConfigurationTest::buildSubconfigsInConfig(int countPerDepth,
                                                int curDepth,
                                                int totalDepth,
                                                Configuration::ptr& config) {
  if (curDepth == totalDepth) {
    return;
  }
  std::string breadcrumb = config->get<std::string>("breadcrumb");
  for (int i = 0; i < countPerDepth; ++i) {
    const std::string subconfigKey = Cr::Utility::formatString(
        "breadcrumb_{}_depth_{}_subconfig_{}", breadcrumb, curDepth, i);
    Configuration::ptr newCfg =
        config->editSubconfig<Configuration>(subconfigKey);
    // set the values within the new subconfig that describe the subconfig's
    // status
    newCfg->set("depth", curDepth);
    newCfg->set("iter", i);
    newCfg->set("breadcrumb", Cr::Utility::formatString("{}{}", breadcrumb, i));
    newCfg->set("key", subconfigKey);

    buildSubconfigsInConfig(countPerDepth, curDepth + 1, totalDepth, newCfg);
  }
}  // ConfigurationTest::buildSubconfigsInConfig

void ConfigurationTest::verifySubconfigTree(int countPerDepth,
                                            int curDepth,
                                            int totalDepth,
                                            Configuration::cptr& config) const {
  if (curDepth == totalDepth) {
    return;
  }
  std::string breadcrumb = config->get<std::string>("breadcrumb");
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
    CORRADE_COMPARE(newCfg->get<std::string>("breadcrumb"),
                    Cr::Utility::formatString("{}{}", breadcrumb, i));
    CORRADE_COMPARE(newCfg->get<std::string>("key"), subconfigKey);
    // Check into tree
    verifySubconfigTree(countPerDepth, curDepth + 1, totalDepth, newCfg);
  }
  CORRADE_COMPARE(config->getNumSubconfigs(), countPerDepth);

}  // ConfigurationTest::verifySubconfigTree

void ConfigurationTest::compareSubconfigs(Configuration::cptr& src,
                                          Configuration::cptr& target) {
  // verify target has at least as many subconfigs that src has
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

}  // ConfigurationTest::compareSubconfigs

void ConfigurationTest::addOrModSubconfigTreeVals(const std::string& key,
                                                  const std::string& val,
                                                  Configuration::ptr& config) {
  std::string newVal = val;
  if (config->hasKeyToValOfType(key,
                                esp::core::config::ConfigValType::String)) {
    newVal =
        Cr::Utility::formatString("{}_{}", config->get<std::string>(key), val);
  }
  config->set(key, newVal);
  auto cfgIterPair = config->getSubconfigIterator();
  // Get subconfig keys
  const auto& subsetKeys = config->getSubconfigKeys();
  for (const auto& subKey : subsetKeys) {
    auto subconfig = config->editSubconfig<Configuration>(subKey);
    addOrModSubconfigTreeVals(key, val, subconfig);
  }
}  // ConfigurationTest::addOrModSubconfigTreeVals

void ConfigurationTest::verifySubconfigTreeVals(
    const std::string& key,
    const std::string& val,
    Configuration::cptr& config) const {
  CORRADE_VERIFY(
      config->hasKeyToValOfType(key, esp::core::config::ConfigValType::String));
  std::string testVal = config->get<std::string>(key);
  std::size_t foundLoc = testVal.find(val);
  CORRADE_VERIFY(foundLoc != std::string::npos);
  auto srcIterConfigPair = config->getSubconfigIterator();
  for (auto& cfgIter = srcIterConfigPair.first;
       cfgIter != srcIterConfigPair.second; ++cfgIter) {
    Configuration::cptr subCfg = cfgIter->second;
    verifySubconfigTreeVals(key, val, subCfg);
  }
}  // ConfigurationTest::verifySubconfigTreeVals

void ConfigurationTest::TestConfiguration() {
  Configuration cfg;
  cfg.set("myBool", true);
  cfg.set("myInt", 10);
  cfg.set("myFloatToDouble", 1.2f);
  cfg.set("myVec2", Mn::Vector2{1.0, 2.0});
  cfg.set("myVec2i", Mn::Vector2i{30, 40});
  cfg.set("myVec3", Mn::Vector3{1.0, 2.0, 3.0});
  cfg.set("myVec4", Mn::Vector4{1.0, 2.0, 3.0, 4.0});
  cfg.set("myQuat", Mn::Quaternion{{1.0, 2.0, 3.0}, 0.1});
  cfg.set("myMat3", Mn::Matrix3(Mn::Math::IdentityInit));
  cfg.set("myMat4", Mn::Matrix4(Mn::Math::IdentityInit));
  cfg.set("myRad", Mn::Rad{1.23});
  cfg.set("myDeg", Mn::Deg{35.6});
  cfg.set("myString", "test");

  CORRADE_VERIFY(cfg.hasValue("myBool"));
  CORRADE_VERIFY(cfg.hasValue("myInt"));
  CORRADE_VERIFY(cfg.hasValue("myFloatToDouble"));
  CORRADE_VERIFY(cfg.hasValue("myVec2"));
  CORRADE_VERIFY(cfg.hasValue("myVec2i"));
  CORRADE_VERIFY(cfg.hasValue("myVec3"));
  CORRADE_VERIFY(cfg.hasValue("myMat3"));
  CORRADE_VERIFY(cfg.hasValue("myVec4"));
  CORRADE_VERIFY(cfg.hasValue("myQuat"));
  CORRADE_VERIFY(cfg.hasValue("myMat3"));
  CORRADE_VERIFY(cfg.hasValue("myRad"));
  CORRADE_VERIFY(cfg.hasValue("myDeg"));
  CORRADE_VERIFY(cfg.hasValue("myString"));

  CORRADE_COMPARE(cfg.get<bool>("myBool"), true);
  CORRADE_COMPARE(cfg.get<int>("myInt"), 10);
  CORRADE_COMPARE(cfg.get<double>("myFloatToDouble"), 1.2f);
  CORRADE_COMPARE(cfg.get<Mn::Vector2>("myVec2"), Mn::Vector2(1.0, 2.0));
  CORRADE_COMPARE(cfg.get<Mn::Vector2i>("myVec2i"), Mn::Vector2i(30, 40));
  CORRADE_COMPARE(cfg.get<Mn::Vector3>("myVec3"), Mn::Vector3(1.0, 2.0, 3.0));
  CORRADE_COMPARE(cfg.get<Mn::Vector4>("myVec4"),
                  Mn::Vector4(1.0, 2.0, 3.0, 4.0));
  CORRADE_COMPARE(cfg.get<Mn::Quaternion>("myQuat"),
                  Mn::Quaternion({1.0, 2.0, 3.0}, 0.1));
  CORRADE_COMPARE(cfg.get<Mn::Rad>("myRad"), Mn::Rad(1.23));
  CORRADE_COMPARE(cfg.get<Mn::Deg>("myDeg"), Mn::Deg(35.6));

  for (int i = 0; i < 3; ++i) {
    CORRADE_COMPARE(cfg.get<Mn::Matrix3>("myMat3").row(i)[i], 1);
  }
  for (int i = 0; i < 4; ++i) {
    CORRADE_COMPARE(cfg.get<Mn::Matrix4>("myMat4").row(i)[i], 1);
  }

  CORRADE_COMPARE(cfg.get<std::string>("myString"), "test");
}  // ConfigurationTest::TestConfiguration test

void ConfigurationTest::TestConfigurationFuzzyVals() {
  Configuration cfg;

  // Specify values to test
  cfg.set("fuzzyTestVal0", 1.0);
  cfg.set("fuzzyTestVal1", 1.0 + Mn::Math::TypeTraits<double>::epsilon() / 2);
  // Scale the epsilon to be too big to be seen as the same.
  cfg.set("fuzzyTestVal2", 1.0 + Mn::Math::TypeTraits<double>::epsilon() * 4);

  CORRADE_VERIFY(cfg.hasValue("fuzzyTestVal0"));
  CORRADE_VERIFY(cfg.hasValue("fuzzyTestVal1"));
  CORRADE_VERIFY(cfg.hasValue("fuzzyTestVal2"));
  // Verify very close doubles are considered sufficiently close by fuzzy
  // compare
  CORRADE_COMPARE(cfg.get("fuzzyTestVal0"), cfg.get("fuzzyTestVal1"));

  // verify very close but not-quite-close enough doubles are considered
  // different by magnum's fuzzy compare
  CORRADE_COMPARE_AS(cfg.get("fuzzyTestVal0"), cfg.get("fuzzyTestVal2"),
                     Cr::TestSuite::Compare::NotEqual);

}  // ConfigurationTest::TestConfigurationFuzzyVals

// Test configuration find functionality
void ConfigurationTest::TestSubconfigFind() {
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
void ConfigurationTest::TestSubconfigFindAndMerge() {
  Configuration::ptr cfg = Configuration::create();
  CORRADE_VERIFY(cfg);
  // Set base value for parent iteration
  cfg->set("breadcrumb", "");
  int depthPlus1 = 5;
  int count = 3;
  // Build subconfig tree 4 levels deep, 3 subconfigs per config
  buildSubconfigsInConfig(count, 0, depthPlus1, cfg);
  Configuration::cptr const_cfg =
      std::const_pointer_cast<const Configuration>(cfg);
  // Verify subconfig tree structure using const views
  verifySubconfigTree(count, 0, depthPlus1, const_cfg);

  // grab a subconfig, edit it and save it with a different key
  // use find to get key path
  const std::string keyToFind = "breadcrumb_212_depth_3_subconfig_2";
  // Find breadcrumb path of keys into subconfigs that lead to keyToFind
  std::vector<std::string> subconfigKeyPath = cfg->findValue(keyToFind);
  // Display breadcrumbs
  std::string resStr = Cr::Utility::formatString(
      "Vector of desired subconfig keys to get to '{}' "
      "subconfig is size : {}",
      keyToFind, subconfigKeyPath.size());
  for (const auto& key : subconfigKeyPath) {
    Cr::Utility::formatInto(resStr, resStr.size(), "\n\t{}", key);
  }
  ESP_DEBUG() << resStr;
  // Verify there are 4 layers of subconfigs to keyToFind
  CORRADE_COMPARE(subconfigKeyPath.size(), 4);
  // Verify the last entry holds keyToFind
  CORRADE_COMPARE(subconfigKeyPath[3], keyToFind);
  // Procede through the subconfig layers and find the desired target config
  Configuration::cptr viewConfig = cfg;
  Configuration::cptr lastConfig = cfg;
  for (const std::string& key : subconfigKeyPath) {
    Configuration::cptr tempConfig = viewConfig->getSubconfigView(key);
    CORRADE_COMPARE(tempConfig->get<std::string>("key"), key);
    lastConfig = viewConfig;
    viewConfig = tempConfig;
  }

  // By here lastConfig is the parent of viewConfig, the config we want at
  // keyToFind
  // We make a copy of viewConfig and verify the copy has the same values as the
  // original
  Configuration::ptr cfgToEdit =
      lastConfig->getSubconfigCopy<Configuration>(keyToFind);
  CORRADE_VERIFY(*cfgToEdit == *viewConfig);
  // Now modify new subconfig copy and verify the original viewConfig is not
  // also modified
  const std::string newKey = "edited_subconfig";
  cfgToEdit->set("key", newKey);
  cfgToEdit->set("depth", 0);
  cfgToEdit->set("breadcrumb", "");
  cfgToEdit->set("test_string", "this is an added test string");

  // Added edited subconfig to base config (top of hierarchy)
  cfg->setSubconfigPtr(newKey, cfgToEdit);
  CORRADE_VERIFY(cfg->hasSubconfig(newKey));
  // Get an immutable view of this subconfig and verify the data it holds
  Configuration::cptr cfgToVerify = cfg->getSubconfigView(newKey);
  CORRADE_COMPARE(cfgToVerify->get<std::string>("breadcrumb"), "");
  CORRADE_COMPARE(cfgToVerify->get<std::string>("test_string"),
                  "this is an added test string");
  CORRADE_COMPARE(cfgToVerify->get<std::string>("key"), newKey);
  CORRADE_COMPARE(cfgToVerify->get<int>("depth"), 0);
  // Make sure original was not modified
  CORRADE_VERIFY(viewConfig->get<std::string>("breadcrumb") != "");
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
  cfgToOverwrite->set("breadcrumb", "1123");
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
  CORRADE_COMPARE(cfgToVerifyOverwrite->get<std::string>("breadcrumb"), "");
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
  CORRADE_COMPARE(cfgToVerifyOverwrite->get<std::string>("myString"), "test");
  // Verify overwrite performed properly
  compareSubconfigs(cfgToVerify, cfgToVerifyOverwrite);
}

void ConfigurationTest::TestSubconfigFilter() {
  Configuration::ptr cfg = Configuration::create();
  CORRADE_VERIFY(cfg);
  // Build and verify hierarchy
  int depthPlus1 = 5;
  int count = 3;
  // build two identical subconfigs and place within base Configuration
  const std::string filterKey = "cfgToFilterBy";
  // Get actual subconfig
  Configuration::ptr cfgToFilterBy =
      cfg->editSubconfig<Configuration>(filterKey);
  // add some root values will be present in 'modified' subconfig too
  cfgToFilterBy->set("baseStrKey", "filterKey");
  cfgToFilterBy->set("baseIntKey", 1);
  cfgToFilterBy->set("baseBoolKey", true);
  cfgToFilterBy->set("baseDoubleKey", 3.14);
  cfgToFilterBy->set("baseStrKeyToMod", "modFilterKey");
  cfgToFilterBy->set("baseIntKeyToMod", 11);
  cfgToFilterBy->set("baseBoolKeyToMod", true);
  cfgToFilterBy->set("baseDoubleKeyToMod", 2.718);
  // Build subconfig tree 4 levels deep, 3 subconfigs per config
  buildSubconfigsInConfig(count, 0, depthPlus1, cfgToFilterBy);

  const std::string modKey = "modifiedSubconfig";
  // get copy of original filter config
  Configuration::ptr cfgToModify =
      cfg->getSubconfigCopy<Configuration>(filterKey);
  // move copy into cfg
  cfg->setSubconfigPtr<Configuration>(modKey, cfgToModify);
  // retrieve pointer to new copy
  cfgToModify = cfg->editSubconfig<Configuration>(modKey);

  // Build subconfig tree 4 levels deep, 3 subconfigs per config
  buildSubconfigsInConfig(count, 0, depthPlus1, cfgToModify);

  // compare both configs and verify they are equal
  {
    Configuration::cptr baseCompareCfg =
        std::static_pointer_cast<const Configuration>(cfgToFilterBy);
    Configuration::cptr modCompareCfg =
        std::static_pointer_cast<const Configuration>(cfgToModify);
    // verify both subconfigs are the same
    compareSubconfigs(baseCompareCfg, modCompareCfg);
  }

  // modify and save appropriate subconfig, filter using base subconfig, and
  // then verify expected results
  // Modify values that are also present in cfgToFilterBy
  cfgToModify->set("baseStrKeyToMod", "_modified");
  cfgToModify->set("baseIntKeyToMod", 111);
  cfgToModify->set("baseBoolKeyToMod", false);
  cfgToModify->set("baseDoubleKeyToMod", 1234.5);
  // Add 'mod' string to end of each config's breadcrumb string
  addOrModSubconfigTreeVals("breadcrumb", "mod", cfgToModify);
  // Add new string to each config
  addOrModSubconfigTreeVals("newStrValue", "new string", cfgToModify);
  // Add values that are not present in cfgToFilterBy
  cfgToModify->set("myBool", true);
  cfgToModify->set("myInt", 10);
  cfgToModify->set("myFloatToDouble", 1.2f);
  cfgToModify->set("myVec2", Mn::Vector2{1.0, 2.0});
  cfgToModify->set("myVec3", Mn::Vector3{1.0, 2.0, 3.0});
  cfgToModify->set("myVec4", Mn::Vector4{1.0, 2.0, 3.0, 4.0});
  cfgToModify->set("myQuat", Mn::Quaternion{{1.0, 2.0, 3.0}, 0.1});
  cfgToModify->set("myRad", Mn::Rad{1.23});
  cfgToModify->set("myString", "test");

  // Now filter the config to not have any data shared ith cfgToFilterBy
  cfgToModify->filterFromConfig(cfgToFilterBy);
  // Verify old shared values are gone
  CORRADE_VERIFY(!cfgToModify->hasValue("baseStrKey"));
  CORRADE_VERIFY(!cfgToModify->hasValue("baseIntKey"));
  CORRADE_VERIFY(!cfgToModify->hasValue("baseBoolKey"));
  CORRADE_VERIFY(!cfgToModify->hasValue("baseDoubleKey"));

  // Verify modified shared values are present
  CORRADE_VERIFY(cfgToModify->hasValue("baseStrKeyToMod"));
  CORRADE_VERIFY(cfgToModify->hasValue("baseIntKeyToMod"));
  CORRADE_VERIFY(cfgToModify->hasValue("baseBoolKeyToMod"));
  CORRADE_VERIFY(cfgToModify->hasValue("baseDoubleKeyToMod"));
  // ...and have expected values
  CORRADE_COMPARE(cfgToModify->get<std::string>("baseStrKeyToMod"),
                  "_modified");
  CORRADE_COMPARE(cfgToModify->get<int>("baseIntKeyToMod"), 111);
  CORRADE_COMPARE(cfgToModify->get<bool>("baseBoolKeyToMod"), false);
  CORRADE_COMPARE(cfgToModify->get<double>("baseDoubleKeyToMod"), 1234.5);

  // Verify modified values still present
  CORRADE_COMPARE(cfgToModify->get<bool>("myBool"), true);
  CORRADE_COMPARE(cfgToModify->get<int>("myInt"), 10);
  CORRADE_COMPARE(cfgToModify->get<double>("myFloatToDouble"), 1.2f);
  CORRADE_COMPARE(cfgToModify->get<Mn::Vector2>("myVec2"),
                  Mn::Vector2(1.0, 2.0));
  CORRADE_COMPARE(cfgToModify->get<Mn::Vector3>("myVec3"),
                  Mn::Vector3(1.0, 2.0, 3.0));
  CORRADE_COMPARE(cfgToModify->get<Mn::Vector4>("myVec4"),
                  Mn::Vector4(1.0, 2.0, 3.0, 4.0));
  CORRADE_COMPARE(cfgToModify->get<Mn::Quaternion>("myQuat"),
                  Mn::Quaternion({1.0, 2.0, 3.0}, 0.1));
  CORRADE_COMPARE(cfgToModify->get<Mn::Rad>("myRad"), Mn::Rad(1.23));
  CORRADE_COMPARE(cfgToModify->get<std::string>("myString"), "test");

  Configuration::cptr constModCfg =
      std::const_pointer_cast<const Configuration>(cfgToModify);
  // verify breadcrumb mod is present
  verifySubconfigTreeVals("breadcrumb", "mod", constModCfg);
  // verify newStrValue is present
  verifySubconfigTreeVals("newStrValue", "new string", constModCfg);
}  // ConfigurationTest::TestSubconfigFilter

}  // namespace

CORRADE_TEST_MAIN(ConfigurationTest)
