// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <gtest/gtest.h>

#include "esp/assets/attributes/ObjectAttributes.h"
#include "esp/core/Configuration.h"
#include "esp/core/esp.h"
#include "esp/io/json.h"

using namespace esp::core;
using esp::assets::attributes::AbstractObjectAttributes;
using esp::assets::attributes::ObjectAttributes;

TEST(CoreTest, ConfigurationTest) {
  Configuration cfg;
  cfg.set("myInt", 10);
  cfg.set("myString", "test");
  EXPECT_TRUE(cfg.hasValue("myInt"));
  EXPECT_TRUE(cfg.hasValue("myString"));
  EXPECT_EQ(cfg.get<int>("myInt"), 10);
  EXPECT_EQ(cfg.get<std::string>("myString"), "test");
}

TEST(CoreTest, JsonTest) {
  std::string s = "{\"test\":[1, 2, 3, 4]}";
  const auto& json = esp::io::parseJsonString(s);
  std::vector<int> t;
  esp::io::toIntVector(json["test"], &t);
  EXPECT_EQ(t[1], 2);
  EXPECT_EQ(esp::io::jsonToString(json), "{\"test\":[1,2,3,4]}");

  // test attributes populating

  std::string attr_str =
      "{\"render mesh\": \"banana.glb\",\"join collision "
      "meshes\":false,\"mass\": 0.066,\"scale\": [2.0,2.0,2]}";

  const auto& jsonDoc = esp::io::parseJsonString(attr_str);

  // for function ptr placeholder
  using std::placeholders::_1;
  ObjectAttributes::ptr attributes = ObjectAttributes::create("temp");

  bool success = false;
  // test vector
  success = esp::io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonDoc, "scale",
      std::bind(&AbstractObjectAttributes::setScale, attributes, _1));
  EXPECT_EQ(success, true);
  EXPECT_EQ(attributes->getScale()[1], 2);

  // test double
  success = esp::io::jsonIntoSetter<double>(
      jsonDoc, "mass", std::bind(&ObjectAttributes::setMass, attributes, _1));
  EXPECT_EQ(success, true);
  EXPECT_EQ(attributes->getMass(), 0.066);

  // test bool
  success = esp::io::jsonIntoSetter<bool>(
      jsonDoc, "join collision meshes",
      std::bind(&ObjectAttributes::setJoinCollisionMeshes, attributes, _1));
  EXPECT_EQ(success, true);
  EXPECT_EQ(attributes->getJoinCollisionMeshes(), false);

  // test string
  success = esp::io::jsonIntoSetter<std::string>(
      jsonDoc, "render mesh",
      std::bind(&ObjectAttributes::setRenderAssetHandle, attributes, _1));
  EXPECT_EQ(success, true);
  EXPECT_EQ(attributes->getRenderAssetHandle(), "banana.glb");

  // test string
}
