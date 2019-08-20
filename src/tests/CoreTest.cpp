// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <gtest/gtest.h>

#include "esp/core/Configuration.h"
#include "esp/core/esp.h"
#include "esp/io/json.h"

using namespace esp::core;

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
}
